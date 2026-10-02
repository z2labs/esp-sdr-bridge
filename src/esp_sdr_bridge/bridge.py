"""ESP32-S3 esp-sdr IQ bridge: SpyServer (SDR++, SDR#) and rtl_tcp servers in one process.

    esp-sdr-bridge --port COM4        (or: python -m esp_sdr_bridge ...) [--spyserver 0.0.0.0:5555] [--rtltcp 0.0.0.0:1234] [--ppm 0] [--fake]

Firmware side: esp-sdr IQS stream (two-stage FIR DDC on the S3 DSP core, fs/4 IF, 16 MS/s ring).
Rates: 250 kS/s (8-bit link), 125 / 62.5 kS/s (16-bit link). Samples are kept in FIR units
(10-bit ADC * 32, int16 full scale) so the level does not depend on rate, link bits or shift.

ESP-SDR: https://github.com/ESPARGOS/esp-sdr
Turbo Mode developed by Zoltan Doczi from https://www.z2labs.io
"""
import argparse, math, socket, struct, threading, time, zlib
import numpy as np

FS = 16_000_000                     # ring rate in IQ mode
MODE = 2                            # IQS mode: 0 FIR at 0 Hz IF, 2 FIR with fs/4 IF
RATES = {250000: (6, 64, 8), 125000: (6, 128, 16), 62500: (6, 256, 16)}   # out rate -> (rate code, dec, link bits)
FMIN, FMAX = 2_204_000_000, 2_804_000_000
S3_GAIN = [(0, 0.0), (10, 10.3), (20, 21.1), (40, 30.2), (50, 41.9), (60, 51.3), (70, 61.3), (82, 73.3)]  # index, dB
R820T_GAINS = [0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328, 338,
               364, 372, 386, 402, 421, 434, 439, 445, 480, 496]


def s3_gain_db(idx):
    return float(np.interp(idx, [a for a, _ in S3_GAIN], [b for _, b in S3_GAIN]))


def out_shift(idx):
    # one bit less shift per 6 dB less analog gain keeps the noise floor at a few LSB of an 8-bit stream
    return int(max(0, min(4, round(4 - (51.3 - s3_gain_db(idx)) / 6.02))))


class Sinks:
    def __init__(self): self.l = []; self.lock = threading.Lock()
    def add(self, f):
        with self.lock: self.l.append(f)
    def remove(self, f):
        with self.lock:
            if f in self.l: self.l.remove(f)
    def __call__(self, z, g):
        with self.lock: l = list(self.l)
        for f in l:
            try: f(z, g)
            except OSError: self.remove(f)


class Device:
    """Serial link to the ESP32-S3; restarts the IQS stream on every parameter change."""
    def __init__(self, port, out, ppm=0.0):
        import serial
        self.s = serial.Serial(port, 2000000, timeout=0.05); time.sleep(0.4); self.s.reset_input_buffer()
        self.out = out; self.lock = threading.Lock(); self.want = None; self.cur = None
        self.freq = 2_450_000_000; self.gain = 60; self.rate = 250000; self.ppm = ppm; self.dc = 0j
        self.stats = dict(frames=0, crc=0, gaps=0, lost=0); self.next_idx = None
        threading.Thread(target=self.run, daemon=True).start()

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items(): setattr(self, k, v)
            fk = max(FMIN // 1000, min(FMAX // 1000, round(self.freq * (1 - self.ppm * 1e-6) / 1e3)))
            self.want = (fk, int(self.gain), min(RATES, key=lambda r: abs(r - self.rate)))

    def cmd(self, c, pre="OK", t=1.5):
        self.s.write((c + "\n").encode()); end = time.time() + t; buf = b""
        while time.time() < end:
            buf += self.s.read(256)
            for l in buf.split(b"\n"):
                if l.startswith(pre.encode()) or l.startswith(b"ERR"): return l.decode(errors="replace")
        return None

    def stop_stream(self):
        self.s.write(b"\n"); t = time.time() + 2; buf = b""
        while time.time() < t:
            buf += self.s.read(65536)
            if b"IQSEND" in buf and buf.rfind(b"\n") > buf.rfind(b"IQSEND"): break
        time.sleep(0.05); self.s.reset_input_buffer()

    def run(self):
        buf = b""; streaming = False
        while True:
            with self.lock: want = self.want
            if want and want != self.cur:
                if streaming: self.stop_stream()
                f, g, r = want; code, dec, bits = RATES[r]
                # LO fs/4 below the wanted frequency, the chip shifts by +fs/4 before the FIR (mode 2):
                # the LO leakage / 1/f hump at 0 Hz IF is outside the output band
                mhz, khz = divmod(f - (4000 if MODE == 2 else 0), 1000)
                sh = out_shift(g) if bits == 8 else 0
                print(f"[dev] tune {f / 1000:.3f} MHz: LO {mhz} MHz + {khz} kHz, gain idx {g}, {r} S/s "
                      f"(dec {dec}, {bits}-bit link, shift {sh})", flush=True)
                self.cmd(f"FREQ {mhz}"); self.cmd(f"FOFS {khz}"); self.cmd(f"GAIN MANUAL {g}")
                self.s.write(f"IQS 0 {dec} {bits} {code} {sh} {MODE}\n".encode())
                buf = b""; streaming = True; self.cur = want; self.next_idx = None
            d = self.s.read(65536)
            if not d: continue
            buf += d
            while True:
                i = buf.find(b"IQS1")
                if i < 0: buf = buf[-3:]; break
                if len(buf) < i + 24: buf = buf[i:]; break
                magic, fr, sidx, ns, bits, fl, dd, g, sh = struct.unpack("<IIQHBBHBB", buf[i:i + 24])
                if bits not in (8, 16): buf = buf[i + 4:]; continue
                L = 24 + ns * bits // 4 + 4
                if len(buf) < i + L: buf = buf[i:]; break
                blob = buf[i:i + L]; buf = buf[i + L:]
                if zlib.crc32(blob[:-4]) != struct.unpack("<I", blob[-4:])[0]: self.stats["crc"] += 1; continue
                if dd != RATES[self.cur[2]][1]: continue                # tail of the previous stream after a retune
                if self.next_idx is not None and sidx != self.next_idx:
                    self.stats["gaps"] += 1; self.stats["lost"] += sidx - self.next_idx
                self.next_idx = sidx + ns; self.stats["frames"] += 1
                a = np.frombuffer(blob[24:-4], np.int8 if bits == 8 else "<i2").astype(np.float32) * float(1 << sh)
                z = a[0::2] + 1j * a[1::2]
                self.dc = 0.98 * self.dc + 0.02 * z.mean()          # residual DC
                z = np.conj(z - self.dc).astype(np.complex64)       # S3: RF above LO is negative
                self.out(z, self.cur[1])


class Fake:
    """Tone at +40 kHz plus noise, for protocol tests without hardware."""
    def __init__(self, out, ppm=0.0):
        self.out = out; self.rate = 250000; self.gain = 60; self.stats = {}; self.n = 0
        threading.Thread(target=self.run, daemon=True).start()
    def set(self, **kw):
        for k, v in kw.items(): setattr(self, k, v)
        self.rate = min(RATES, key=lambda r: abs(r - self.rate))
    def run(self):
        rng = np.random.default_rng()
        while True:
            m = self.rate // 50; t = (self.n + np.arange(m)) / self.rate; self.n += m
            z = 4000 * np.exp(2j * np.pi * 40e3 * t) + 20 * (rng.standard_normal(m) + 1j * rng.standard_normal(m))
            self.out(z.astype(np.complex64), int(self.gain)); time.sleep(0.02)


def recv_exact(c, n):
    b = b""
    while len(b) < n:
        d = c.recv(n - len(b))
        if not d: raise OSError("closed")
        b += d
    return b


# ---------------------------------------------------------------- SpyServer
SS_PROTO = (2 << 24) | (0 << 16) | 1700
SS_DEV_RTLSDR = 3            # SDR++ only knows Airspy One / HF+ / RTL-SDR; RTL-SDR: no gain-dependent digital gain
SS_SERIAL = 0xE5D3_5301


def spyserver_client(c, addr, dev, sinks, info):
    print("[ss] client", addr, flush=True)
    st = dict(fmt=2, mode=1, on=False, seq=0, dig=0)
    wl = threading.Lock()

    def send(mtype, stype, body, flags=0):
        with wl:
            c.sendall(struct.pack("<5I", SS_PROTO, mtype | (flags << 16), stype, st["seq"], len(body)) + body)
            st["seq"] = (st["seq"] + 1) & 0xFFFFFFFF

    def sync():
        f = int(dev.freq)
        send(1, 0, struct.pack("<9I", 1, int(dev.gain), f, f, f, FMIN, FMAX, FMIN, FMAX))

    def sink(z, g):
        if not (st["on"] and st["mode"] & 1): return
        v = z.view(np.float32)
        if st["fmt"] == 1:      # uint8, digital gain in the header flags (client divides it out)
            D = int(round(6.0206 * (8 - out_shift(g))))
            u = v * (128 / 32768 * 10 ** (D / 20)) + 128 + (np.random.random(v.size) - np.random.random(v.size))
            send(100, 1, np.clip(np.rint(u), 0, 255).astype(np.uint8).tobytes(), D)
        elif st["fmt"] == 4:
            send(103, 1, (v / 32768).astype("<f4").tobytes())
        else:
            send(101, 1, np.clip(np.rint(v), -32768, 32767).astype("<i2").tobytes())

    sinks.add(sink)
    try:
        while True:
            ct, n = struct.unpack("<II", recv_exact(c, 8))
            if n > 65536: break
            body = recv_exact(c, n)
            if ct == 0:          # HELLO: uint32 version + client name
                ver = struct.unpack("<I", body[:4])[0] if n >= 4 else 0
                print(f"[ss] hello {body[4:].decode(errors='replace')!r} proto {ver:08x}", flush=True)
                send(0, 0, struct.pack("<12I", SS_DEV_RTLSDR, SS_SERIAL, FS, FS, 8, 1, 82,
                                       FMIN, FMAX, 10, 6, 0))   # rates FS/2^6 .. FS/2^8
                sync()
            elif ct == 2 and n >= 8:
                s, v = struct.unpack("<II", body[:8])
                print(f"[ss] set {s} = {v}", flush=True)
                if s == 0: st["mode"] = v
                elif s == 1: st["on"] = bool(v)
                elif s == 2: dev.set(gain=max(0, min(82, v)))
                elif s == 100: st["fmt"] = v if v in (1, 2, 4) else 2
                elif s == 101: dev.set(freq=max(FMIN, min(FMAX, v)))
                elif s == 102:
                    r = FS >> min(max(v, 6), 8); dev.set(rate=r)
                elif s == 103: st["dig"] = v
                elif s >= 200: info.setdefault("fft_req", {})[s] = v   # FFT stream: not yet
                sync()
            elif ct == 3:
                send(2, 0, b"")
    except (OSError, struct.error):
        pass
    sinks.remove(sink)
    try: c.close()
    except OSError: pass
    print("[ss] client gone", addr, getattr(dev, "stats", {}), flush=True)


# ---------------------------------------------------------------- rtl_tcp
def rtltcp_client(c, addr, dev, sinks, info):
    print("[rtl] client", addr, flush=True)
    c.sendall(b"RTL0" + struct.pack(">II", 5, len(R820T_GAINS)))   # 5 = R820T

    def sink(z, g):
        v = z.view(np.float32) / float(1 << out_shift(g))
        d = np.random.random(v.size) - np.random.random(v.size)    # TPDF: unbiased rounding around 127.5
        c.sendall(np.clip(np.rint(v + 127.5 + d), 0, 255).astype(np.uint8).tobytes())

    sinks.add(sink)
    try:
        while True:
            cmd, p = struct.unpack(">BI", recv_exact(c, 5))
            print(f"[rtl] cmd {cmd} {p}", flush=True)
            if cmd == 1: dev.set(freq=p)
            elif cmd == 5: dev.set(ppm=struct.unpack(">i", struct.pack(">I", p))[0])
            elif cmd == 3 and p == 0: dev.set(gain=60)
            elif cmd == 2: dev.set(rate=p)
            elif cmd == 4: dev.set(gain=max(0, min(82, round(p / 496 * 82))))
            elif cmd == 0x0d: dev.set(gain=max(0, min(82, round(min(p, 28) / 28 * 82))))
    except (OSError, struct.error):
        pass
    sinks.remove(sink)
    print("[rtl] client gone", addr, getattr(dev, "stats", {}), flush=True)


def listen(spec, handler, dev, sinks, info):
    host, port = spec.rsplit(":", 1)
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, int(port))); srv.listen(4)
    def loop():
        while True:
            c, addr = srv.accept(); c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=handler, args=(c, addr, dev, sinks, info), daemon=True).start()
    threading.Thread(target=loop, daemon=True).start()


def main():
    ap = argparse.ArgumentParser(description="ESP32-S3 esp-sdr IQ bridge (SpyServer + rtl_tcp)")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--spyserver", default="0.0.0.0:5555", help="host:port, empty to disable")
    ap.add_argument("--rtltcp", default="0.0.0.0:1234", help="host:port, empty to disable")
    ap.add_argument("--ppm", type=float, default=0.0)
    ap.add_argument("--freq", type=float, default=2450e6); ap.add_argument("--gain", type=int, default=60)
    ap.add_argument("--mode", type=int, default=2); ap.add_argument("--fake", action="store_true")
    a = ap.parse_args()
    global MODE; MODE = a.mode
    sinks = Sinks(); info = {}
    dev = Fake(sinks) if a.fake else Device(a.port, sinks, a.ppm)
    dev.set(freq=a.freq, gain=a.gain, rate=250000)
    if a.spyserver: listen(a.spyserver, spyserver_client, dev, sinks, info)
    if a.rtltcp: listen(a.rtltcp, rtltcp_client, dev, sinks, info)
    print(f"esp_bridge: {'fake tone' if a.fake else a.port}; spyserver {a.spyserver or 'off'}, rtl_tcp {a.rtltcp or 'off'}; "
          f"rates {sorted(RATES)}", flush=True)
    while True:
        time.sleep(60)
        print("[stat]", getattr(dev, "stats", {}), flush=True)


if __name__ == "__main__":
    main()