"""Minimal SpyServer client (SDR++-style) for bridge tests: per rate/format -> rate, tone, SNR, dBFS.
    python ss_client.py [host:port] [freq_hz] [gain] [seconds]
"""
import socket, struct, sys, time
import numpy as np
PROTO = (2 << 24) | (0 << 16) | 1700
hp = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:5555"
F = float(sys.argv[2]) if len(sys.argv) > 2 else 2450e6
G = int(sys.argv[3]) if len(sys.argv) > 3 else 60
T = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
import os
OFF = float(os.environ.get('OFF', 40e3)); DECS = [int(x) for x in os.environ.get('DECS', '').split(',') if x]
if os.environ.get('VSG'):
    v = socket.create_connection(('127.0.0.1', 5124), timeout=20); vf = v.makefile('rwb')
    for q in (':OUTPut:MODulation:STATe OFF', ':SOURce:POWer -50.00', ':SOURce:FREQuency %d' % int(F + OFF), ':OUTPut:STATe ON'): vf.write((q + '\n').encode()); vf.flush()
    print('VSG -50 dBm @ %.3f MHz' % ((F + OFF) / 1e6))
h, p = hp.rsplit(":", 1); c = socket.create_connection((h, int(p)), timeout=5)

def cmd(t, body): c.sendall(struct.pack("<II", t, len(body)) + body)
def setting(s, v): cmd(2, struct.pack("<II", s, int(v)))
def rx(n):
    b = b""
    while len(b) < n:
        d = c.recv(n - len(b))
        if not d: raise SystemExit("closed")
        b += d
    return b
def msg():
    pid, mt, st, seq, n = struct.unpack("<5I", rx(20)); return mt & 0xFFFF, mt >> 16, st, seq, rx(n)

cmd(0, struct.pack("<I", PROTO) + b"ss_client")
t, _, _, _, b = msg(); di = struct.unpack("<12I", b); print("devinfo", di)
t, _, _, _, b = msg(); print("sync", struct.unpack("<9I", b))
for dec in (DECS or range(di[10], di[4] + 1)):
    for fmt in (2, 1, 4):
        sr = di[2] / (1 << dec)
        setting(100, fmt); setting(102, dec); setting(101, F); setting(0, 1); setting(2, G); setting(1, 1)
        t0 = time.time(); xs = []; seqs = []; N = 0; settle = 2.5
        while time.time() - t0 < T + settle:
            mt, fl, st, seq, b = msg()
            if mt not in (100, 101, 103): continue
            g = 10 ** (fl / 20)
            if mt == 101: v = np.frombuffer(b, "<i2") / (32768 * g)
            elif mt == 100: v = (np.frombuffer(b, np.uint8) - 128.0) / (128 * g)
            else: v = np.frombuffer(b, "<f4") * g
            if time.time() - t0 > settle: xs.append(v[0::2] + 1j * v[1::2]); seqs.append(seq)
        setting(1, 0); time.sleep(0.3)
        c.settimeout(0.3)
        try:
            while True: c.recv(1 << 20)
        except OSError: pass
        c.settimeout(5)
        z = np.concatenate(xs); N = len(z); rate = N / T
        nf = 8192; k = N // nf; w = np.hanning(nf)
        P = np.mean([np.abs(np.fft.fftshift(np.fft.fft(z[i*nf:(i+1)*nf] * w))) ** 2 for i in range(k)], 0) / np.sum(w) ** 2
        fr = np.fft.fftshift(np.fft.fftfreq(nf, 1 / sr)); pk = np.argmax(P)
        ton = 10 * np.log10(P[pk-3:pk+4].sum()); floor = 10 * np.log10(np.median(P))
        print(f"dec {dec} sr {sr/1e3:6.1f}k fmt {fmt}: got {rate/1e3:6.1f} kS/s  tone {fr[pk]/1e3:+7.2f} kHz {ton:6.1f} dBFS  "
              f"floor {floor:6.1f} dBFS/bin  SNR {ton-floor:5.1f} dB  rms {10*np.log10(np.mean(np.abs(z)**2)):6.1f} dBFS", flush=True)
c.close()
if os.environ.get('VSG'):
    vf.write(b':OUTPut:STATe OFF\n'); vf.flush()
