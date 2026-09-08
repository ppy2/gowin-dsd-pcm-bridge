#!/usr/bin/env python3
"""Butterworth LPF -> Q2.30 biquad params for biquad_df1 (RBJ cookbook)."""
import argparse, math, cmath

QS_4TH = (0.541196100146197, 1.306562964876377)  # 24 dB/oct Butterworth

def design(fc, fs):
    w0 = 2 * math.pi * fc / fs
    cosw, sinw = math.cos(w0), math.sin(w0)
    out = []
    for q in QS_4TH:
        alpha = sinw / (2 * q)
        b0 = (1 - cosw) / 2; b1 = 1 - cosw; b2 = b0
        a0 = 1 + alpha; a1 = -2 * cosw; a2 = 1 - alpha
        out.append((b0/a0, b1/a0, b2/a0, a1/a0, a2/a0))
    return out

def resp(sos, f, fs):
    z = cmath.exp(1j * 2 * math.pi * f / fs)
    h = 1 + 0j
    for b0, b1, b2, a1, a2 in sos:
        h *= (b0 + b1*z**-1 + b2*z**-2) / (1 + a1*z**-1 + a2*z**-2)
    return 20 * math.log10(abs(h))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fc', type=float, default=8000)
    ap.add_argument('--fs', type=float, default=48000)
    a = ap.parse_args()
    sos = design(a.fc, a.fs)
    S = 2 ** 30
    for i, (b0, b1, b2, a1, a2) in enumerate(sos, 1):
        ints = [int(round(v * S)) for v in (b0, b1, b2, a1, a2)]
        print(f'section {i}: ' + ' '.join(f'{v:+.9f}' for v in (b0, b1, b2, a1, a2)))
        print(f'  B0=32\'h{ints[0]&0xFFFFFFFF:08X} B1=32\'h{ints[1]&0xFFFFFFFF:08X} '
              f'B2=32\'h{ints[2]&0xFFFFFFFF:08X} A1=32\'h{ints[3]&0xFFFFFFFF:08X} '
              f'A2=32\'h{ints[4]&0xFFFFFFFF:08X}')
    for f in (1000, 4000, 8000, 12000, 16000, 20000):
        if f < a.fs / 2:
            print(f'f={f:6.0f} Hz mag={resp(sos, f, a.fs):+7.2f} dB')

if __name__ == '__main__':
    main()
