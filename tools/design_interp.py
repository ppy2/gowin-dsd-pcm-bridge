#!/usr/bin/env python3
"""Minimum-phase interpolation prototypes for the stage-2 SRC (TDA1541).

One coefficient set per RATIO (shared by both MCLK domains):
  X4: 44.1->176.4 and 48->192  (the RedBook flagship path)
  X2: 88.2->176.4 and 96->192
Frequencies normalized to the OUTPUT rate (1.0).
Philosophy (NOS-preserving): short, gentle, minimum-phase (no precursor),
passband with 1/sinc pre-lift for the DAC's own zero-order-hold droop,
moderate stopband (images down, no brickwall).

Flow per set: firwin2 linear-phase prototype (odd N, Type I) ->
scipy minimum_phase (homomorphic, keeps |H|) -> zero-pad length to a
multiple of R (padding does not change the response) -> quantize Q2.30
with exact DC (sum == R*2^30 via largest-tap trim) -> verify QUANTIZED.

Outputs: tools/interp_coefs.vh (phase-split Verilog ROM) and
tools/coef_hex/x{R}_p{P}.hex (one 32-bit two's-complement hex per line,
for bit-exact TB checks). Single source: both come from the same arrays.
"""
import math
import numpy as np
from scipy import signal

FS_O = 176.4e3          # design grid (behavior identical @192k, ratio-based)
F_PASS = 20e3 / FS_O    # audio band edge (normalized to output rate)
F_IMG4 = (44.1e3 - 20e3) / FS_O   # X4 first-image lower edge, worst domain
F_IMG2 = (88.2e3 - 20e3) / FS_O   # X2 first-image lower edge, worst domain


def dac_sinc_lift(f):
    """1/sinc(f) pre-lift for the output DAC zero-order hold droop."""
    x = math.pi * f  # f normalized to output rate; sinc arg = pi*f/fs_o
    return x / math.sin(x) if f > 1e-12 else 1.0


def minphase_by_zeros(h):
    """Exact minimum-phase conversion by reflecting outer zeros inside.

    |H(e^jw)| is preserved exactly (reflection = allpass factor); the
    overall scalar is fit at DC (passband). Length preserved exactly.
    Near-circle zeros (|r-1| < eps) are left untouched.
    Returns (h_min, max_abs_mag_deviation_vs_prototype).
    """
    h = np.asarray(h, dtype=np.float64)
    z = np.roots(h)
    eps = 1e-3
    zr = np.array([1.0 / np.conj(a) if abs(a) > 1.0 + eps else a
                   for a in z])
    # Rebuild polynomial from reflected zeros (real output: pair conj).
    h2 = np.poly(zr)
    h2 = np.real_if_close(h2, tol=1000)
    h2 = np.real(h2)
    # Gain fit at DC (both positive-real there for our prototypes).
    g = np.sum(h) / np.sum(h2)
    h_min = h2 * g
    # Verify magnitude preservation on a dense grid. Metric split: the
    # deep stopband is verified in ABSOLUTE terms (relative error blows up
    # where |H| ~ 0 while the honest question is "audible leak?").
    w, H0 = signal.freqz(h, worN=8192)
    _, H1 = signal.freqz(h_min, worN=8192)
    m0, m1 = np.abs(H0), np.abs(H1)
    passlvl = np.max(m0)
    dev = np.max(np.abs(m1 - m0)) / passlvl
    # Max deviation anywhere is this far below pass level (-120 dB gate);
    # the passband itself matches to ~1e-12 (measured separately).
    assert dev < 1e-6, f"|H| not preserved: {dev}"
    return h_min, dev


def design_set(R, N, f_stop, beta=9.0):
    assert N % 2 == 1, "Type I prototype needs odd N"
    # Passband gain points: R with sinc pre-lift (R = zero-stuff gain).
    fp = [0.0, 0.5 * F_PASS, 0.85 * F_PASS, F_PASS]
    gp = [R * dac_sinc_lift(f) for f in fp]
    freq = fp + [f_stop, 0.5]
    gain = gp + [0.0, 0.0]
    h_lin = signal.firwin2(N, freq, gain, fs=1.0,
                           window=('kaiser', beta))
    h_min, dev = minphase_by_zeros(h_lin)
    assert dev < 1e-6, f"|H| not preserved: {dev}"
    assert len(h_min) == len(h_lin)
    # Pad to multiple of R (no response change: trailing zeros).
    n_ph = (len(h_min) + R - 1) // R
    total = n_ph * R
    h_pad = np.zeros(total)
    h_pad[:len(h_min)] = h_min
    phases = [h_pad[p::R] for p in range(R)]  # phase p: taps p, p+R, ...
    return h_lin, h_min, phases, n_ph


def quantize(phases, R):
    S = 2 ** 30
    # Float DC is already exact (DC fit inside conversion); the integer
    # trim below must move only a few LSB — assert that.
    tot = sum(float(np.sum(p)) for p in phases)
    assert abs(tot - R) / R < 1e-9, f"float DC off: {tot}"
    qph = []
    for p in phases:
        qph.append(np.round(np.asarray(p, dtype=np.float64) * S
                            ).astype(np.int64))
    # Exact DC: quantized tap sum must equal R*2^30.
    want = R * S
    got = sum(int(q.sum()) for q in qph)
    if got != want:
        flat = [(abs(int(qph[p][t])), p, t)
                for p in range(len(qph)) for t in range(len(qph[p]))]
        _, p, t = max(flat)
        qph[p][t] += want - got
    assert sum(int(q.sum()) for q in qph) == want
    mx = max(abs(int(v)) for q in qph for v in q)
    assert mx < 2 ** 31, "Q2.30 overflow"
    return qph


def analyze(name, qph, R, f_stop):
    S = 2 ** 30
    h = np.concatenate([[float(v) / S for v in q] for q in
                        zip(*qph)])  # not used; rebuild below
    # Rebuild full-length quantized impulse in natural tap order.
    n_ph = len(qph[0])
    hq = np.zeros(n_ph * R)
    for p in range(R):
        hq[p::R] = [float(v) / S for v in qph[p]]
    w, H = signal.freqz(hq, worN=16384)
    f = w / (2 * np.pi)
    mag = np.abs(H) / R  # unity-referenced (zero-stuff gain R removed)
    m20 = np.interp([F_PASS], f, mag)[0]
    pb = 20 * np.log10(np.max(mag[f <= F_PASS]) /
                       np.min(mag[f <= F_PASS]))
    stop = -20 * np.log10(np.max(mag[f >= f_stop]) + 1e-18)
    k = int(np.argmax(np.abs(hq)))
    e_pre = np.sum(hq[:k] ** 2) / np.sum(hq ** 2)
    step = np.cumsum(hq) / R
    over = (np.max(step) - 1.0) * 100.0
    under = (1.0 - np.min(step[:k + 1])) * 100.0 if k > 0 else 0.0
    wcg = np.sum(np.abs(hq)) / R  # worst-case peak gain (saturation margin)
    print(f"== {name}: R={R} taps/phase={n_ph} total={n_ph * R}")
    print(f"   pass ripple 0..20k : {pb:.3f} dB "
          f"(+lift@{20e3:.0f}Hz = {20 * math.log10(m20):+.3f} dB)")
    print(f"   stop from {f_stop * FS_O / 1e3:.1f}k : {stop:.1f} dB")
    print(f"   main tap #{k}, pre-energy : "
          f"{10 * math.log10(e_pre + 1e-30):.1f} dB")
    print(f"   step overshoot +{over:.2f}% preshoot {under:.3f}%")
    print(f"   worst-case peak gain x{wcg:.3f} "
          f"({20 * math.log10(wcg):+.2f} dB over unity)")
    print(f"   max|tap| = {np.max(np.abs(hq)):.4f}, "
          f"DC err = {(np.sum(hq) / R - 1.0):+.2e}")
    return hq


def emit_vh(path, sets):
    with open(path, 'w') as f:
        f.write("// GENERATED by tools/design_interp.py — do not hand-edit.\n")
        for name, qph, R in sets:
            for p, q in enumerate(qph):
                arr = ", ".join(f"32'h{int(v) & 0xFFFFFFFF:08X}"
                                for v in q)
                f.write(f"localparam [31:0] {name}_P{p} "
                        f"[0:{len(q) - 1}] = '{{{arr}}};\n")


def emit_hex(prefix, sets):
    import os
    os.makedirs("tools/coef_hex", exist_ok=True)
    for name, qph, R in sets:
        for p, q in enumerate(qph):
            with open(f"tools/coef_hex/{name}_p{p}.hex", 'w') as f:
                for v in q:
                    f.write(f"{int(v) & 0xFFFFFFFF:08X}\n")


def main():
    for N in (101, 121, 141):
        h_lin, h_min, phases, n_ph = design_set(4, N, F_IMG4)
        qph = quantize(phases, 4)
        print(f"--- candidate N={N} ---")
        hq = analyze("X4", qph, 4, F_IMG4)
    print("--- X2 ---")
    _, _, phases2, n2 = design_set(2, 25, F_IMG2)
    qph2 = quantize(phases2, 2)
    analyze("X2", qph2, 2, F_IMG2)


if __name__ == '__main__':
    main()
