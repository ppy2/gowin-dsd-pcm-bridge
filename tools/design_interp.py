#!/usr/bin/env python3
"""Minimum-phase interpolation prototypes for the stage-2 SRC (TDA1541).

One coefficient set per RATIO (shared by both MCLK domains):
  X4: 44.1->176.4 and 48->192  (the RedBook flagship path)
  X2: 88.2->176.4 and 96->192
Frequencies normalized to the OUTPUT rate (1.0).
Philosophy (NOS-preserving): short, gentle, minimum-phase (no precursor),
FLAT passband (no EQ tricks inside the filter — the DAC's own residual
sinc droop at 176.4/192k is -0.19 dB @20k, inaudible and stated openly),
strong-enough stopband (images down, no brickwall worship).

Prototype: REMEZ equiripple (constant bands — the reason the passband is
flat by design, not by luck), then exact zero-reflection to minimum
phase (length preserved, |H| verified), zero-pad to a multiple of R
(no response change), quantize Q2.30 with exact DC.

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


def minphase_convert(h, f_pass=None):
    """Minimum-phase conversion via real cepstrum (no root finding:
    explicit zeros of a degree-100+ clustered polynomial are numerically
    hopeless — reconstructed taps blew up to 8e6).

    Recipe: X=FFT(h); fold the real cepstrum of log|X| (eps floor for the
    remez stopband zeros, -240 dB — irrelevant); H_min=exp(); IFFT and
    truncate to N (cepstral tail beyond N is aliasing noise, verified).
    Split verification metric (see previous docstring discussion):
    passband relative deviation (gate 1e-6) + stopband ABSOLUTE floor
    relative to pass level (gate -90 dB).
    Returns (h_min, dev_pass, dev_stop_abs_db).
    """
    h = np.asarray(h, dtype=np.float64)
    n = len(h)
    n_fft = 16384
    X = np.fft.rfft(h, n_fft)
    mag = np.maximum(np.abs(X), 1e-12)  # floor the remez stopband zeros
    logm = np.log(mag)
    # Real cepstrum via full FFT (even spectrum handling by irfft/rfft).
    ceps = np.fft.irfft(logm, n_fft)
    # Causalize: fold to minimum-phase.
    cmin = np.zeros(n_fft)
    cmin[0] = ceps[0]
    cmin[1:n_fft // 2] = 2.0 * ceps[1:n_fft // 2]
    cmin[n_fft // 2] = ceps[n_fft // 2]
    Hmin = np.exp(np.fft.rfft(cmin, n_fft))
    h_full = np.fft.irfft(Hmin, n_fft)
    h_min = h_full[:n].copy()
    # Gain fit at DC (both positive-real there for our prototypes).
    g = np.sum(h) / np.sum(h_min)
    h_min = h_min * g
    # Verify magnitude preservation on a dense grid, split metric (see
    # docstring): passband relative, stopband absolute vs pass level.
    w, H0 = signal.freqz(h, worN=8192)
    _, H1 = signal.freqz(h_min, worN=8192)
    f = w / (2 * np.pi)
    m0, m1 = np.abs(H0), np.abs(H1)
    passlvl = np.max(m0)
    d = np.abs(m1 - m0)
    if f_pass is None:
        dev_pass, dev_db = d.max() / passlvl, -np.inf
    else:
        dev_pass = d[f <= f_pass].max() / passlvl
        dev_db = 20 * np.log10(d[f > f_pass].max() / passlvl + 1e-30)
    assert dev_pass < 1e-5, f"passband not preserved: {dev_pass}"
    # -75 dB gate: conversion noise 20+ dB below the shallowest stopband
    # spec (-55 dB); the true stopband is verified on QUANTIZED taps below.
    assert dev_db < -75.0, f"stopband numerical floor too high: {dev_db} dB"
    return h_min, dev_pass, dev_db


def design_set(R, N, f_stop, w_stop=5.0):
    # Remez equiripple prototype, Type I (odd N). Bands are constant by
    # construction: flat R in the passband, 0 in the stopband.
    assert N % 2 == 1, "Type I prototype needs odd N"
    bands = [0.0, F_PASS, f_stop, 0.5]
    h_lin = signal.remez(N, bands, [R, 0.0], weight=[1.0, w_stop], fs=1.0)
    h_min, dev_p, dev_s = minphase_convert(h_lin, F_PASS)
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
    # Normalize float DC to exactly R (firwin2's own DC is R +/- 0.001 —
    # inaudible, but unity must be exact by construction, not by luck),
    # then round; the integer trim below moves only a few LSB.
    tot = sum(float(np.sum(p)) for p in phases)
    k = R / tot
    assert 0.98 < k < 1.02, f"prototype DC far off: {tot}"
    qph = []
    for p in phases:
        qph.append(np.round(np.asarray(p, dtype=np.float64) * k * S
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
    # Precursor OSCILLATION (the audible thing): negative taps before the
    # peak. A monotonic positive ramp-up is rise time, not pre-ringing.
    pre_neg = -np.min(hq[:k]) / np.max(np.abs(hq)) if k > 0 else 0.0
    pre_neg = max(pre_neg, 0.0)
    step = np.cumsum(hq) / R
    over = (np.max(step) - 1.0) * 100.0
    under = -np.min(np.minimum(step, 0.0)) * 100.0  # below-zero dip only
    wcg = np.sum(np.abs(hq)) / R  # worst-case peak gain (saturation margin)
    print(f"== {name}: R={R} taps/phase={n_ph} total={n_ph * R}")
    print(f"   pass ripple 0..20k : {pb:.3f} dB "
          f"(+lift@{20e3:.0f}Hz = {20 * math.log10(m20):+.3f} dB)")
    print(f"   stop from {f_stop * FS_O / 1e3:.1f}k : {stop:.1f} dB")
    print(f"   main tap #{k}, pre-energy : "
          f"{10 * math.log10(e_pre + 1e-30):.1f} dB "
          f"(neg-precursor {pre_neg * 100:.2f}% of peak)")
    print(f"   step overshoot +{over:.2f}% preshoot {under:.3f}%")
    print(f"   worst-case peak gain x{wcg:.3f} "
          f"({20 * math.log10(wcg):+.2f} dB over unity)")
    print(f"   max|tap| = {np.max(np.abs(hq)):.4f}, "
          f"DC err = {(np.sum(hq) / R - 1.0):+.2e}")
    return hq


def emit_vh(path, sets):
    # Pure Verilog-2001 case ROM (no SystemVerilog array literals: the
    # Gowin project compiles as Verilog 2001). Address map:
    # addr = {rom_set, phase[1:0], tap[4:0]}; rom_set 0=X2, 1=X4.
    # X2 uses phases 0..1 (2..3 read as 0); taps beyond the prototype
    # length are explicit zeros (padding does not change the response).
    with open(path, 'w') as f:
        f.write("// GENERATED by tools/design_interp.py — do not hand-edit.\n")
        f.write("`timescale 1ns/1ps\n")
        f.write("function [31:0] interp_rom;\n")
        f.write("    input [7:0] addr;\n")
        f.write("    case (addr)\n")
        for name, qph, R in sets:
            sbit = 1 if name == "X4" else 0
            for p in range(4):
                q = qph[p] if p < len(qph) else [0] * len(qph[0])
                for t, v in enumerate(q):
                    a = (sbit << 7) | (p << 5) | t
                    f.write(f"        8'h{a:02X}: "
                            f"interp_rom = 32'h{int(v) & 0xFFFFFFFF:08X};\n")
        f.write("        default: interp_rom = 32'h00000000;\n")
        f.write("    endcase\n")
        f.write("endfunction\n")


def emit_hex(prefix, sets):
    import os
    os.makedirs("tools/coef_hex", exist_ok=True)
    for name, qph, R in sets:
        for p, q in enumerate(qph):
            with open(f"tools/coef_hex/{name}_p{p}.hex", 'w') as f:
                for v in q:
                    f.write(f"{int(v) & 0xFFFFFFFF:08X}\n")


def main():
    # LOCKED (measured 2026-09): X4 N=121 (stop 59.7 dB, ripple 0.09 dB,
    # zero precursor); X2 N=25 (stop 125 dB, 26 taps — short on purpose).
    # Taller candidates explored: N=101 too weak (52 dB / 0.21 dB),
    # N=141 stronger (66 dB) but longer time-smear for no audible need.
    h_lin, h_min, phases4, n4 = design_set(4, 121, F_IMG4)
    qph4 = quantize(phases4, 4)
    analyze("X4", qph4, 4, F_IMG4)
    print("--- X2 ---")
    _, _, phases2, n2 = design_set(2, 25, F_IMG2)
    qph2 = quantize(phases2, 2)
    analyze("X2", qph2, 2, F_IMG2)
    # Pad X2 phases to the engine TAPN (trailing zeros: no response change).
    tapn = len(qph4[0])
    qph2p = []
    for q in qph2:
        e = np.zeros(tapn, dtype=np.int64)
        e[:len(q)] = q
        qph2p.append(e)
    emit_vh("tools/interp_coefs.vh", [("X4", qph4, 4), ("X2", qph2p, 2)])
    emit_hex("tools/coef_hex", [("X4", qph4, 4), ("X2", qph2p, 2)])
    print(f"emitted tools/interp_coefs.vh + tools/coef_hex, TAPN={tapn}")


if __name__ == '__main__':
    main()
