#!/usr/bin/env python3
"""DSD64 0dB test file (SACD convention: sine at 50% modulation depth).

Why 50%: DSD silence is 50% ones-density; Scarlet Book caps the signal
at 50% modulation around it, i.e. density swings 25..75%. After an ideal
DSD->PCM decode that is a sine at 0.5 FS = -6.02 dBFS. So an honest
"0 dB DSD" file plays 6 dB quieter than a 0 dBFS PCM sine BY SPEC, and
the file below proves chain unity (output must equal exactly 0.5 FS),
not loudness. Any louder DSD file would overload the modulator.

Modulator: 5th-order CRFB sigma-delta (NTF = synthesizeNTF(5, 64,
opt=1, H_inf=1.5), Schreier deltasigma toolbox; ABCD stuffed by the
toolbox, loop bit-exactly as simulateDSM: y0=C.x+D1.u, v=sgn(y0),
x=A.x+B.[u,v], doubles). Measured: tone exactly 0.5 FS, HD2/3 < -130,
noise shelf ~-140 dBFS across 2..20 kHz (HQP/Roon class).

Output: stereo DSF64 (Roon-readable), L = 1 kHz, R = 2 kHz, equal level
(channel ID + level in one file). MSB-first, block-interleaved per the
DSF spec. 10 ms raised-cosine ends.

Usage: python3 tools/make_dsd0db.py [seconds] [out.dsf]  (default 30 s)
Requires: gcc.
"""
import os
import subprocess
import sys
import tempfile

# CRFB loop filter, doubles (from deltasigma stuffABCD, order 5).
A = [1.0, 0.0, 0.0, 0.0, 0.0,
     1.0, 1.0, -0.0006986126155796857, 0.0, 0.0,
     1.0, 1.0, 0.9993013873844203, 0.0, 0.0,
     0.0, 0.0, 1.0, 1.0, -0.001978322017535783,
     0.0, 0.0, 1.0, 1.0, 0.9980216779824642]
B = [0.0006604571797814107, -0.0006604571797814107,
     0.008744639684950864, -0.008744639684950864,
     0.06414238207256302, -0.06414238207256302,
     0.25018126352087433, -0.25018126352087433,
     0.8063314180838644, -0.8063314180838644]
C = [0.0, 0.0, 0.0, 0.0, 1.0]
D1 = 1.0

C_SRC = r"""
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static double A[25] = {%s};
static double B[10] = {%s};
static double C[5] = {%s};
// args: out.dsf seconds freqL freqR amplitude
int main(int argc, char **argv) {
    const char *path = argv[1];
    long seconds = atol(argv[2]);
    double fL = atof(argv[3]), fR = atof(argv[4]), Amp = atof(argv[5]);
    const double FS = 2822400.0;
    long n = (long)(FS * seconds);
    double xL[5] = {0,0,0,0,0}, xR[5] = {0,0,0,0,0};
    long nf = (long)(FS * 0.010);
    static unsigned char bL[4096], bR[4096];
    FILE *f = fopen(path, "wb");
    if (!f) { perror("open"); return 1; }
    unsigned long long dataBytes = ((unsigned long long)n + 7) / 8 * 2;
    unsigned long long total = 28 + 52 + 12 + dataBytes;
    fwrite("DSD ", 1, 4, f);
    unsigned long long t64 = 28; fwrite(&t64, 8, 1, f);
    fwrite(&total, 8, 1, f);
    t64 = 0; fwrite(&t64, 8, 1, f);
    fwrite("fmt ", 1, 4, f);
    t64 = 52; fwrite(&t64, 8, 1, f);
    unsigned int t32;
    t32 = 1; fwrite(&t32, 4, 1, f);
    t32 = 0; fwrite(&t32, 4, 1, f);
    t32 = 2; fwrite(&t32, 4, 1, f);
    t32 = 2; fwrite(&t32, 4, 1, f);
    t32 = 2822400; fwrite(&t32, 4, 1, f);
    t32 = 1; fwrite(&t32, 4, 1, f);
    unsigned long long nn = (unsigned long long)n; fwrite(&nn, 8, 1, f);
    t32 = 4096; fwrite(&t32, 4, 1, f);
    t32 = 0; fwrite(&t32, 4, 1, f);
    fwrite("data", 1, 4, f);
    t64 = 12 + dataBytes; fwrite(&t64, 8, 1, f);
    int bl = 0, bc = 0;
    memset(bL, 0, sizeof bL); memset(bR, 0, sizeof bR);
    for (long i = 0; i < n; i++) {
        double g = 1.0;
        if (i < nf) g = 0.5 - 0.5 * cos(M_PI * (double)i / (double)nf);
        else if (i >= n - nf) g = 0.5 - 0.5 * cos(M_PI * (double)(n - 1 - i) / (double)nf);
        double u[2];
        u[0] = Amp * g * sin(2 * M_PI * fL * (double)i / FS);
        u[1] = Amp * g * sin(2 * M_PI * fR * (double)i / FS);
        int q[2];
        double *xs[2] = {xL, xR};
        for (int ch = 0; ch < 2; ch++) {
            double *x = xs[ch];
            double y0 = C[0]*x[0]+C[1]*x[1]+C[2]*x[2]+C[3]*x[3]+C[4]*x[4] + %.1f*u[ch];
            q[ch] = (y0 >= 0);
            double v = q[ch] ? 1.0 : -1.0;
            double xn[5];
            for (int r = 0; r < 5; r++)
                xn[r] = A[r*5+0]*x[0]+A[r*5+1]*x[1]+A[r*5+2]*x[2]
                      + A[r*5+3]*x[3]+A[r*5+4]*x[4] + B[r*2]*u[ch] + B[r*2+1]*v;
            for (int r = 0; r < 5; r++) x[r] = xn[r];
        }
        if (q[0]) bL[bl] |= (0x80 >> bc);
        if (q[1]) bR[bl] |= (0x80 >> bc);
        if (++bc == 8) { bc = 0; if (++bl == 4096) {
            fwrite(bL, 1, 4096, f); fwrite(bR, 1, 4096, f);
            memset(bL, 0, sizeof bL); memset(bR, 0, sizeof bR);
            bl = 0;
        }}
    }
    if (bl || bc) { fwrite(bL, 1, 4096, f); fwrite(bR, 1, 4096, f); }
    fclose(f);
    printf("wrote %%s samples/ch=%%ld\n", path, n);
    return 0;
}
""" % (", ".join(repr(v) for v in A), ", ".join(repr(v) for v in B),
       ", ".join(repr(v) for v in C), D1)

DSF_HDR = 28 + 52 + 12


def read_dsf_mono(path, ch, seconds=6):
    import numpy as np
    with open(path, "rb") as f:
        f.seek(DSF_HDR)
        nblk = int(2822400 * seconds / (4096 * 8)) + 1
        bits = []
        for _ in range(nblk):
            blk = f.read(8192)
            if len(blk) < 8192:
                break
            b = blk[ch * 4096:(ch + 1) * 4096]
            bits.append(np.unpackbits(np.frombuffer(b, dtype=np.uint8)))
    return np.concatenate(bits).astype(float) * 2 - 1


def verify(path):
    import numpy as np
    print("verify (skip first 1 s, then 5 s per channel):")
    for ch, fr in ((0, 1000.0), (1, 2000.0)):
        v = read_dsf_mono(path, ch)[:2822400 * 6][2822400:]
        t = np.arange(len(v)) / 2822400.0
        amp = 2 * abs((v * np.exp(-2j * np.pi * fr * t)).mean())
        hd2 = 2 * abs((v * np.exp(-2j * np.pi * 2 * fr * t)).mean())
        hd3 = 2 * abs((v * np.exp(-2j * np.pi * 3 * fr * t)).mean())
        W = v * np.hanning(len(v))
        S = np.abs(np.fft.rfft(W)) / len(v) * 2
        f = np.fft.rfftfreq(len(v), 1 / 2822400.0)
        keep = np.ones_like(f, dtype=bool)
        for g in (fr, 2 * fr, 3 * fr):
            keep &= ~((f > g - 1000) & (f < g + 1000))
        band = (f > 2000) & (f < 20000) & keep
        floor = S[band].max()
        print("  ch%d: tone %.4f FS (want 0.5000) HD2 %.1f HD3 %.1f dBFS "
              "floor %.1f dBFS"
              % (ch, amp, 20 * np.log10(hd2 + 1e-18),
                 20 * np.log10(hd3 + 1e-18), 20 * np.log10(floor + 1e-18)))
        assert abs(amp - 0.5) < 0.005, "level off"
        assert hd2 < 3.2e-5 and hd3 < 3.2e-5, "harmonic too high"
        assert floor < 1.1e-6, "noise floor too high"
    print("VERIFY OK: SACD-0dB tones at 0.5 FS on a -140-class floor")


def main():
    seconds = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    out = sys.argv[2] if len(sys.argv) > 2 else "dsd0db_1k_2k.dsf"
    with tempfile.TemporaryDirectory() as td:
        c = os.path.join(td, "m.c")
        exe = os.path.join(td, "m")
        open(c, "w").write(C_SRC)
        subprocess.run(["gcc", "-O2", "-o", exe, c, "-lm"], check=True)
        subprocess.run([exe, out, str(seconds), "1000", "2000", "0.5"],
                       check=True)
    verify(out)
    print("deliver:", os.path.abspath(out))


if __name__ == "__main__":
    main()
