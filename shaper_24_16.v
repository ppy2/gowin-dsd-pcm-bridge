`timescale 1ns/1ps
// Stereo S24 -> S16 with 2nd-order FIR error-feedback noise shaping +
// dithered quantizer (drop-in alternative to dither_24_16).
//
// NTF = (1 - z^-1)^2 (FIR: no poles, unconditionally stable — the worst
// overload can do is a transient saturation, never a sustained limit
// cycle).
// DITH_ATTN: quantizer dither dose. 0 = full TPDF +-1 LSB (legacy, kept
// for methodology A/B); 1 = half TPDF +-0.5 LSB (ship candidate).
// Rationale (measured, LPF-calibrated 0..20k @384k — the earlier +7.4 dB
// "worse than TPDF" number was FFT-sidelobe leakage of the HF mountain
// through the brickwall-FFT meter, NOT in-band noise; white-noise cal
// of the LPF meter reads exactly 20/192):
//   full TPDF: -1.8 dB vs flat TPDF (dither passes unshaped — the classic
//     EF-dither floor; Wannamaker) | THD -128 | clean tones
//   half TPDF: -7.8 dB vs flat TPDF | THD -131 | idle/low-level spectra
//     tone-free (-90/-60 dBFS probes) | HF 1.6x TPDF (0.8 vs 0.5 LSB rms,
//     negligible for the TDA I/V)
// Discarded: quarter TPDF (-13.8 dB but thinner linearization — ears must
// prove half first), NTF-shaped dither (limit-cycle tones, pk/med 16),
// 3rd-order EF (same dither floor, 2.8x HF — order buys nothing once the
// flat dither dominates).
// Loop (per channel, all integer, exact):
//   safe = x - (x >>> 13)            // same -0.00106 dB headroom as TPDF
//   u    = safe + (e1 <<< 1) - e2    // e1/e2 = past quantizer errors
//   uq   = u + tpdf_att + 128        // tpdf_att = full/half triangle from
//                                    //   the proven LFSRs/seeds
//   y    = sat16(uq >>> 8)           // SATURATE, never wrap
//   e    = sat12(uq - (y <<< 8))     // normally +-128 (rounding remainder)
// dither_24_16 has NO clamp by design (headroom makes it unnecessary);
// HERE the clamp is mandatory: feedback + hot transients can exceed the
// rails, and a wrap would be a full-scale harmonic comb. Saturation is
// a benign clip on illegal overshoots only (TB counts fires).
// Error width: arithmetic needs 28 bits; e is saturated to +-2047
// (fires only on quantizer overload; TB asserts never on music-like
// signals and boundedness always).
// DC note: mean out = in/256 + ~1 LSB (round-half-up +0.5 plus the EF
// error mean +0.5 — same sub-LSB class as the proven TPDF's +0.5 bias,
// inaudible, DAC offsets dominate; TB models it explicitly).
module shaper_24_16 #(
    parameter DITH_ATTN = 0   // 0 = full TPDF, 1 = half TPDF (ship)
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              in_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    output reg               out_valid,
    output reg  signed [15:0] out_l,
    output reg  signed [15:0] out_r
);
    // Proven 16-step LFSR advance (same polynomials/seeds as dither_24_16:
    // triangular marginals, white, L/R independent).
    function [15:0] adv_la;
        input [15:0] s;
        integer j;
        begin
            adv_la = s;
            for (j = 0; j < 16; j = j + 1)
                adv_la = {adv_la[14:0],
                    adv_la[15] ^ adv_la[13] ^ adv_la[12] ^ adv_la[10]};
        end
    endfunction
    function [15:0] adv_lb;
        input [15:0] s;
        integer j;
        begin
            adv_lb = s;
            for (j = 0; j < 16; j = j + 1)
                adv_lb = {adv_lb[14:0],
                    adv_lb[15] ^ adv_lb[14] ^ adv_lb[12] ^ adv_lb[3]};
        end
    endfunction

    reg [15:0] la, lb, ra, rb;
    reg signed [11:0] e1l, e2l, e1r, e2r; // past errors, +-2047 saturated

    wire signed [9:0] tpdf_full_l =
        $signed({2'b00, la[7:0]}) + $signed({2'b00, lb[7:0]}) - 10'sd255;
    wire signed [9:0] tpdf_full_r =
        $signed({2'b00, ra[7:0]}) + $signed({2'b00, rb[7:0]}) - 10'sd255;
    // Half dose: exact arithmetic shift (symmetric, still triangular).
    wire signed [9:0] tpdf_l = (DITH_ATTN == 0) ? tpdf_full_l
                                               : (tpdf_full_l >>> 1);
    wire signed [9:0] tpdf_r = (DITH_ATTN == 0) ? tpdf_full_r
                                               : (tpdf_full_r >>> 1);

    wire signed [23:0] safe_l = in_l - (in_l >>> 13);
    wire signed [23:0] safe_r = in_r - (in_r >>> 13);

    // 28-bit loop arithmetic: safe (24) + feedback (2*e1-e2, |.|<=6141) +
    // dither (10) + round (8) — no overflow possible, all exact.
    wire signed [27:0] u_l =
        $signed({{4{safe_l[23]}}, safe_l}) + ($signed(e1l) <<< 1) - $signed(e2l);
    wire signed [27:0] u_r =
        $signed({{4{safe_r[23]}}, safe_r}) + ($signed(e1r) <<< 1) - $signed(e2r);
    wire signed [27:0] uq_l = u_l + $signed({{18{tpdf_l[9]}}, tpdf_l}) + 28'sd128;
    wire signed [27:0] uq_r = u_r + $signed({{18{tpdf_r[9]}}, tpdf_r}) + 28'sd128;

    wire signed [27:0] qraw_l = uq_l >>> 8;
    wire signed [27:0] qraw_r = uq_r >>> 8;
    wire signed [15:0] y_l = (qraw_l > 28'sd32767) ? 16'sd32767 :
                             (qraw_l < -28'sd32768) ? -16'sd32768 : qraw_l[15:0];
    wire signed [15:0] y_r = (qraw_r > 28'sd32767) ? 16'sd32767 :
                             (qraw_r < -28'sd32768) ? -16'sd32768 : qraw_r[15:0];

    wire signed [27:0] e_raw_l = uq_l - ($signed(y_l) <<< 8);
    wire signed [27:0] e_raw_r = uq_r - ($signed(y_r) <<< 8);
    wire signed [11:0] e_new_l = (e_raw_l > 28'sd2047) ? 12'sd2047 :
                                 (e_raw_l < -28'sd2048) ? -12'sd2048 : e_raw_l[11:0];
    wire signed [11:0] e_new_r = (e_raw_r > 28'sd2047) ? 12'sd2047 :
                                 (e_raw_r < -28'sd2048) ? -12'sd2048 : e_raw_r[11:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            la <= 16'h1ace; lb <= 16'hb357;
            ra <= 16'h5d19; rb <= 16'h54a4;
            e1l <= 12'sd0; e2l <= 12'sd0;
            e1r <= 12'sd0; e2r <= 12'sd0;
            out_valid <= 1'b0;
            out_l <= 16'sd0; out_r <= 16'sd0;
        end else begin
            out_valid <= 1'b0;
            if (in_valid) begin
                out_l <= y_l;
                out_r <= y_r;
                out_valid <= 1'b1;
                e2l <= e1l; e1l <= e_new_l;
                e2r <= e1r; e1r <= e_new_r;
                la <= adv_la(la);
                lb <= adv_lb(lb);
                ra <= adv_la(ra);
                rb <= adv_lb(rb);
            end
        end
    end
endmodule
