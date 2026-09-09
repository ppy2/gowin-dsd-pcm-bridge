`timescale 1ns/1ps
// Stereo S24 -> S16 with one-LSB TPDF dither and deterministic headroom.
//
// Correct order is FILTER (24b) -> DITHER -> QUANTIZE (16b). Dither must be
// the LAST operation before quantization: filtering after dither would
// colour the white dither (destroying its decorrelation), force a second
// quantization after the filter, and truncate the biquad feedback states
// (y1/y2) to 16b -> higher noise, limit cycles, worse stopband on the
// Q=1.306 stage. The sequential filter already costs 1 multiplier;
// narrowing it to 16b would save ~100 FFs (<0.5% of a 25K) and lose ~48 dB
// of internal dynamic range. So the filter is untouched; this block sits
// AFTER it.
//
// Headroom (proven in rebuild_i2s/stage2_16bit_dither/quantize16_tpdf.v):
//   safe = x - (x >>> 13)          // x * 8191/8192 = -0.00106 dB
//   q = (safe + tpdf + 128) >>> 8  // tpdf = r1+r2-255 in [-255,+255]
// Legal S24 endpoints then land at -32765..+32765: TPDF can never push a
// legal sample into the S16 rails, so there is NOTHING to saturate at
// 0 dBFS. No clamp exists by design (a clamp firing would itself be a
// nonlinear fault); the filter already saturates its 24b states, so the
// input here is always legal.
//
// TPDF: difference/sum of two independent 16-bit maximal-length LFSRs per
// channel (distinct polynomials AND distinct L/R seeds, as in the proven
// mono module). Stepped 16 states per stereo pair: consecutive output
// bytes share no register bits (a 1-step advance overlaps 7/8 bits and
// measured lag-1 autocorrelation 0.25 — lowpassed, not white). 16-step
// keeps full 65535-cycles (gcd(16, 65535) == 1), so marginals stay exactly
// triangular; the joint (la, lb) orbit is 65535 pairs (~0.37 s at the
// output grid — inaudible at +-1 LSB16, and identical to the 1-step
// design, which locksteps the same way). Zero mean, white,
// channel-independent (proven by tb_dither whiteness asserts + dump).
module dither_24_16 (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              in_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    output reg               out_valid,
    output reg  signed [15:0] out_l,
    output reg  signed [15:0] out_r
);
    reg [15:0] la, lb; // left RPDF pair
    reg [15:0] ra, rb; // right RPDF pair (different seeds)

    // 16-step advance (combinational unroll): consecutive bytes share no
    // bits. Taps: la/ra = x^16+x^14+x^13+x^11+1, lb/rb = x^16+x^15+x^13+x^4+1.
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

    // Unsigned 0..255 RPDF bytes, summed to 0..510, centred to +-255.
    wire signed [9:0] tpdf_l =
        $signed({2'b00, la[7:0]}) + $signed({2'b00, lb[7:0]}) - 10'sd255;
    wire signed [9:0] tpdf_r =
        $signed({2'b00, ra[7:0]}) + $signed({2'b00, rb[7:0]}) - 10'sd255;

    wire signed [23:0] safe_l = in_l - (in_l >>> 13);
    wire signed [23:0] safe_r = in_r - (in_r >>> 13);

    // Fits S24 by construction (see header): no overflow, no clamp.
    wire signed [23:0] total_l = safe_l + {{14{tpdf_l[9]}}, tpdf_l[9:0]} + 24'sd128;
    wire signed [23:0] total_r = safe_r + {{14{tpdf_r[9]}}, tpdf_r[9:0]} + 24'sd128;

    wire signed [15:0] q_l = total_l >>> 8;
    wire signed [15:0] q_r = total_r >>> 8;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            la <= 16'h1ace; lb <= 16'hb357; // as proven (left)
            ra <= 16'h5d19; rb <= 16'h54a4; // right: rb phase picked for
            // minimum ra x rb cross-correlation over lags +-8 (worst
            // 0.0040, at the measurement floor; m-sequence pairs have
            // ~0.008 cross peaks elsewhere — 0x7643 sat on one at lag-1)
            out_valid <= 1'b0;
            out_l <= 16'sd0; out_r <= 16'sd0;
        end else begin
            out_valid <= 1'b0;
            if (in_valid) begin
                out_l <= q_l;
                out_r <= q_r;
                out_valid <= 1'b1;
                la <= adv_la(la);
                lb <= adv_lb(lb);
                ra <= adv_la(ra);
                rb <= adv_lb(rb);
            end
        end
    end
endmodule
