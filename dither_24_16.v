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
// mono module). Stepped once per stereo pair. Combined period ~2^32
// samples; zero mean, white, channel-independent.
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
            ra <= 16'h5d19; rb <= 16'h7643; // as proven (right)
            out_valid <= 1'b0;
            out_l <= 16'sd0; out_r <= 16'sd0;
        end else begin
            out_valid <= 1'b0;
            if (in_valid) begin
                out_l <= q_l;
                out_r <= q_r;
                out_valid <= 1'b1;
                la <= {la[14:0], la[15] ^ la[13] ^ la[12] ^ la[10]};
                lb <= {lb[14:0], lb[15] ^ lb[14] ^ lb[12] ^ lb[3]};
                ra <= {ra[14:0], ra[15] ^ ra[13] ^ ra[12] ^ ra[10]};
                rb <= {rb[14:0], rb[15] ^ rb[14] ^ rb[12] ^ rb[3]};
            end
        end
    end
endmodule
