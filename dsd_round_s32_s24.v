`timescale 1ns/1ps
// DSD chain S32 (DC gain ~2^31, FS-to-FS) -> S24 by round-half-up with
// saturation (never wrap).
// The add is 33-bit: a plain 32-bit +128 would wrap +FS to negative.
// Saturation IS needed (an earlier revision claimed otherwise — wrong):
// the stage-2 Kaiser DC sums to 2^31-8, so a full-scale DSD block gives
// S32 = +FS-8, and +128 rounds that to 2^23, one past the S24 rail.
// Clamping engages exactly at the rails, like the interpolator
// overshoot path. Dropped residue otherwise (<= 0.5 LSB24 = 0.002 LSB16)
// sits far below the TPDF floor. Shared by top.v and the DSD bench.
module dsd_round_s32_s24 (
    input  wire signed [31:0] in_s32,
    output wire signed [23:0] out_s24
);
    wire signed [32:0] tmp = $signed({in_s32[31], in_s32}) + 33'sd128;
    assign out_s24 = (tmp > 33'sd8388607)  ? 24'h7FFFFF :
                     (tmp < -33'sd8388608) ? 24'h800000 :
                                              tmp[23:0];
endmodule
