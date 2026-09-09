`timescale 1ns/1ps
// DSD chain S32 (DC gain ~2^31, FS-to-FS) -> +6 dB -> S24 by round-half-up
// with saturation (never wrap).
// The +6 dB (x2) matches commercial-DAC practice (ESS/AKM DSD-gain bits):
// SACD content peaks at 50% modulation (0.5 FS), so unity playback sounds
// 6 dB quieter than PCM. x2 puts SACD-0dB peaks exactly on the S24 rail
// (1 LSB saturation at the very peak); hotter-than-spec signals saturate
// instead of wrapping. The add is 34-bit: a narrower op would wrap +FS
// to negative. Shared by top.v and the DSD bench.
module dsd_round_s32_s24 (
    input  wire signed [31:0] in_s32,
    output wire signed [23:0] out_s24
);
    // x2 first (34-bit, cannot overflow), round at the S24 boundary, then
    // compare in the S24 scale. Comparing the unshifted Q31 value would
    // clamp normal music levels to a rail.
    wire signed [33:0] doubled =
        $signed({in_s32[31], in_s32, 1'b0});
    wire signed [33:0] rounded_s24 =
        (doubled + 34'sd128) >>> 8;
    assign out_s24 = (rounded_s24 > 34'sd8388607)  ? 24'h7FFFFF :
                     (rounded_s24 < -34'sd8388608) ? 24'h800000 :
                                                      rounded_s24[23:0];
endmodule
