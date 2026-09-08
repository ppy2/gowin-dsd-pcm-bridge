`timescale 1ns/1ps
// Stereo 4th-order Butterworth low-pass, fc = 8 kHz @ fs = 48 kHz.
// Two biquad sections per channel (Q = 0.5412 / 1.3066).
// Coefficients: normalized RBJ cookbook, Q2.30. See tools/gen_lpf_coeffs.py.
// Latency: 2 mclk after in_valid (one per section).
module lpf_8k_4th (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    output wire        out_valid,
    output wire signed [23:0] out_l,
    output wire signed [23:0] out_r
);
    wire s1_valid_l;
    wire s1_valid_r;
    wire signed [23:0] s1_l;
    wire signed [23:0] s1_r;
    wire s2_valid_l;
    wire s2_valid_r;

    // Section 1: Q = 0.541196 (Butterworth pole pair 1)
    biquad_df1 #(
        .B0(32'h00E1450A), .B1(32'h01C28A14), .B2(32'h00E1450A),
        .A1(32'h9C38744F), .A2(32'h274C9FD8)
    ) bq1_l (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_data(in_l),
        .out_valid(s1_valid_l), .out_data(s1_l)
    );
    biquad_df1 #(
        .B0(32'h00E1450A), .B1(32'h01C28A14), .B2(32'h00E1450A),
        .A1(32'h9C38744F), .A2(32'h274C9FD8)
    ) bq1_r (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_data(in_r),
        .out_valid(s1_valid_r), .out_data(s1_r)
    );

    // Section 2: Q = 1.306563 (Butterworth pole pair 2)
    biquad_df1 #(
        .B0(32'h00FDFAE1), .B1(32'h01FBF5C1), .B2(32'h00FDFAE1),
        .A1(32'h8F80F701), .A2(32'h3476F481)
    ) bq2_l (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s1_valid_l), .in_data(s1_l),
        .out_valid(s2_valid_l), .out_data(out_l)
    );
    biquad_df1 #(
        .B0(32'h00FDFAE1), .B1(32'h01FBF5C1), .B2(32'h00FDFAE1),
        .A1(32'h8F80F701), .A2(32'h3476F481)
    ) bq2_r (
        .clk(clk), .rst_n(rst_n),
        .in_valid(s1_valid_r), .in_data(s1_r),
        .out_valid(s2_valid_r), .out_data(out_r)
    );

    // L/R latencies are identical; expose one valid.
    assign out_valid = s2_valid_l;
endmodule
