`timescale 1ns/1ps
// Biquad IIR section, Direct Form I.
// Data: 24-bit signed. Coefficients: Q2.30 signed (range -2..+1.999).
//   y[n] = b0*x[n]+b1*x[n-1]+b2*x[n-2]-a1*y[n-1]-a2*y[n-2]
// Plain signed '*' — the Gowin IDE infers DSP macros from it.
// 1 mclk latency: out_valid follows in_valid by one cycle.
module biquad_df1 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire signed [23:0] in_data,
    output reg         out_valid,
    output reg  signed [23:0] out_data
);
    parameter [31:0] B0 = 32'h08E36CD8;
    parameter [31:0] B1 = 32'h11C6D9B0;
    parameter [31:0] B2 = 32'h08E36CD8;
    parameter [31:0] A1 = 32'hDC724CA1;
    parameter [31:0] A2 = 32'h071B66BE;

    wire signed [31:0] b0 = B0;
    wire signed [31:0] b1 = B1;
    wire signed [31:0] b2 = B2;
    wire signed [31:0] a1 = A1;
    wire signed [31:0] a2 = A2;

    reg signed [23:0] x1;
    reg signed [23:0] x2;
    reg signed [23:0] y1;
    reg signed [23:0] y2;

    wire signed [23:0] x0 = in_data;
    wire signed [55:0] p0 = x0 * b0;
    wire signed [55:0] p1 = x1 * b1;
    wire signed [55:0] p2 = x2 * b2;
    wire signed [55:0] q1 = y1 * a1;
    wire signed [55:0] q2 = y2 * a2;
    wire signed [63:0] acc = p0 + p1 + p2 - q1 - q2;
    // round-half-up at the Q30 LSB, then drop the fraction
    wire signed [63:0] acc_rnd = acc + 64'sd536870912;
    wire signed [63:0] acc_sh  = acc_rnd >>> 30;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x1 <= 24'sd0;
            x2 <= 24'sd0;
            y1 <= 24'sd0;
            y2 <= 24'sd0;
            out_valid <= 1'b0;
            out_data  <= 24'sd0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                x2 <= x1;
                x1 <= x0;
                y2 <= y1;
                // saturate to 24 bit; feed the saturated value back
                if (acc_sh > 64'sd8388607) begin
                    y1 <= 24'sd8388607;
                    out_data <= 24'sd8388607;
                end else if (acc_sh < -64'sd8388608) begin
                    y1 <= -24'sd8388608;
                    out_data <= -24'sd8388608;
                end else begin
                    y1 <= acc_sh[23:0];
                    out_data <= acc_sh[23:0];
                end
            end
        end
    end
endmodule
