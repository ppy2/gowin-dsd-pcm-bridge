`timescale 1ns/1ps
// Input-rate detector for the stage-1 SRC. Fully MCLK-ratio based, so one
// image serves both domains (45.1584 MHz -> 176.4 kHz out,
// 49.152 MHz -> 192 kHz out): only the RATIO matters, never absolute Hz.
//
// Input LRCK period in MCLK ticks tells the input rate:
//   ~128  -> input = 2x output : sel = D2 (drop every 2nd pair)
//   ~256  -> input = 1x output : sel = X1 (pass)
//   ~512  -> input = 1x/2      : sel = X2 (duplicate each pair)
//   ~1024 -> input = 1x/4      : sel = X4 (duplicate x4)
// Anything else holds the previous sel (glitch ride-through).
// A sel change is committed only after 3 consecutive equal measurements.
// No LRCK edge for 1500 mclk -> in_idle (transport stopped/mute path).
module rate_detect (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       lrck_in,
    output reg  [1:0] sel,      // 00=D2 01=X1 10=X2 11=X4
    output reg        in_idle
);
    // localparam sel codes
    localparam [1:0] S_D2 = 2'b00;
    localparam [1:0] S_X1 = 2'b01;
    localparam [1:0] S_X2 = 2'b10;
    localparam [1:0] S_X4 = 2'b11;

    reg [1:0] lr_s;
    reg lr_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lr_s <= 2'b00;
            lr_d <= 1'b0;
        end else begin
            lr_s <= {lr_s[0], lrck_in};
            lr_d <= lr_s[1];
        end
    end
    wire rise = lr_s[1] & ~lr_d;

    reg [10:0] cnt;        // mclk since last rise (max 2047)
    reg [1:0]  cand;       // pending classification
    reg [1:0]  agree;      // consecutive agreements (0..2+, saturate)

    // Classify with +-4 tolerance (sync jitter is +-1; transport is exact).
    function [2:0] classify;
        input [10:0] n;
        begin
            if (n >= 11'd124 && n <= 11'd132)       classify = {1'b1, S_D2};
            else if (n >= 11'd252 && n <= 11'd260)  classify = {1'b1, S_X1};
            else if (n >= 11'd508 && n <= 11'd516)  classify = {1'b1, S_X2};
            else if (n >= 11'd1020 && n <= 11'd1028) classify = {1'b1, S_X4};
            else                                    classify = 3'b000;
        end
    endfunction

    wire [2:0] cl = classify(cnt);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 11'd0;
            cand <= S_X1;
            agree <= 2'd0;
            sel <= S_X1;      // default: pass-through
            in_idle <= 1'b1;  // silent until the grid is proven
        end else begin
            if (cnt != 11'd2047)
                cnt <= cnt + 11'd1;
            if (cnt == 11'd1500)
                in_idle <= 1'b1;   // LRCK stopped -> mute path
            if (rise) begin
                cnt <= 11'd0;
                in_idle <= 1'b0;
                if (cl[2]) begin
                    if (cl[1:0] == cand) begin
                        if (agree != 2'd2)
                            agree <= agree + 2'd1;
                        if (agree == 2'd2)
                            sel <= cl[1:0];   // 3rd consecutive: commit
                    end else begin
                        cand <= cl[1:0];
                        agree <= 2'd0;
                    end
                end else begin
                    agree <= 2'd0;   // unrecognized period: do not switch
                end
            end
        end
    end
endmodule
