`timescale 1ns/1ps
// I2S master transmitter, Philips format, 16-bit slots @384 kHz.
// Clocks are derived from mclk_in (49.152 MHz):
//   BCLK = MCLK/4 = 12.288 MHz, LRCK = MCLK/128 = 384 kHz,
//   32 BCLK per stereo frame (16 per channel).
// Data changes on BCLK falling edges, MSB one BCLK after the LRCK edge.
// Sample payload: 16-bit dithered word, MSB-first, no padding.
// Two frame ticks: frame_tick every 128 MCLK (output grid) and
// frame_tick_s1 every 256 MCLK (stage-1 grid = every other output frame,
// deterministic phase from POR; the SRC tolerates any constant phase).
module i2s_transmitter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire signed [15:0] in_l,
    input  wire signed [15:0] in_r,
    output wire        bclk_out,
    output reg         lrck_out,
    output reg         sdata_out,
    output wire        frame_tick,    // 1-mclk pulse at output frame start
    output wire        frame_tick_s1  // 1-mclk pulse at stage-1 frame start
);
    reg [1:0] div;      // BCLK = MCLK/4
    reg [4:0] bit_cnt;  // 0..31 BCLK rises per stereo frame
    reg       fr_par;   // frame parity: stage-1 tick every other frame
    reg [15:0] shift;
    reg signed [15:0] buf_l;
    reg signed [15:0] buf_r;
    reg signed [15:0] hold_l;
    reg signed [15:0] hold_r;

    assign bclk_out = div[1];
    assign frame_tick = (div == 2'd0) && (bit_cnt == 5'd0);
    assign frame_tick_s1 = frame_tick && (fr_par == 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div <= 2'd0;
            bit_cnt <= 5'd0;
            fr_par <= 1'b0;
            shift <= 16'd0;
            lrck_out <= 1'b0;
            sdata_out <= 1'b0;
            buf_l <= 16'sd0;
            buf_r <= 16'sd0;
            hold_l <= 16'sd0;
            hold_r <= 16'sd0;
        end else begin
            div <= div + 2'd1;
            if (in_valid) begin
                buf_l <= in_l;
                buf_r <= in_r;
            end
            if (div == 2'd2) begin
                // BCLK rising edge: advance the frame counter
                if (bit_cnt == 5'd31)
                    bit_cnt <= 5'd0;
                else
                    bit_cnt <= bit_cnt + 5'd1;
            end
            if (div == 2'd0) begin
                // BCLK falling edge: LRCK + data (Philips timing).
                // sdata gets the OLD shift MSB: one BCLK of delay.
                sdata_out <= shift[15];
                if (bit_cnt == 5'd0) begin
                    // Frame start: BOTH slots must come from the snapshot
                    // latched HERE (shift uses the post-update value, else
                    // L lags R by a whole frame on changing data).
                    if (in_valid) begin
                        hold_l <= in_l;
                        hold_r <= in_r;
                        shift <= in_l;
                    end else begin
                        hold_l <= buf_l;
                        hold_r <= buf_r;
                        shift <= buf_l;
                    end
                    lrck_out <= 1'b0;
                    fr_par <= ~fr_par;
                end else if (bit_cnt == 5'd16) begin
                    shift <= hold_r;
                    lrck_out <= 1'b1;
                end else begin
                    shift <= {shift[14:0], 1'b0};
                end
            end
        end
    end
endmodule
