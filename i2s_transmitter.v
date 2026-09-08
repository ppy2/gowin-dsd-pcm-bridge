`timescale 1ns/1ps
// I2S master transmitter, Philips format, 16-bit slots @192 kHz.
// Clocks are derived from mclk_in (49.152 MHz):
//   BCLK = MCLK/8 = 6.144 MHz, LRCK = MCLK/256 = 192 kHz,
//   32 BCLK per stereo frame (16 per channel).
// Data changes on BCLK falling edges, MSB one BCLK after the LRCK edge.
// Sample payload: 16-bit dithered word, MSB-first, no padding.
module i2s_transmitter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire signed [15:0] in_l,
    input  wire signed [15:0] in_r,
    output wire        bclk_out,
    output reg         lrck_out,
    output reg         sdata_out,
    output wire        frame_tick   // 1-mclk pulse at output frame start
);
    reg [2:0] div;      // BCLK = MCLK/8
    reg [4:0] bit_cnt;  // 0..31 BCLK rises per stereo frame
    reg [15:0] shift;
    reg signed [15:0] buf_l;
    reg signed [15:0] buf_r;
    reg signed [15:0] hold_l;
    reg signed [15:0] hold_r;

    assign bclk_out = div[2];
    assign frame_tick = (div == 3'd0) && (bit_cnt == 5'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div <= 3'd0;
            bit_cnt <= 5'd0;
            shift <= 16'd0;
            lrck_out <= 1'b0;
            sdata_out <= 1'b0;
            buf_l <= 16'sd0;
            buf_r <= 16'sd0;
            hold_l <= 16'sd0;
            hold_r <= 16'sd0;
        end else begin
            div <= div + 3'd1;
            if (in_valid) begin
                buf_l <= in_l;
                buf_r <= in_r;
            end
            if (div == 3'd4) begin
                // BCLK rising edge: advance the frame counter
                if (bit_cnt == 5'd31)
                    bit_cnt <= 5'd0;
                else
                    bit_cnt <= bit_cnt + 5'd1;
            end
            if (div == 3'd0) begin
                // BCLK falling edge: LRCK + data (Philips timing).
                // sdata gets the OLD shift MSB: one BCLK of delay.
                sdata_out <= shift[15];
                if (bit_cnt == 5'd0) begin
                    // Frame start: BOTH slots must come from the snapshot
                    // latched HERE. (The old code loaded shift with the
                    // pre-update hold_l, so on changing data L lagged R by
                    // a whole frame: 20.8 us @48k. shift must use the
                    // post-update value.)
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
