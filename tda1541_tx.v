`timescale 1ns/1ps
// TDA1541(A) simultaneous-mode transmitter (master, MCLK-derived).
// Functionality per miro1360/MT02 (I2S-simultaneous-TDA1541A): 16-bit
// words L+R in parallel, MSB-first, MSB INVERTED (offset binary for the
// TDA1541A), 16 stopped BCK pulses per sample, LE latch pulse (4 BCK)
// after the last bit, BCK low otherwise.
//
// Unlike the reference (which resynchronizes an input 64fs I2S stream),
// this block is clocked by our own MCLK/frame_tick grid and fed the same
// post-dither 16-bit pair as the I2S TX, so both outputs always agree.
// BCK = MCLK/8 (5.6/6.1 MHz, 32/frame budget, 16 used + LE + idle).
// Frame schedule (t = MCLK since frame_tick):
//   t=0        : present bit15 (MSB, inverted); BCK starts low
//   t=4..124   : 16 BCK rises sample bits 15..0 (data changes on falling
//                edges t=8..120, 4 MCLK setup/hold both sides)
//   t=128      : data lines to 0, BCK stops (ends on a falling edge: clean)
//   t=132..164 : LE high (4 BCK periods)
//
// Timing margins: first rise 4 MCLK after bit present, LE 8 MCLK
// after the last rise, 92 MCLK LE-off before the next frame.
module tda1541_tx (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              in_valid,   // new dithered pair (any phase)
    input  wire signed [15:0] in_l,
    input  wire signed [15:0] in_r,
    input  wire              frame_tick, // 1-mclk pulse, output frame start
    output wire              bck_out,
    output reg               le_out,
    output reg               dl_out,
    output reg               dr_out
);
    reg signed [15:0] buf_l, buf_r;   // last pair (starve -> repeat, = I2S TX)
    reg [15:0] sh_l, sh_r;            // offset-binary shift regs
    reg [7:0]  t;                     // MCLK phase in frame
    reg        running;

    wire signed [15:0] use_l = in_valid ? in_l : buf_l;
    wire signed [15:0] use_r = in_valid ? in_r : buf_r;

    assign bck_out = (running && (t < 8'd128)) ? t[2] : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_l <= 16'sd0; buf_r <= 16'sd0;
            sh_l <= 16'd0; sh_r <= 16'd0;
            t <= 8'd0; running <= 1'b0;
            le_out <= 1'b0; dl_out <= 1'b0; dr_out <= 1'b0;
        end else begin
            if (in_valid) begin
                buf_l <= in_l;
                buf_r <= in_r;
            end
            if (frame_tick) begin
                // Latch offset-binary words (MSB inverted), present bit15.
                sh_l <= {~use_l[15], use_l[14:0]};
                sh_r <= {~use_r[15], use_r[14:0]};
                dl_out <= ~use_l[15];
                dr_out <= ~use_r[15];
                t <= 8'd0;
                running <= 1'b1;
                le_out <= 1'b0;
            end else if (running) begin
                t <= t + 8'd1;
                if (t < 8'd120 && t[2:0] == 3'b111) begin
                    // BCK falling edge: present the NEXT bit (shift
                    // first — presenting sh[15] here would repeat the
                    // bit just sampled on the rise).
                    dl_out <= sh_l[14];
                    dr_out <= sh_r[14];
                    sh_l <= {sh_l[14:0], 1'b0};
                    sh_r <= {sh_r[14:0], 1'b0};
                end
                if (t == 8'd128) begin
                    dl_out <= 1'b0;
                    dr_out <= 1'b0;
                end
                if (t == 8'd132)
                    le_out <= 1'b1;
                if (t == 8'd164) begin
                    le_out <= 1'b0;
                    running <= 1'b0;
                end
            end
        end
    end
endmodule
