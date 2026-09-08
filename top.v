`timescale 1ns/1ps
// Tang Primer 25K (GW5A-LV25MG121NC1/I0).
// Stage-1 SRC + 16-bit TPDF out. NO FIR (removed: it was a capability test).
// I2S in (32b slots) -> rate_detect -> dup/drop to fixed grid -> TPDF
// dither 24->16 -> I2S out (16b slots).
// ONE image, two domains (ratio logic, MCLK-agnostic):
//   44.1/88.2/176.4/352.8 kHz @MCLK 45.1584 MHz -> 176.4 kHz out (MCLK/256)
//   48/96/192/384 kHz @MCLK 49.152 MHz          -> 192 kHz out (MCLK/256)
// x1 = pass, x2/x4 = duplicate, x8 = drop every 2nd pair.
// All logic runs on mclk_in. No external reset: power-on reset counter.
module top (
    input  wire mclk_in,
    input  wire i2s_bclk_in,
    input  wire i2s_lrck_in,
    input  wire i2s_sdata_in,
    output wire i2s_bclk_out,
    output wire i2s_lrck_out,
    output wire i2s_sdata_out
);
    reg [5:0] por = 6'd0;
    always @(posedge mclk_in) begin
        if (por != 6'h3F)
            por <= por + 6'd1;
    end
    wire rst_n = (por == 6'h3F);

    wire pair_valid;
    wire signed [23:0] rx_l;
    wire signed [23:0] rx_r;

    wire [1:0] rate_sel;
    wire rate_idle;

    wire frame_tick;
    wire src_valid;
    wire signed [23:0] src_l;
    wire signed [23:0] src_r;

    wire dith_valid;
    wire signed [15:0] dith_l;
    wire signed [15:0] dith_r;

    i2s_receiver u_rx (
        .clk(mclk_in),
        .rst_n(rst_n),
        .bclk_in(i2s_bclk_in),
        .lrck_in(i2s_lrck_in),
        .sdata_in(i2s_sdata_in),
        .out_valid(pair_valid),
        .out_l(rx_l),
        .out_r(rx_r)
    );

    rate_detect u_det (
        .clk(mclk_in),
        .rst_n(rst_n),
        .lrck_in(i2s_lrck_in),
        .sel(rate_sel),
        .in_idle(rate_idle)
    );

    src_interp u_src (
        .clk(mclk_in),
        .rst_n(rst_n),
        .pair_valid(pair_valid),
        .in_l(rx_l),
        .in_r(rx_r),
        .frame_tick(frame_tick),
        .sel(rate_sel),
        .in_idle(rate_idle),
        .out_valid(src_valid),
        .out_l(src_l),
        .out_r(src_r)
    );

    dither_24_16 u_dith (
        .clk(mclk_in),
        .rst_n(rst_n),
        .in_valid(src_valid),
        .in_l(src_l),
        .in_r(src_r),
        .out_valid(dith_valid),
        .out_l(dith_l),
        .out_r(dith_r)
    );

    i2s_transmitter u_tx (
        .clk(mclk_in),
        .rst_n(rst_n),
        .in_valid(dith_valid),
        .in_l(dith_l),
        .in_r(dith_r),
        .bclk_out(i2s_bclk_out),
        .lrck_out(i2s_lrck_out),
        .sdata_out(i2s_sdata_out),
        .frame_tick(frame_tick)
    );
endmodule
