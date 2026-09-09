`timescale 1ns/1ps
// Tang Primer 25K (GW5A-LV25MG121NC1/I0).
// PCM: Stage-1 SRC + 16-bit TPDF out (see below).
// DSD (dsd_on high, Amanero-style): DSD64..512 -> dsd_to_pcm (352.8 kHz)
// -> dsd_pcm_decim2 /2 (176.4 kHz) -> the same TPDF dither + I2S out.
// DSD requires the 44.1-domain transport clock (MCLK 45.1584 MHz):
// DSD rates are 44.1-family and the output grid is fixed MCLK/256.
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
    input  wire dsd_on,            // native-DSD enable (Amanero DSD-on)
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

    // ---------------- native DSD path (Amanero-style) ----------------
    // dsd_on high: BCLK pin = DSD bit clock (2.8224..22.5792 MHz),
    // SDATA = DATA1, LRCLK = DATA2. The PCM path above keeps running
    // (its LRCK input is DATA2 garbage in DSD mode); the mux only
    // switches the data source, so mode flips are glitch-free.
    reg [1:0] dsd_on_sync;
    always @(posedge mclk_in or negedge rst_n) begin
        if (!rst_n)
            dsd_on_sync <= 2'b00;
        else
            dsd_on_sync <= {dsd_on_sync[0], dsd_on};
    end
    wire dsd_active = dsd_on_sync[1];

    wire dsd_valid_352;
    wire signed [31:0] dsd_352_l;
    wire signed [31:0] dsd_352_r;

    dsd_to_pcm dsd_stage1 (
        .mclk(mclk_in),
        .rst_n(rst_n),
        .dsd_on(dsd_on),
        .dsd_clk_in(i2s_bclk_in),
        .dsd_data1(i2s_sdata_in),
        .dsd_data2_alt(1'b0),   // Amanero style: DATA2 rides the LRCLK pin
        .lrck_in(i2s_lrck_in),
        .sample_valid(dsd_valid_352),
        .sample_l(dsd_352_l),
        .sample_r(dsd_352_r)
    );

    wire dsd_valid_176;
    wire signed [31:0] dsd_176_l;
    wire signed [31:0] dsd_176_r;

    dsd_pcm_decim2 dsd_stage2 (
        .clk(mclk_in),
        .rst_n(rst_n),
        .in_valid(dsd_valid_352),
        .in_l(dsd_352_l),
        .in_r(dsd_352_r),
        .out_valid(dsd_valid_176),
        .out_l(dsd_176_l),
        .out_r(dsd_176_r)
    );

    // S32 (DC gain ~2^31, FS-to-FS) -> S24 by round-half-up with
    // saturation (shared module, bit-identical in top and bench).
    wire signed [23:0] dsd_l24;
    wire signed [23:0] dsd_r24;
    dsd_round_s32_s24 round_l (.in_s32(dsd_176_l), .out_s24(dsd_l24));
    dsd_round_s32_s24 round_r (.in_s32(dsd_176_r), .out_s24(dsd_r24));

    // Measured on hardware (2026-09-09): this transport delivers the RIGHT
    // channel on SDATA and LEFT on LRCLK in DSD mode (opposite of the
    // DATA1=left assumption in dsd_to_pcm.v). Swapped here so DSD L/R
    // matches PCM L/R. dsd_to_pcm.v itself stays verbatim with /mnt/sdb/fpga.
    wire src_mux_valid = dsd_active ? dsd_valid_176 : src_valid;
    wire signed [23:0] src_mux_l = dsd_active ? dsd_r24 : src_l;
    wire signed [23:0] src_mux_r = dsd_active ? dsd_l24 : src_r;

    // Switch blanking: the newly-selected path is not ready at the mux
    // edge (DSD: ~1 ms acquisition + FIR window fill from zero; PCM:
    // rate relock + history flush of DSD-era garbage). Switching bare
    // emits a frozen-DC + rail-ramp burst — the loud switch transient.
    // Blank 2^18 MCLK (~5.8 ms @45.1584) with exactly one zero pair per
    // output frame (dithered silence, clocks keep running); the new path
    // settles underneath. Also covers POR (blank init, not an edge).
    reg [17:0] blank;
    reg dsd_active_d;
    always @(posedge mclk_in or negedge rst_n) begin
        if (!rst_n) begin
            blank <= 18'h3FFFF;
            dsd_active_d <= 1'b0;
        end else begin
            dsd_active_d <= dsd_active;
            if (dsd_active != dsd_active_d)
                blank <= 18'h3FFFF;
            else if (blank != 18'd0)
                blank <= blank - 18'd1;
        end
    end
    wire blanking = (blank != 18'd0);
    wire dith_in_valid = blanking ? frame_tick : src_mux_valid;
    wire signed [23:0] dith_in_l = blanking ? 24'sd0 : src_mux_l;
    wire signed [23:0] dith_in_r = blanking ? 24'sd0 : src_mux_r;

    dither_24_16 u_dith (
        .clk(mclk_in),
        .rst_n(rst_n),
        .in_valid(dith_in_valid),
        .in_l(dith_in_l),
        .in_r(dith_in_r),
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
