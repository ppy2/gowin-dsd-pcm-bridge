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
        .rst_n(pcm_rst_n),
        .bclk_in(i2s_bclk_in),
        .lrck_in(i2s_lrck_in),
        .sdata_in(i2s_sdata_in),
        .out_valid(pair_valid),
        .out_l(rx_l),
        .out_r(rx_r)
    );

    rate_detect u_det (
        .clk(mclk_in),
        .rst_n(pcm_rst_n),
        .lrck_in(i2s_lrck_in),
        .sel(rate_sel),
        .in_idle(rate_idle)
    );

    src_interp u_src (
        .clk(mclk_in),
        .rst_n(pcm_rst_n),
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

    // PCM-path re-init around DSD: while dsd_active the shared pins carry
    // the DSD bit clock + DATA garbage straight into the PCM receiver,
    // rate detector and SRC (flooded FIFOs/histories/sel). Whatever state
    // that leaves behind, wipe it: hold receiver + rate_detect + src in
    // reset for the whole DSD session and 65536 MCLK (~1.5 ms) after the
    // fall, so transports clocks stabilize first. Release lands inside
    // the switch HOLD (5.8 ms), PCM relocks in 3 frames underneath.
    // Dither/TX/fade keep running (no latchable state there) so output
    // clocks never stop.
    reg dsd_prev;
    reg fall_hold;
    reg [15:0] fall_cnt;
    always @(posedge mclk_in or negedge rst_n) begin
        if (!rst_n) begin
            dsd_prev <= 1'b0; fall_hold <= 1'b0; fall_cnt <= 16'd0;
        end else begin
            dsd_prev <= dsd_active;
            if (dsd_active) begin
                fall_hold <= 1'b0; fall_cnt <= 16'd0;
            end else if (dsd_prev && !dsd_active) begin
                fall_hold <= 1'b1; fall_cnt <= 16'hFFFF;
            end else if (fall_hold) begin
                if (fall_cnt == 16'd0)
                    fall_hold <= 1'b0;
                else
                    fall_cnt <= fall_cnt - 16'd1;
            end
        end
    end
    wire pcm_rst_n = rst_n & ~dsd_active & ~fall_hold;

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

    // Click-free source switching: a hard music<->silence step is itself
    // a full-scale click, so blanking alone is not enough. Envelope:
    //   EDGE -> FADE_OUT the still-served source over 128 output frames
    //           (~0.7 ms; the old clocks are usually already dying, the
    //           ramp covers the cases they linger) ->
    //           flip the mux, HOLD zeros 2^18 MCLK (~5.8 ms @45.1584) while
    //           DSD acq+FIR-fill / PCM relock+flush settle ->
    //           FADE_IN the new source over 1024 frames (~5.3 ms) -> STEADY.
    // Zero-fill (one zero pair per output frame) covers valid gaps; clocks
    // never stop. POR starts in HOLD (also silences power-up garbage).
    // Gain mult is 24x11 bit-exact at full gain (x1024>>10 = x1); Yosys
    // gate expects 5 $mul (3 DSP path + 2 fade), LUT-mapped, timing-clean.
    localparam [1:0] ST_STEADY = 2'd0, ST_OUT = 2'd1,
                     ST_HOLD = 2'd2, ST_IN = 2'd3;
    reg [1:0] sw_state;
    reg mux_frozen;      // source actually served (flips only at gain 0)
    reg [10:0] fgain;    // 0..1024
    reg [17:0] hold;
    always @(posedge mclk_in or negedge rst_n) begin
        if (!rst_n) begin
            sw_state <= ST_HOLD; mux_frozen <= 1'b0;
            fgain <= 11'd0; hold <= 18'h3FFFF;
        end else begin
            case (sw_state)
                ST_STEADY: begin
                    fgain <= 11'd1024;
                    if (dsd_active != mux_frozen)
                        sw_state <= ST_OUT;
                end
                ST_OUT: begin
                    // Edge here just keeps fading: the flip at gain 0
                    // always takes the latest dsd_active.
                    if (frame_tick) begin
                        if (fgain <= 11'd8) begin
                            fgain <= 11'd0;
                            mux_frozen <= dsd_active;
                            hold <= 18'h3FFFF;
                            sw_state <= ST_HOLD;
                        end else
                            fgain <= fgain - 11'd8;
                    end
                end
                ST_HOLD: begin
                    if (dsd_active != mux_frozen) begin
                        mux_frozen <= dsd_active;
                        hold <= 18'h3FFFF;
                    end else if (hold != 18'd0)
                        hold <= hold - 18'd1;
                    else
                        sw_state <= ST_IN;
                end
                ST_IN: begin
                    if (dsd_active != mux_frozen)
                        sw_state <= ST_OUT;
                    else if (frame_tick) begin
                        if (fgain >= 11'd1024) begin
                            fgain <= 11'd1024;
                            sw_state <= ST_STEADY;
                        end else
                            fgain <= fgain + 11'd1;
                    end
                end
            endcase
        end
    end

    // Frozen source (with the measured DSD L/R swap) + gain.
    // |fz|*1024>>10 never overflows S24 (worst case -FS maps to -FS).
    wire signed [23:0] fz_l = mux_frozen ? dsd_r24 : src_l;
    wire signed [23:0] fz_r = mux_frozen ? dsd_l24 : src_r;
    wire fz_v = mux_frozen ? dsd_valid_176 : src_valid;
    // Coast register: the last sample actually emitted to the dither.
    // Fading the live source breaks when the old path collapses mid-ramp
    // (in_idle zeros arrive as *valid* data a few frames after the pins
    // freeze, DSD valids stop with the bit clock): the ramp would step
    // off that cliff. Coasting the last emitted sample is continuous by
    // construction in every case — live, collapsing, or long-dead
    // (then it equals what the TX already repeats, and fades that out).
    // Tracked in STEADY/IN, frozen in OUT/HOLD. Same two mults.
    reg signed [23:0] coast_l, coast_r;
    always @(posedge mclk_in or negedge rst_n) begin
        if (!rst_n) begin
            coast_l <= 24'sd0; coast_r <= 24'sd0;
        end else if (dith_in_valid &&
                     ((sw_state == ST_STEADY) || (sw_state == ST_IN))) begin
            coast_l <= dith_in_l; coast_r <= dith_in_r;
        end
    end
    wire signed [23:0] m_in_l =
        (sw_state == ST_OUT) ? coast_l : fz_l;
    wire signed [23:0] m_in_r =
        (sw_state == ST_OUT) ? coast_r : fz_r;
    wire signed [34:0] fprod_l = $signed(m_in_l) * $signed({1'b0, fgain});
    wire signed [34:0] fprod_r = $signed(m_in_r) * $signed({1'b0, fgain});
    wire signed [23:0] sc_l = fprod_l[33:10];
    wire signed [23:0] sc_r = fprod_r[33:10];

    wire filling = (sw_state != ST_STEADY);
    // Zero-fill only when the served source is actually silent this
    // frame: a fill between live valids would chop music with zeros
    // (full-scale steps — the bench caught exactly that). vseen tracks
    // whether any source valid arrived since the previous frame tick.
    reg vseen;
    always @(posedge mclk_in or negedge rst_n) begin
        if (!rst_n)
            vseen <= 1'b0;
        else if (fz_v)
            vseen <= 1'b1;
        else if (frame_tick)
            vseen <= 1'b0;
    end
    wire fill_tick = filling & frame_tick & ~vseen & ~fz_v;
    wire dith_in_valid = fz_v | fill_tick;
    // OUT fills coast on the held sample (never 0 — that step is the
    // click); anywhere else a fill is true silence. HOLD needs no case:
    // gain is 0 there so sc is 0 already.
    wire signed [23:0] dith_in_l =
        (sw_state == ST_OUT) ? sc_l : (fz_v ? sc_l : 24'sd0);
    wire signed [23:0] dith_in_r =
        (sw_state == ST_OUT) ? sc_r : (fz_v ? sc_r : 24'sd0);

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
