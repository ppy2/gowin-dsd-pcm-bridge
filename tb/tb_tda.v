`timescale 1ns/1ps
// TDA1541(A) simultaneous TX bench.
// Part 1 (unit): known words -> 16 stopped BCK rises, MSB-first offset
//   binary on DL/DR, LE 0 during data, 4-BCK LE pulse after, BCK low
//   otherwise, data 0 outside the window, starve repeats buf.
// Part 2 (through-top): X1 PCM DC -> TDA frame decodes to the same
//   dithered DC as the I2S path (offset binary), 16 BCK, LE pulses.
module tb_tda;
    reg mclk = 1'b0;
    always #11.072 mclk = ~mclk;   // 45.1584 MHz

    integer err = 0;

    // ---------------- Part 1: unit ----------------
    reg u_valid = 1'b0;
    reg signed [15:0] u_l = 16'sd0, u_r = 16'sd0;
    reg u_tick = 1'b0;
    wire u_bck, u_le, u_dl, u_dr;
    tda1541_tx uut (
        .clk(mclk), .rst_n(1'b1),
        .in_valid(u_valid), .in_l(u_l), .in_r(u_r),
        .frame_tick(u_tick),
        .bck_out(u_bck), .le_out(u_le), .dl_out(u_dl), .dr_out(u_dr)
    );

    reg u_bck_d = 1'b0;
    always @(posedge mclk) u_bck_d <= u_bck;
    wire u_rise = u_bck & ~u_bck_d;
    wire u_fall = ~u_bck & u_bck_d;

    integer rises, falls_idle_bck;
    reg [15:0] got_l, got_r;
    reg le_seen_data, le_ok;
    integer le_on_t, le_off_t, cyc;
    reg [15:0] exp_l, exp_r;

    task one_frame;
        input [15:0] wl;
        input [15:0] wr;
        input [15:0] el;   // expected (starve: previous pair repeats)
        input [15:0] er;
        input drive_valid;   // 0 = starve (must repeat previous)
        begin
            exp_l = el; exp_r = er;
            @(posedge mclk);
            u_l = wl; u_r = wr; u_valid = drive_valid;
            u_tick = 1'b1;
            @(posedge mclk); #1;
            u_tick = 1'b0; u_valid = 1'b0;
            rises = 0; falls_idle_bck = 0;
            got_l = 0; got_r = 0;
            le_seen_data = 1'b0; le_on_t = -1; le_off_t = -1;
            for (cyc = 0; cyc < 300; cyc = cyc + 1) begin
                @(posedge mclk); #1;
                if (u_rise) begin
                    rises = rises + 1;
                    got_l = {got_l[14:0], u_dl};
                    got_r = {got_r[14:0], u_dr};
                    if (u_le) le_seen_data = 1'b1;
                end
                if (u_fall && cyc > 140) falls_idle_bck = falls_idle_bck + 1;
                if (u_le && le_on_t < 0) le_on_t = cyc;
                if (!u_le && le_on_t >= 0 && le_off_t < 0) le_off_t = cyc;
            end
            if (rises !== 16) begin
                $display("FAIL: rises=%0d want 16", rises); err = err + 1;
            end
            if (got_l !== {~exp_l[15], exp_l[14:0]}) begin
                $display("FAIL: DL got %h want OB(%h)", got_l, exp_l);
                err = err + 1;
            end
            if (got_r !== {~exp_r[15], exp_r[14:0]}) begin
                $display("FAIL: DR got %h want OB(%h)", got_r, exp_r);
                err = err + 1;
            end
            if (le_seen_data) begin
                $display("FAIL: LE high during data"); err = err + 1;
            end
            if (le_on_t < 0 || (le_off_t - le_on_t) < 28
                || (le_off_t - le_on_t) > 36) begin
                $display("FAIL: LE pulse on=%0d off=%0d (want 32 wide)",
                    le_on_t, le_off_t); err = err + 1;
            end
            if (falls_idle_bck !== 0) begin
                $display("FAIL: BCK toggles after window (%0d)",
                    falls_idle_bck); err = err + 1;
            end
        end
    endtask

    // ---------------- Part 2: through-top ----------------
    reg i_bclk = 1'b0, i_lrck = 1'b0, i_sdata = 1'b0;
    reg pcm_run = 1'b0;
    reg [1:0] bdiv = 2'd0;
    reg [5:0] fpos = 6'd0;
    always @(negedge mclk) begin
        bdiv <= bdiv + 2'd1;
        if (pcm_run && bdiv == 2'd2) begin
            i_bclk <= 1'b1;
            fpos <= (fpos == 6'd63) ? 6'd0 : fpos + 6'd1;
        end else if (pcm_run && bdiv == 2'd0) begin
            i_bclk <= 1'b0;
            if (fpos == 6'd0) begin i_lrck <= 1'b0; i_sdata <= 1'b0; end
            else if (fpos == 6'd32) begin i_lrck <= 1'b1; i_sdata <= 1'b0; end
            else if (fpos >= 6'd1 && fpos <= 6'd31)
                i_sdata <= (32'h20000000 >> (32 - fpos)) & 1'b1;  // +0.25FS
            else i_sdata <= (32'hE0000000 >> (64 - fpos)) & 1'b1; // -0.25FS
        end
    end

    wire t_bck, t_le, t_dl, t_dr;
    top #(.DB_BITS(6)) top_i (
        .mclk_in(mclk),
        .i2s_bclk_in(i_bclk), .i2s_lrck_in(i_lrck),
        .i2s_sdata_in(i_sdata), .dsd_on(1'b0), .dsd_data2_in(1'b0),
        .i2s_bclk_out(), .i2s_lrck_out(), .i2s_sdata_out(),
        .tda_bck_out(t_bck), .tda_le_out(t_le),
        .tda_dl_out(t_dl), .tda_dr_out(t_dr)
    );
    reg t_bck_d = 1'b0;
    always @(posedge mclk) t_bck_d <= t_bck;

    integer wc, tr, tw_l, tw_r, le_n;
    initial begin
        // ---- Part 1 ----
        repeat (10) @(posedge mclk);
        one_frame(16'h7FFF, 16'h8000, 16'h7FFF, 16'h8000, 1'b1);
        one_frame(16'h0000, 16'hFFFF, 16'h0000, 16'hFFFF, 1'b1);
        one_frame(16'h4000, 16'hC000, 16'h4000, 16'hC000, 1'b1);
        one_frame(16'hAAAA, 16'h5555, 16'h4000, 16'hC000, 1'b0);
        if (err == 0) $display("unit tda1541_tx ok");

        // ---- Part 2 ----
        pcm_run = 1'b1;
        wc = 0;
        while (top_i.sw_state !== 2'd0 && wc < 2000000) begin
            @(posedge mclk); wc = wc + 1;
        end
        if (top_i.sw_state !== 2'd0) begin
            $display("FAIL: top no STEADY"); err = err + 1;
        end
        repeat (40 * 256) @(posedge mclk);
        // capture one TDA frame
        tr = 0; tw_l = 0; tw_r = 0; le_n = 0;
        @(posedge mclk);
        while (top_i.frame_tick !== 1'b1) begin @(posedge mclk); end
        // 240 cycles: the 16 rises (+4..+124) and LE (+132..164) fit;
        // the next frame's first rise (+260) stays outside.
        repeat (240) begin
            @(posedge mclk); #1;
            if (t_bck && !t_bck_d) begin
                tr = tr + 1;
                tw_l = {tw_l[14:0], t_dl};
                tw_r = {tw_r[14:0], t_dr};
                if (tr <= 2 || tr >= 16)
                    $display("RISE%0d dl=%b dr=%b le=%b", tr, t_dl, t_dr, t_le);
            end
            if (t_le) le_n = le_n + 1;
        end
        // +0.25FS -> ~0x2000 -> OB ~0xA000; -0.25FS -> ~0xE000 -> ~0x6000
        $display("TOP-TDA: L=%h R=%h rises=%0d", tw_l, tw_r, tr);
        if (tr !== 16) begin $display("FAIL: top TDA rises"); err = err + 1; end
        if (!(tw_l > 16'h9FF0 && tw_l < 16'hA010)) begin
            $display("FAIL: top TDA L word"); err = err + 1;
        end
        if (!(tw_r > 16'h5FF0 && tw_r < 16'h6010)) begin
            $display("FAIL: top TDA R word"); err = err + 1;
        end
        if (le_n == 0) begin $display("FAIL: top TDA no LE"); err = err + 1; end

        if (err == 0) $display("PASS tb_tda");
        else $display("FAIL tb_tda (%0d)", err);
        $finish;
    end

    initial begin
        repeat (6000000) @(posedge mclk);
        $display("WATCHDOG hang"); $finish;
    end
endmodule
