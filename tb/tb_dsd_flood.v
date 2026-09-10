`timescale 1ns/1ps
// Hostile-transition bench: PCM -> DSD-with-garbage-flood -> PCM.
// Same wires carry the DSD bit clock + dense DATA into the PCM
// receiver/rate-detector during DSD (the frozen-pins bench never
// modeled this). Proves the PCM path survives the flood: per-channel
// resume levels after dsd_on falls.
module tb_dsd_flood;
    reg mclk = 1'b0;
    always #11.072 mclk = ~mclk;   // 45.1584 MHz

    // ---- shared transport pins (I2S in PCM, DSD bitstream in DSD) ----
    reg i2s_bclk_in = 1'b0;
    reg i2s_lrck_in = 1'b0;
    reg i2s_sdata_in = 1'b0;
    reg dsd_on = 1'b0;
    reg pcm_run = 1'b0;   // 1 = bit-bang PCM I2S frames

    // PCM generator: overall-X2 stimulus (LRCK=MCLK/256 = 192k @49.152;
    // x384 out is MCLK/128), 32-bit slots, BCLK=MCLK/4.
    // L = +0.5FS DC (0x40000000), R = -0.5FS DC (0xC0000000).
    reg [1:0] bdiv = 2'd0;
    reg [5:0] fpos = 6'd0;
    always @(negedge mclk) begin
        bdiv <= bdiv + 2'd1;
        // No else branch on purpose: when pcm_run=0 the DSD task owns
        // these pins; forced zeros here would fight it every negedge
        // and corrupt the bitstream (seen: both rails stick at -FS).
        if (pcm_run) begin
            if (bdiv == 2'd2) begin
                i2s_bclk_in <= 1'b1;
                fpos <= (fpos == 6'd63) ? 6'd0 : fpos + 6'd1;
            end else if (bdiv == 2'd0) begin
                i2s_bclk_in <= 1'b0;
                if (fpos == 6'd0) begin
                    i2s_lrck_in <= 1'b0;
                    i2s_sdata_in <= 1'b0;
                end else if (fpos == 6'd32) begin
                    i2s_lrck_in <= 1'b1;
                    i2s_sdata_in <= 1'b0;
                end else if (fpos >= 6'd1 && fpos <= 6'd31) begin
                    i2s_sdata_in <= (32'h40000000 >> (32 - fpos)) & 1'b1;
                end else begin
                    i2s_sdata_in <= (32'hC0000000 >> (64 - fpos)) & 1'b1;
                end
            end
        end
    end

    top #(.DB_BITS(6)) top_i (
        .mclk_in(mclk),
        .i2s_bclk_in(i2s_bclk_in),
        .i2s_lrck_in(i2s_lrck_in),
        .i2s_sdata_in(i2s_sdata_in),
        .dsd_on(dsd_on),
        .dsd_data2_in(1'b0),
        .nos_bypass(1'b0),
        .i2s_bclk_out(),
        .i2s_lrck_out(),
        .i2s_sdata_out()
    );

    // DSD DC drive on the shared pins (DSD64: 16 mclk per bit).
    // Music-like DSD64 drive: SDATA mostly ones (loud L), LRCLK pin
    // toggling densely (active R). On hardware these same wires feed the
    // PCM receiver/rate-detector with a ~2.8 MHz bit clock + garbage —
    // the flood the frozen-pins bench never modeled.
    task dsd_music;
        input integer nbits;
        integer k, h;
        begin
            for (k = 0; k < nbits; k = k + 1) begin
                i2s_sdata_in = (k % 5 < 3);
                if (k % 3 == 0) i2s_lrck_in = ~i2s_lrck_in;
                i2s_bclk_in = 1'b0;
                for (h = 0; h < 8; h = h + 1) @(posedge mclk);
                i2s_bclk_in = 1'b1;
                for (h = 0; h < 8; h = h + 1) @(posedge mclk);
            end
            i2s_bclk_in = 1'b0;
        end
    endtask

    task dsd_dc;
        input integer nbits;
        input b1;
        input b2;
        integer k, h;
        begin
            for (k = 0; k < nbits; k = k + 1) begin
                i2s_sdata_in = b1; i2s_lrck_in = b2;
                i2s_bclk_in = 1'b0;
                for (h = 0; h < 8; h = h + 1) @(posedge mclk);
                i2s_bclk_in = 1'b1;
                for (h = 0; h < 8; h = h + 1) @(posedge mclk);
            end
            i2s_bclk_in = 1'b0;
        end
    endtask

    integer err = 0;
    integer vc, cc;
    integer ms_b, nz_b, ms_c, nz_c;
    reg signed [23:0] acc_l, acc_r;
    integer n;

    // Count src_mux_valid over 512 mclk (expect 4 = 1 pair/128, x384 grid)
    // average the muxed data.
    task sniff_mux;
        output integer valids;
        output signed [23:0] mean_l;
        output signed [23:0] mean_r;
        integer c;
        reg signed [31:0] sl, sr;
        begin
            valids = 0; sl = 0; sr = 0;
            for (c = 0; c < 512; c = c + 1) begin
                @(posedge mclk); #1;
                if (top_i.src_mux_valid === 1'b1) begin
                    valids = valids + 1;
                    sl = sl + $signed(top_i.src_mux_l);
                    sr = sr + $signed(top_i.src_mux_r);
                end
            end
            mean_l = (valids > 0) ? sl / valids : 0;
            mean_r = (valids > 0) ? sr / valids : 0;
        end
    endtask

    // Envelope monitor on the post-fade dither input: largest step
    // between consecutive emitted samples (a hard music<->silence step
    // is ~4M+ LSB24 = the click) + zero samples seen (the hold).
    task env_check;
        input integer cycles;
        output integer maxstep;
        output integer nzeros;
        integer c, d;
        reg signed [23:0] prev_l, prev_r;
        reg have_prev;
        begin
            maxstep = 0; nzeros = 0; have_prev = 1'b0;
            for (c = 0; c < cycles; c = c + 1) begin
                @(posedge mclk); #1;
                if (top_i.u_dith.in_valid === 1'b1) begin
                    if ($signed(top_i.u_dith.in_l) === 0 &&
                        $signed(top_i.u_dith.in_r) === 0)
                        nzeros = nzeros + 1;
                    if (have_prev) begin
                        d = $signed(top_i.u_dith.in_l) - $signed(prev_l);
                        if (d < 0) d = -d;
                        if (d > maxstep) maxstep = d;
                        d = $signed(top_i.u_dith.in_r) - $signed(prev_r);
                        if (d < 0) d = -d;
                        if (d > maxstep) maxstep = d;
                    end
                    prev_l = top_i.u_dith.in_l;
                    prev_r = top_i.u_dith.in_r;
                    have_prev = 1'b1;
                end
            end
        end
    endtask

    initial begin
        repeat (200) @(posedge mclk); // POR + settle

        // ---- A. PCM baseline (wait for the POR hold+fade-in) ----
        pcm_run = 1'b1;
        begin : lockwait_a
            integer wc;
            wc = 0;
            while (top_i.sw_state !== 2'd0 && wc < 2000000) begin
                @(posedge mclk); wc = wc + 1;
            end
            if (top_i.sw_state !== 2'd0) begin
                $display("FAIL: no STEADY-A"); err = err + 1;
            end
        end
        repeat (40*256) @(posedge mclk); // rate lock + SRC flush
        sniff_mux(vc, acc_l, acc_r);
        $display("PCM-A: valids=%0d/512 meanL=%0d meanR=%0d", vc, $signed(acc_l), $signed(acc_r));
        if (vc != 4) begin $display("FAIL: PCM-A rate"); err = err + 1; end
        if (!($signed(acc_l) > 4000000)) begin $display("FAIL: PCM-A L not +0.5FS"); err = err + 1; end
        if (!($signed(acc_r) < -4000000)) begin $display("FAIL: PCM-A R not -0.5FS"); err = err + 1; end

        // ---- B1. DSD music-like flood (the hardware reality) ----
        // Envelope under observation; no data assertions here (DSD
        // content is music-like) — only: no click-step, silent hold,
        // no hang, and the PCM side must survive the flood.
        pcm_run = 1'b0;
        repeat (10) @(posedge mclk);
        dsd_on = 1'b1;
        fork
            dsd_music(30000);
            begin
                env_check(480000, ms_b, nz_b);
                $display("PCM->DSD(flood) env: maxstep=%0d zeros=%0d", ms_b, nz_b);
                if (ms_b > 200000) begin
                    $display("FAIL: flood step too big (click)");
                    err = err + 1;
                end
                if (nz_b < 500) begin
                    $display("FAIL: flood no silent hold");
                    err = err + 1;
                end
            end
        join
        $display("flood rate_sel=%b in_idle=%b (info only)",
            top_i.u_det.sel, top_i.u_det.in_idle);

        // ---- B2. DSD DC in the same session (window refills with ones)
        // -> swap check through the top mux.
        fork
            dsd_dc(44000, 1'b1, 1'b0);
            begin repeat (700000) @(posedge mclk); end
        join
        sniff_mux(vc, acc_l, acc_r);
        $display("DSD-B: valids=%0d/512 meanL=%0d meanR=%0d", vc, $signed(acc_l), $signed(acc_r));
        if (vc != 4) begin $display("FAIL: DSD-B rate"); err = err + 1; end
        // Through-top swap (hardware 2026-09-09: transport carries RIGHT
        // on SDATA, LEFT on LRCLK): SDATA=1 must land R=+FS, LRCLK=0 -> L=-FS.
        if (!($signed(acc_l) < -8000000)) begin $display("FAIL: DSD-B L not -FS (swap?)"); err = err + 1; end
        if (!($signed(acc_r) > 8000000)) begin $display("FAIL: DSD-B R not +FS (swap?)"); err = err + 1; end
        else $display("DSD-B swap ok (SDATA=1 -> R+, LRCLK=0 -> L-)");

        // ---- C. back to PCM (envelope observed the same way) ----
        dsd_on = 1'b0;
        pcm_run = 1'b1;
        fork
            begin repeat (720000) @(posedge mclk); end
            begin
                env_check(700000, ms_c, nz_c);
                $display("DSD->PCM env: maxstep=%0d zeros=%0d", ms_c, nz_c);
                if (ms_c > 200000) begin
                    $display("FAIL: DSD->PCM step too big (click)");
                    err = err + 1;
                end
                if (nz_c < 500) begin
                    $display("FAIL: DSD->PCM no silent hold");
                    err = err + 1;
                end
            end
        join
        sniff_mux(vc, acc_l, acc_r);
        $display("PCM-C: valids=%0d/512 meanL=%0d meanR=%0d", vc, $signed(acc_l), $signed(acc_r));
        if (vc != 4) begin $display("FAIL: PCM-C rate (no resume)"); err = err + 1; end
        if (!($signed(acc_l) > 4000000)) begin $display("FAIL: PCM-C L not +0.5FS"); err = err + 1; end
        if (!($signed(acc_r) < -4000000)) begin $display("FAIL: PCM-C R not -0.5FS"); err = err + 1; end

        if (err == 0) $display("PASS tb_dsd_flood");
        else $display("FAIL tb_dsd_flood (%0d)", err);
        $finish;
    end

    // Watchdog (A ~550k + B1 480k + B2 704k + C 720k mclk + margin)
    initial begin
        repeat (12000000) @(posedge mclk);
        $display("WATCHDOG hang");
        $finish;
    end
endmodule
