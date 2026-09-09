`timescale 1ns/1ps
// Hostile DSD-transition bench (silicon reality, 2026-09-09):
// PCM 44.1 kHz (X4 engine path — the rate in the stuck LA capture) ->
// clock-stop gap -> async DSD episode (own crystal: bit clock drifts
// against MCLK, DATA1/DATA2 = LFSR garbage = 50%-density DSD silence
// on the shared pins) -> gap -> PCM resumes.
// tb_dsd_trans passes with clean cooperative clocks; this bench models
// what the Amanero actually does. FAIL = PCM does not resume (the
// silicon symptom: dither-flow zeros forever, reset is the only cure).
module tb_dsd_hostile;
    reg mclk = 1'b0;
    always #11.072 mclk = ~mclk;   // 45.1584 MHz

    reg i2s_bclk_in = 1'b0;
    reg i2s_lrck_in = 1'b0;
    reg i2s_sdata_in = 1'b0;
    reg dsd_on = 1'b0;
    reg pcm_run = 1'b0;
    reg dsd_run = 1'b0;

    // ---- PCM generator: 44.1 kHz X4 grid ----
    // LRCK = MCLK/1024, BCLK = MCLK/16, 64 BCLK/frame, 32-bit slots.
    // L = +0.25FS (0x20000000), R = -0.25FS (0xE0000000).
    reg [3:0] bdiv = 4'd0;
    reg [5:0] fpos = 6'd0;
    always @(negedge mclk) begin
        bdiv <= bdiv + 4'd1;
        if (pcm_run) begin
            if (bdiv == 4'd8) begin
                i2s_bclk_in <= 1'b1;
                fpos <= (fpos == 6'd63) ? 6'd0 : fpos + 6'd1;
            end else if (bdiv == 4'd0) begin
                i2s_bclk_in <= 1'b0;
                if (fpos == 6'd0) begin
                    i2s_lrck_in <= 1'b0;
                    i2s_sdata_in <= 1'b0;
                end else if (fpos == 6'd32) begin
                    i2s_lrck_in <= 1'b1;
                    i2s_sdata_in <= 1'b0;
                end else if (fpos >= 6'd1 && fpos <= 6'd31) begin
                    i2s_sdata_in <= (32'h20000000 >> (32 - fpos)) & 1'b1;
                end else begin
                    i2s_sdata_in <= (32'hE0000000 >> (64 - fpos)) & 1'b1;
                end
            end
        end
    end

    // ---- async DSD driver: own crystal (~DSD256 + drift, NOT mclk-locked)
    reg dsd_bclk = 1'b0;
    always #22.0 dsd_bclk = ~dsd_bclk;   // 44 ns period vs 44.29 ideal
    reg [15:0] lfsr = 16'hACE1;
    always @(posedge dsd_bclk or negedge dsd_bclk) begin
        if (dsd_run) begin
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            i2s_bclk_in <= dsd_bclk;
            i2s_sdata_in <= lfsr[15];
            i2s_lrck_in <= lfsr[7];
        end
    end

    top #(.DB_BITS(8)) top_i (
        .mclk_in(mclk),
        .i2s_bclk_in(i2s_bclk_in),
        .i2s_lrck_in(i2s_lrck_in),
        .i2s_sdata_in(i2s_sdata_in),
        .dsd_on(dsd_on),
        .i2s_bclk_out(),
        .i2s_lrck_out(),
        .i2s_sdata_out()
    );

    integer err = 0;

    // Sniff the post-fade dither input over a window: valid rate + means.
    task sniff_dith;
        input integer cycles;
        output integer valids;
        output signed [23:0] mean_l;
        output signed [23:0] mean_r;
        integer c;
        reg signed [31:0] sl, sr;
        begin
            valids = 0; sl = 0; sr = 0;
            for (c = 0; c < cycles; c = c + 1) begin
                @(posedge mclk); #1;
                if (top_i.u_dith.in_valid === 1'b1) begin
                    valids = valids + 1;
                    sl = sl + $signed(top_i.u_dith.in_l);
                    sr = sr + $signed(top_i.u_dith.in_r);
                end
            end
            mean_l = (valids > 0) ? sl / valids : 0;
            mean_r = (valids > 0) ? sr / valids : 0;
        end
    endtask

    // One-line state dump for the stuck hunt.
    task dump_state;
        begin
            $display("t=%0t sw=%0d mux=%0d db=%0d fg=%0d idle=%0b sel=%0b occu=%0d",
                $time, top_i.sw_state, top_i.mux_frozen,
                top_i.dsd_active_db, top_i.fgain,
                top_i.u_det.in_idle, top_i.u_det.sel, top_i.u_src.occu);
        end
    endtask

    // Trace: per-frame_tick + per-pair snapshot of the X4 engine feed.
    task trace_feed;
        input integer frames;
        integer c;
        begin
            for (c = 0; c < frames*256 + 128; c = c + 1) begin
                @(posedge mclk); #1;
                if (top_i.frame_tick === 1'b1) begin
                    $display("FR occu=%0d fw=%0b fr=%0b hold=%0d/%0d push=%0d/%0d eng=%0d/%0d st=%0d ph=%0d idle=%0b sel=%0b",
                        top_i.u_src.occu, top_i.u_src.fwptr, top_i.u_src.frptr,
                        $signed(top_i.u_src.hold_l), $signed(top_i.u_src.hold_r),
                        $signed(top_i.u_src.push_l), $signed(top_i.u_src.push_r),
                        $signed(top_i.u_src.eng_l), $signed(top_i.u_src.eng_r),
                        top_i.u_src.st, top_i.u_src.phase,
                        top_i.u_det.in_idle, top_i.u_det.sel);
                end
                if (top_i.u_rx.out_valid === 1'b1) begin
                    $display("PR rx=%0d/%0d", $signed(top_i.u_rx.out_l), $signed(top_i.u_rx.out_r));
                end
            end
        end
    endtask

    integer vc;
    reg signed [23:0] ml, mr;
    integer wc;

    initial begin
        repeat (200) @(posedge mclk);

        // ---- A. PCM baseline ----
        pcm_run = 1'b1;
        wc = 0;
        while (top_i.sw_state !== 2'd0 && wc < 3000000) begin
            @(posedge mclk); wc = wc + 1;
        end
        if (top_i.sw_state !== 2'd0) begin
            $display("FAIL: no STEADY-A"); err = err + 1;
        end
        repeat (40*1024) @(posedge mclk); // X4 lock + engine flush
        dump_state();
        sniff_dith(4096, vc, ml, mr);
        $display("PCM-A: valids=%0d/4096 meanL=%0d meanR=%0d", vc, $signed(ml), $signed(mr));
        if (vc < 8) begin $display("FAIL: PCM-A no valids"); err = err + 1; end
        if (!($signed(ml) > 1500000)) begin $display("FAIL: PCM-A L not +0.25FS"); err = err + 1; end
        if (!($signed(mr) < -1500000)) begin $display("FAIL: PCM-A R not -0.25FS"); err = err + 1; end

        // ---- B. gap + hostile DSD (~9 ms, ~200k bits) ----
        pcm_run = 1'b0;
        repeat (90000) @(posedge mclk);       // ~2 ms clock-stop gap
        dsd_on = 1'b1;
        repeat (10) @(posedge mclk);
        dsd_run = 1'b1;
        repeat (400000) @(posedge mclk);      // ~9 ms async DSD garbage
        dsd_run = 1'b0;
        dump_state();

        // ---- C. gap + PCM resumes ----
        repeat (90000) @(posedge mclk);       // ~2 ms gap
        dsd_on = 1'b0;
        repeat (10) @(posedge mclk);
        pcm_run = 1'b1;

        // ---- D. observe: dump every ~2.3 ms, then grade ----
        repeat (12) begin
            repeat (100000) @(posedge mclk);
            dump_state();
        end
        if (top_i.sw_state !== 2'd0) begin
            $display("FAIL: PCM-C machine never STEADY (stuck, silicon symptom)");
            err = err + 1;
        end
        if (top_i.mux_frozen !== 1'b0) begin
            $display("FAIL: PCM-C mux stuck on DSD"); err = err + 1;
        end
        sniff_dith(4096, vc, ml, mr);
        $display("PCM-C: valids=%0d/4096 meanL=%0d meanR=%0d", vc, $signed(ml), $signed(mr));
        if (vc < 8) begin $display("FAIL: PCM-C no valids (starved)"); err = err + 1; end
        if (!($signed(ml) > 1500000)) begin $display("FAIL: PCM-C L not +0.25FS (stuck zeros?)"); err = err + 1; end
        if (!($signed(mr) < -1500000)) begin $display("FAIL: PCM-C R not -0.25FS (stuck zeros?)"); err = err + 1; end
        if (err != 0) begin
            $display("--- feed trace (12 frames) ---");
            trace_feed(12);
        end

        if (err == 0) $display("PASS tb_dsd_hostile");
        else $display("FAIL tb_dsd_hostile (%0d)", err);
        $finish;
    end

    initial begin
        repeat (30000000) @(posedge mclk);
        $display("WATCHDOG hang"); dump_state(); $finish;
    end
endmodule
