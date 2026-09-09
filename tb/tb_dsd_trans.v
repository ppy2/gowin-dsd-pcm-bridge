`timescale 1ns/1ps
// Transition bench: PCM -> DSD -> PCM through the full top.v.
// Proves (1) which physical pin ends up on which DSD output channel,
// (2) that the PCM path resumes after dsd_on falls (valid + data).
module tb_dsd_trans;
    reg mclk = 1'b0;
    always #11.072 mclk = ~mclk;   // 45.1584 MHz

    // ---- shared transport pins (I2S in PCM, DSD bitstream in DSD) ----
    reg i2s_bclk_in = 1'b0;
    reg i2s_lrck_in = 1'b0;
    reg i2s_sdata_in = 1'b0;
    reg dsd_on = 1'b0;
    reg pcm_run = 1'b0;   // 1 = bit-bang PCM I2S frames

    // PCM generator: X1 grid (LRCK=MCLK/256), 32-bit slots, BCLK=MCLK/4.
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

    top top_i (
        .mclk_in(mclk),
        .i2s_bclk_in(i2s_bclk_in),
        .i2s_lrck_in(i2s_lrck_in),
        .i2s_sdata_in(i2s_sdata_in),
        .dsd_on(dsd_on),
        .i2s_bclk_out(),
        .i2s_lrck_out(),
        .i2s_sdata_out()
    );

    // DSD DC drive on the shared pins (DSD64: 16 mclk per bit).
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
    reg signed [23:0] acc_l, acc_r;
    integer n;

    // Count src_mux_valid over 512 mclk (expect 2 = 1 pair/256) and
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

    initial begin
        repeat (200) @(posedge mclk); // POR + settle

        // ---- A. PCM baseline ----
        pcm_run = 1'b1;
        repeat (400*256) @(posedge mclk); // ~400 frames lock + flush
        sniff_mux(vc, acc_l, acc_r);
        $display("PCM-A: valids=%0d/512 meanL=%0d meanR=%0d", vc, $signed(acc_l), $signed(acc_r));
        if (vc != 2) begin $display("FAIL: PCM-A rate"); err = err + 1; end
        if (!($signed(acc_l) > 4000000)) begin $display("FAIL: PCM-A L not +0.5FS"); err = err + 1; end
        if (!($signed(acc_r) < -4000000)) begin $display("FAIL: PCM-A R not -0.5FS"); err = err + 1; end

        // ---- B. DSD: SDATA=1, LRCLK=0 ----
        // 24000 bits = 384k mclk: covers the 262k switch blank + the
        // ~1 ms acquisition + FIR window fill.
        pcm_run = 1'b0;
        repeat (10) @(posedge mclk);
        dsd_on = 1'b1;
        // Switch blank must hold dither input at zero while settling.
        begin : blankchk_b
            integer c, nz;
            nz = 0;
            for (c = 0; c < 20000; c = c + 1) begin
                @(posedge mclk); #1;
                if (top_i.u_dith.in_valid === 1'b1) begin
                    if ($signed(top_i.u_dith.in_l) !== 0 ||
                        $signed(top_i.u_dith.in_r) !== 0) nz = nz + 1;
                end
            end
            if (nz != 0) begin $display("FAIL: PCM->DSD blank not silent (%0d)", nz); err = err + 1; end
            else $display("PCM->DSD blank ok (zeros during settle)");
        end
        dsd_dc(24000, 1'b1, 1'b0);
        sniff_mux(vc, acc_l, acc_r);
        $display("DSD-B: valids=%0d/512 meanL=%0d meanR=%0d", vc, $signed(acc_l), $signed(acc_r));
        if (vc != 2) begin $display("FAIL: DSD-B rate"); err = err + 1; end
        // Through-top swap (hardware 2026-09-09: transport carries RIGHT
        // on SDATA, LEFT on LRCLK): SDATA=1 must land R=+FS, LRCLK=0 -> L=-FS.
        if (!($signed(acc_l) < -8000000)) begin $display("FAIL: DSD-B L not -FS (swap?)"); err = err + 1; end
        if (!($signed(acc_r) > 8000000)) begin $display("FAIL: DSD-B R not +FS (swap?)"); err = err + 1; end
        else $display("DSD-B swap ok (SDATA=1 -> R+, LRCLK=0 -> L-)");

        // ---- C. back to PCM ----
        // 1600 frames = 410k mclk: covers the 262k switch blank + the
        // rate relock + interp history flush.
        dsd_on = 1'b0;
        // Same blank check on the way back (mechanism is shared).
        begin : blankchk_c
            integer c, nz;
            nz = 0;
            for (c = 0; c < 20000; c = c + 1) begin
                @(posedge mclk); #1;
                if (top_i.u_dith.in_valid === 1'b1) begin
                    if ($signed(top_i.u_dith.in_l) !== 0 ||
                        $signed(top_i.u_dith.in_r) !== 0) nz = nz + 1;
                end
            end
            if (nz != 0) begin $display("FAIL: DSD->PCM blank not silent (%0d)", nz); err = err + 1; end
            else $display("DSD->PCM blank ok (zeros during settle)");
        end
        repeat (10) @(posedge mclk);
        pcm_run = 1'b1;
        repeat (1600*256) @(posedge mclk);
        sniff_mux(vc, acc_l, acc_r);
        $display("PCM-C: valids=%0d/512 meanL=%0d meanR=%0d", vc, $signed(acc_l), $signed(acc_r));
        if (vc != 2) begin $display("FAIL: PCM-C rate (no resume)"); err = err + 1; end
        if (!($signed(acc_l) > 4000000)) begin $display("FAIL: PCM-C L not +0.5FS"); err = err + 1; end
        if (!($signed(acc_r) < -4000000)) begin $display("FAIL: PCM-C R not -0.5FS"); err = err + 1; end

        if (err == 0) $display("PASS tb_dsd_trans");
        else $display("FAIL tb_dsd_trans (%0d)", err);
        $finish;
    end

    // Watchdog
    initial begin
        repeat (3000000) @(posedge mclk);
        $display("WATCHDOG hang");
        $finish;
    end
endmodule
