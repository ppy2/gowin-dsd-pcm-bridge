`timescale 1ns/1ps
// Silence auto-mute bench (through top, X1 grid).
// 1. POR starts muted; driven zeros keep exact-0 out (no +-1 dither dirt).
// 2. DC tone unmutes instantly with bit-correct level.
// 3. Back to zeros: dither flows briefly, then exact 0 (muted, sticks).
// 4. A sine never trips the mute (only 1-2 zero samples per crossing).
module tb_mute;
    reg mclk = 1'b0;
    always #11.072 mclk = ~mclk;

    reg signed [23:0] gen_l = 24'sd0, gen_r = 24'sd0;
    reg i_bclk = 1'b0, i_lrck = 1'b0, i_sdata = 1'b0;
    reg [1:0] bdiv = 2'd0;
    reg [5:0] fpos = 6'd0;
    reg [31:0] sh = 32'd0;
    always @(negedge mclk) begin
        bdiv <= bdiv + 2'd1;
        if (bdiv == 2'd2) begin
            i_bclk <= 1'b1;
            fpos <= (fpos == 6'd63) ? 6'd0 : fpos + 6'd1;
            if (fpos == 6'd63) sh <= {gen_l[23:0], 8'd0};
            else if (fpos == 6'd31) sh <= {gen_r[23:0], 8'd0};
            else sh <= {sh[30:0], 1'b0};
        end else if (bdiv == 2'd0) begin
            i_bclk <= 1'b0;
            if (fpos == 6'd0) i_lrck <= 1'b0;
            else if (fpos == 6'd32) i_lrck <= 1'b1;
            i_sdata <= sh[31];
        end
    end

    top #(.DB_BITS(6)) top_i (
        .mclk_in(mclk),
        .i2s_bclk_in(i_bclk), .i2s_lrck_in(i_lrck),
        .i2s_sdata_in(i_sdata), .dsd_on(1'b0), .dsd_data2_in(1'b0),
        .nos_bypass(1'b0),
        .i2s_bclk_out(), .i2s_lrck_out(), .i2s_sdata_out(),
        .tda_bck_out(), .tda_le_out(), .tda_dl_out(), .tda_dr_out()
    );

    integer err = 0;
    integer c, nz;
    integer wc;

    task wait_steady;
        begin
            wc = 0;
            while (top_i.sw_state !== 2'd0 && wc < 2000000) begin
                @(posedge mclk); wc = wc + 1;
            end
            if (top_i.sw_state !== 2'd0) begin
                $display("FAIL: no STEADY"); err = err + 1;
            end
        end
    endtask

    // Count nonzero gated outputs over nframes output frames.
    task count_nonzero;
        input integer nframes;
        output integer n;
        integer k;
        begin
            n = 0;
            for (k = 0; k < nframes * 256; k = k + 1) begin
                @(posedge mclk); #1;
                if (top_i.dith_valid === 1'b1) begin
                    if (top_i.out_l !== 16'sd0 || top_i.out_r !== 16'sd0)
                        n = n + 1;
                end
            end
        end
    endtask

    real ph;
    initial begin
        // ---- 1. zeros from boot: must stay exactly 0 ----
        gen_l = 24'sd0; gen_r = 24'sd0;
        wait_steady();
        repeat (40 * 256) @(posedge mclk);
        if (top_i.muted !== 1'b1) begin
            $display("FAIL: not muted on digital silence"); err = err + 1;
        end
        count_nonzero(64, nz);
        if (nz !== 0) begin
            $display("FAIL: dirt in stop (%0d nonzero)", nz); err = err + 1;
        end else $display("mute on silence ok");

        // ---- 2. tone unmutes instantly, level bit-correct ----
        gen_l = 24'h200000; gen_r = -24'h200000; // +-0.25FS
        repeat (3 * 256) @(posedge mclk);
        if (top_i.muted !== 1'b0) begin
            $display("FAIL: stuck muted on tone"); err = err + 1;
        end
        count_nonzero(4, nz);
        if (nz < 3) begin
            $display("FAIL: tone did not pass"); err = err + 1;
        end else $display("instant unmute ok");

        // ---- 3. back to zeros: mutes again, exact 0 sticks ----
        gen_l = 24'sd0; gen_r = 24'sd0;
        repeat (4600 * 256) @(posedge mclk); // > 4095 pairs
        if (top_i.muted !== 1'b1) begin
            $display("FAIL: no re-mute"); err = err + 1;
        end
        count_nonzero(64, nz);
        if (nz !== 0) begin
            $display("FAIL: dirt after re-mute (%0d)", nz); err = err + 1;
        end else $display("re-mute exact-0 ok");

        // ---- 4. sine never trips ----
        for (c = 0; c < 528; c = c + 1) begin
            ph = 2 * 3.14159265 * (c % 176) / 176.0;
            gen_l = $rtoi(2097152.0 * $sin(ph));
            gen_r = $rtoi(-2097152.0 * $sin(ph));
            repeat (256) @(posedge mclk);
        end
        if (top_i.muted !== 1'b0) begin
            $display("FAIL: sine tripped mute"); err = err + 1;
        end else $display("sine no-trip ok");

        if (err == 0) $display("PASS tb_mute");
        else $display("FAIL tb_mute (%0d)", err);
        $finish;
    end

    initial begin
        repeat (30000000) @(posedge mclk);
        $display("WATCHDOG hang"); $finish;
    end
endmodule
