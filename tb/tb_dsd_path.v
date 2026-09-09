`timescale 1ns/1ps
// DSD path bench (Amanero-style, DATA2 on the LRCLK pin).
// Unit: dsd_to_pcm -> dsd_pcm_decim2 -> dsd_round_s32_s24 (the same
// rounding module top.v uses):
//   - dsd_on=0 gates sample_valid even with live DSD pins (PCM immunity)
//   - DC ones / zeros at DSD64/128/256/512 -> exact S24 rails, never wrap
//   - L=1/R=0 proves DATA2 comes from the LRCLK pin (dsd_data2_alt=0)
//   - alternating-bit silence -> small residual (bound measured below)
// Top smoke: full top.v, dsd_on=1, DSD64 DC ones -> the muxed S24 stream
// is +FS and the emitted I2S words sit at the rail (dither +- few LSB).
// NOTE: integer regs/arrays only in checks — icarus 11.0 silently drops
// conditional stores to real-array elements (see tb_interp step_check).
module tb_dsd_path;
    reg clk = 1'b0;
    always #11.072 clk = ~clk;   // 45.1584 MHz
    reg rst_n = 1'b0;

    integer err = 0;

    // ---------------- unit chain ----------------
    reg dsd_on_u = 1'b0, bclk_u = 1'b0, d1_u = 1'b0, lr_u = 1'b0;
    wire uv;
    wire signed [31:0] ul, ur;
    dsd_to_pcm u1 (
        .mclk(clk), .rst_n(rst_n), .dsd_on(dsd_on_u),
        .dsd_clk_in(bclk_u), .dsd_data1(d1_u), .dsd_data2_alt(1'b0),
        .lrck_in(lr_u), .sample_valid(uv), .sample_l(ul), .sample_r(ur)
    );
    wire dv;
    wire signed [31:0] dl, dr;
    dsd_pcm_decim2 u2 (
        .clk(clk), .rst_n(rst_n), .in_valid(uv),
        .in_l(ul), .in_r(ur), .out_valid(dv), .out_l(dl), .out_r(dr)
    );
    wire signed [23:0] ol24, or24;
    dsd_round_s32_s24 rl (.in_s32(dl), .out_s24(ol24));
    dsd_round_s32_s24 rr (.in_s32(dr), .out_s24(or24));

    reg signed [23:0] respL [0:31];
    reg signed [23:0] respR [0:31];

    // Emit nbits CONSTANT DSD bits, pb mclk per bit. Data is set up a
    // full low half-period before the rise the DUT syncs to.
    // NOTE: no if/else on the data select here — icarus 11.0 loses
    // task else-branch stores; DC and alternating drive are split into
    // two branch-free tasks instead (verified in /tmp/dbgdsd*).
    task dsd_dc;
        input integer nbits;
        input integer pb;
        input b1;
        input b2;
        integer k, h;
        begin
            for (k = 0; k < nbits; k = k + 1) begin
                d1_u = b1; lr_u = b2;
                bclk_u = 1'b0;
                for (h = 0; h < (pb >> 1); h = h + 1) @(posedge clk);
                bclk_u = 1'b1;
                for (h = 0; h < (pb >> 1); h = h + 1) @(posedge clk);
            end
            bclk_u = 1'b0;
        end
    endtask

    // Alternating-bit drive (DSD silence pattern), branch-free.
    task dsd_alt;
        input integer nbits;
        input integer pb;
        integer k, h;
        begin
            for (k = 0; k < nbits; k = k + 1) begin
                d1_u = (k % 2); lr_u = (k % 2);
                bclk_u = 1'b0;
                for (h = 0; h < (pb >> 1); h = h + 1) @(posedge clk);
                bclk_u = 1'b1;
                for (h = 0; h < (pb >> 1); h = h + 1) @(posedge clk);
            end
            bclk_u = 1'b0;
        end
    endtask

    // Collect n stage-2 outputs, skipping the first `skip` (settle).
    task collect24;
        input integer skip;
        input integer n;
        integer got, sk;
        begin
            got = 0; sk = 0;
            while (got < n) begin
                @(posedge clk); #1;
                if (dv === 1'b1) begin
                    if (sk < skip) sk = sk + 1;
                    else begin
                        respL[got] = ol24; respR[got] = or24;
                        got = got + 1;
                    end
                end
            end
        end
    endtask

    // DC at one DSD rate: expect exact rails on all collected frames.
    task dc_rate;
        input integer pb;
        input b;
        input [23:0] rail;
        integer k, nbits;
        begin
            dsd_on_u = 1'b1;
            nbits = 4400 * 16 / pb;
            fork
                dsd_dc(nbits, pb, b, b);
                collect24(250, 12);
            join
            for (k = 0; k < 12; k = k + 1) begin
                if (respL[k] !== rail) begin
                    $display("FAIL: DC pb=%0d b=%0d L f=%0d got %0d want %0d",
                        pb, b, k, $signed(respL[k]), $signed(rail));
                    err = err + 1;
                end
                if (respR[k] !== rail) begin
                    $display("FAIL: DC pb=%0d b=%0d R f=%0d got %0d want %0d",
                        pb, b, k, $signed(respR[k]), $signed(rail));
                    err = err + 1;
                end
            end
            $display("DSD DC pb=%0d b=%0d ok (rail %0d)", pb, b, $signed(rail));
            dsd_on_u = 1'b0;
            repeat (100) @(posedge clk);
        end
    endtask

    // ---------------- top smoke ----------------
    reg top_bclk = 1'b0, top_lr = 1'b0, top_sd = 1'b0;
    reg top_dsd_on = 1'b0;
    top #(.DB_BITS(6)) top_i (
        .mclk_in(clk),
        .i2s_bclk_in(top_bclk),
        .i2s_lrck_in(top_lr),
        .i2s_sdata_in(top_sd),
        .dsd_on(top_dsd_on),
        .dsd_data2_in(1'b0),
        .nos_bypass(1'b0),
        .i2s_bclk_out(tx_bclk),
        .i2s_lrck_out(tx_lrck),
        .i2s_sdata_out(tx_sdata)
    );
    wire tx_bclk, tx_lrck, tx_sdata;

    // I2S word monitor (Philips straddle, tb_top recipe): shift every
    // BCLK rise; at LRCK rise the window holds [stale, L15..L1] while
    // sdata already carries L_LSB (same div0 edge) — the word is exact.
    reg [15:0] txwin = 16'd0;
    always @(posedge tx_bclk) txwin <= {txwin[14:0], tx_sdata};

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        begin : main
            integer k, nbits, maxabs, a;

            // ---- 1. gate: live pins, dsd_on=0 -> no valid ever ----
            fork
                dsd_dc(600, 16, 1'b1, 1'b1);
                begin
                    repeat (3000) @(posedge clk);
                    if (uv === 1'b1) begin
                        $display("FAIL: sample_valid with dsd_on=0");
                        err = err + 1;
                    end else $display("DSD gate ok (valid held low)");
                end
            join

            // ---- 2. DC rails at all four rates ----
            dc_rate(16, 1'b1, 24'h7FFFFF);   // DSD64  +FS
            dc_rate(16, 1'b0, 24'h800000);   // DSD64  -FS
            dc_rate(8, 1'b1, 24'h7FFFFF);    // DSD128 +FS
            dc_rate(8, 1'b0, 24'h800000);    // DSD128 -FS
            dc_rate(4, 1'b1, 24'h7FFFFF);    // DSD256 +FS
            dc_rate(4, 1'b0, 24'h800000);    // DSD256 -FS
            dc_rate(2, 1'b1, 24'h7FFFFF);    // DSD512 +FS
            dc_rate(2, 1'b0, 24'h800000);    // DSD512 -FS

            // ---- 3. routing (UNIT level, no top swap): SDATA=1 -> stage-L=+FS,
            // LRCLK=0 -> stage-R=-FS. The L/R swap to match PCM lives in
            // top.v and is locked by tb_dsd_trans, not here. --
            dsd_on_u = 1'b1;
            nbits = 4400;
            fork
                dsd_dc(nbits, 16, 1'b1, 1'b0);
                collect24(250, 4);
            join
            for (k = 0; k < 4; k = k + 1) begin
                if (respL[k] !== 24'h7FFFFF || respR[k] !== 24'h800000) begin
                    $display("FAIL: routing f=%0d L=%0d R=%0d",
                        k, $signed(respL[k]), $signed(respR[k]));
                    err = err + 1;
                end
            end
            $display("DSD routing ok (L=+FS from SDATA, R=-FS from LRCLK)");
            dsd_on_u = 1'b0;
            repeat (100) @(posedge clk);

            // ---- 4. alternating-bit silence: residual must be small --
            dsd_on_u = 1'b1;
            fork
                dsd_alt(nbits, 16);
                collect24(250, 12);
            join
            maxabs = 0;
            for (k = 0; k < 12; k = k + 1) begin
                a = ($signed(respL[k]) < 0) ? -$signed(respL[k])
                                            : $signed(respL[k]);
                if (a > maxabs) maxabs = a;
                a = ($signed(respR[k]) < 0) ? -$signed(respR[k])
                                            : $signed(respR[k]);
                if (a > maxabs) maxabs = a;
            end
            $display("DSD silence residual maxabs=%0d", maxabs);
            if (maxabs > 4096) begin
                $display("FAIL: silence residual too big (%0d)", maxabs);
                err = err + 1;
            end
            dsd_on_u = 1'b0;
            repeat (100) @(posedge clk);

            // ---- 5. top smoke: DSD64 DC ones through mux+dither+TX --
            // 50000 bits = 800k mclk: covers fade-out + hold + fade-in
            // + acquisition + both settles.
            top_dsd_on = 1'b1;
            fork
                begin
                    integer kk, hh;
                    for (kk = 0; kk < 50000; kk = kk + 1) begin
                        top_sd = 1'b1; top_lr = 1'b1;
                        top_bclk = 1'b0;
                        for (hh = 0; hh < 8; hh = hh + 1) @(posedge clk);
                        top_bclk = 1'b1;
                        for (hh = 0; hh < 8; hh = hh + 1) @(posedge clk);
                    end
                    top_bclk = 1'b0;
                end
                begin
                    // wait ~700k mclk (fades + hold + acq + settles),
                    // then check
                    repeat (700000) @(posedge clk);
                    begin : vchk
                        integer vc, cc;
                        vc = 0;
                        for (cc = 0; cc < 512; cc = cc + 1) begin
                            @(posedge clk); #1;
                            if (top_i.src_mux_valid === 1'b1) vc = vc + 1;
                        end
                        // exactly 1 pair per 256-mclk frame
                        if (vc != 2) begin
                            $display("FAIL: mux valid %0d/512 (want 2)", vc);
                            err = err + 1;
                        end else $display("top mux rate ok (1/256 mclk)");
                    end
                    if (top_i.src_mux_l !== 24'h7FFFFF) begin
                        $display("FAIL: top mux L=%0d (want +FS)",
                            $signed(top_i.src_mux_l));
                        err = err + 1;
                    end else $display("top mux L=+FS ok");
                    begin : txw
                        reg signed [15:0] wl0, wl1, wl2;
                        @(posedge tx_lrck); #1;
                        wl0 = $signed({txwin[14:0], tx_sdata});
                        @(posedge tx_lrck); #1;
                        wl1 = $signed({txwin[14:0], tx_sdata});
                        @(posedge tx_lrck); #1;
                        wl2 = $signed({txwin[14:0], tx_sdata});
                        $display("top TX words: %0d %0d %0d",
                            $signed(wl0), $signed(wl1), $signed(wl2));
                        if ($signed(wl0) < 32760 || $signed(wl1) < 32760 ||
                            $signed(wl2) < 32760) begin
                            $display("FAIL: top TX words not at rail");
                            err = err + 1;
                        end else $display("top TX at rail ok");
                    end
                end
            join
        end

        if (err == 0) $display("PASS tb_dsd_path");
        else $display("FAIL tb_dsd_path (%0d)", err);
        $finish;
    end
endmodule
