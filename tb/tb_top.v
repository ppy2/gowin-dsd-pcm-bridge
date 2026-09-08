// End-to-end bench: 192 kHz Philips I2S stimulus -> top -> decoded I2S.
// Stage-1 SRC in X1 (input = output grid) + TPDF dither, NO filter:
// all tones pass at the headroom level. Race-free: input generator on
// negedge mclk; output decoder samples sdata/lrck at BCLK rises.
// Gates: Philips framing, flat 1/8/30 kHz (~0 dB, L+R),
// L/R inter-channel alignment on dual-mono (< 2 LSB RMS),
// rate detector locked at X1.
`timescale 1ns/1ps
module tb_top;
    reg mclk = 1'b0;
    always #10.1725 mclk = ~mclk; // 49.152 MHz

    // ---- input I2S generator (MCLK-derived, Philips, 32-bit slots) ----
    // 192 kHz native: BCLK = MCLK/4 = 12.288 MHz, 64 BCLK per frame,
    // LRCK = MCLK/256. 24-bit words MSB-aligned + 8 zeros.
    reg [1:0] bdiv = 2'd0;
    reg [5:0] fpos = 6'd0; // 0..63 frame position, advances on BCLK rise
    reg i2s_bclk_in = 1'b0;
    reg i2s_lrck_in = 1'b0;
    reg i2s_sdata_in = 1'b0;
    reg [31:0] slotL = 32'd0;
    reg [31:0] slotR = 32'd0;
    real phase = 0.0;
    real cur_freq = 1000.0;
    real AMP = 7000000.0;

    always @(negedge mclk) begin
        bdiv <= bdiv + 2'd1;
        if (bdiv == 2'd2) begin
            i2s_bclk_in <= 1'b1;
            fpos <= (fpos == 6'd63) ? 6'd0 : fpos + 6'd1;
        end else if (bdiv == 2'd0) begin
            i2s_bclk_in <= 1'b0;
            // falling edge: LRCK + data (Philips, 1-BCLK delay)
            if (fpos == 6'd0) begin
                i2s_lrck_in <= 1'b0;
                i2s_sdata_in <= 1'b0; // holdover (prev LSB = 0)
                phase <= phase + 2.0 * 3.14159265358979 * cur_freq / 192000.0;
                slotL <= {$rtoi(AMP * $sin(phase)), 8'd0};
                slotR <= {$rtoi(AMP * $sin(phase)), 8'd0};
            end else if (fpos == 6'd32) begin
                i2s_lrck_in <= 1'b1;
                i2s_sdata_in <= 1'b0;
            end else if (fpos >= 6'd1 && fpos <= 6'd31) begin
                i2s_sdata_in <= slotL[32 - fpos];
            end else begin
                i2s_sdata_in <= slotR[64 - fpos];
            end
        end
    end

    wire i2s_bclk_out;
    wire i2s_lrck_out;
    wire i2s_sdata_out;

    top dut (
        .mclk_in(mclk),
        .i2s_bclk_in(i2s_bclk_in),
        .i2s_lrck_in(i2s_lrck_in),
        .i2s_sdata_in(i2s_sdata_in),
        .i2s_bclk_out(i2s_bclk_out),
        .i2s_lrck_out(i2s_lrck_out),
        .i2s_sdata_out(i2s_sdata_out)
    );

    // ---- behavioral output decoder + peak meter (one block, no races) ----
    // 16-bit slots: shift EVERY rise; an LRCK edge FRAMES the window.
    // At the edge-detect rise the 16-bit window holds exactly the ended
    // slot; the word is {window[14:0], sdata} (Philips straddle).
    reg lrck_q = 1'b0;
    reg [15:0] dshift = 16'd0;
    reg running = 1'b0;
    integer skip_cnt = 0;
    integer run_cnt = 0;
    integer run_total = 0;
    real sum2L = 0.0;
    real sum2R = 0.0;
    real diff2 = 0.0;
    integer npair = 0;
    integer fr_seen = 0;
    reg signed [31:0] wfull;
    reg signed [31:0] wL;
    reg [15:0] wraw;

    always @(posedge i2s_bclk_out) begin
        if (i2s_lrck_out != lrck_q) begin
            lrck_q <= i2s_lrck_out;
            fr_seen = fr_seen + 1;
            wraw = {dshift[14:0], i2s_sdata_out};
            wfull = {{16{wraw[15]}}, wraw};
            // ended slot level: 0->1 ends L, 1->0 ends R. RMS over integer
            // periods is sampling-phase invariant (peak is not). L/R are
            // captured every frame; diff2 catches any inter-channel shift:
            // one slipped 192k sample on a 1 kHz mono tone already moves
            // L-R by ~1000 LSB (gate is 2).
            if (i2s_lrck_out == 1'b1 && fr_seen > 4 && running) begin
                if (skip_cnt > 0)
                    skip_cnt = skip_cnt - 1;
                else if (run_cnt > 0) begin
                    run_cnt = run_cnt - 1;
                    wL = wfull;
                    sum2L = sum2L + $itor(wfull) * $itor(wfull);
                end
            end
            if (i2s_lrck_out == 1'b0 && fr_seen > 4 && running) begin
                if (skip_cnt == 0 && run_cnt > 0) begin
                    sum2R = sum2R + $itor(wfull) * $itor(wfull);
                    diff2 = diff2 + $itor(wL - wfull) * $itor(wL - wfull);
                    npair = npair + 1;
                end
            end
        end
        dshift <= {dshift[14:0], i2s_sdata_out};
    end

    integer err = 0;
    real ratioL, ratioR, align;
    integer fr_cnt = 0;

    // progress: count TX frames independently of the decoder
    always @(posedge i2s_bclk_out) begin
        if (i2s_lrck_out != lrck_q) fr_cnt = fr_cnt + 1;
    end

    task run_freq;
        input real freq;
        input integer skip_frames;
        input integer run_frames;
        begin
            @(negedge mclk);
            running = 1'b0;
            cur_freq = freq;
            sum2L = 0.0;
            sum2R = 0.0;
            diff2 = 0.0;
            npair = 0;
            skip_cnt = skip_frames;
            run_cnt = run_frames;
            run_total = run_frames;
            running = 1'b1;
            while (run_cnt > 0) @(posedge mclk);
            @(negedge mclk);
            running = 1'b0;
        end
    endtask

    // Philips framing probe: 2nd BCLK rise after an LRCK edge must carry
    // the MSB of the transmitted word.
    task check_framing;
        begin
            @(negedge i2s_lrck_out); // L slot start
            @(posedge i2s_bclk_out); // rise 1: delay bit
            @(posedge i2s_bclk_out); // rise 2: MSB
            #1;
            if (i2s_sdata_out !== dut.u_tx.hold_l[15]) begin
                $display("FAIL: L MSB framing (sdata=%b expect=%b)",
                    i2s_sdata_out, dut.u_tx.hold_l[15]);
                err = err + 1;
            end
            @(posedge i2s_lrck_out); // R slot start
            @(posedge i2s_bclk_out);
            @(posedge i2s_bclk_out);
            #1;
            if (i2s_sdata_out !== dut.u_tx.hold_r[15]) begin
                $display("FAIL: R MSB framing (sdata=%b expect=%b)",
                    i2s_sdata_out, dut.u_tx.hold_r[15]);
                err = err + 1;
            end
            $display("framing check done");
        end
    endtask

    // Global watchdog as a separate initial block (no fork/join_none):
    // if the main thread hangs, abort with a state dump.
    initial begin
        repeat (1024 * 2800) @(posedge mclk);
        $display("WATCHDOG: hang fr_cnt=%0d running=%b skip=%0d run=%0d fr_seen=%0d",
            fr_cnt, running, skip_cnt, run_cnt, fr_seen);
        $finish;
    end

    initial begin
        repeat (200) @(posedge mclk); // POR + settle
        repeat (1024 * 300) @(posedge mclk); // grids lock, filter flush

        check_framing;

        run_freq(1000.0, 1200, 960);
        // 16-bit out: nominal peak is AMP/256; headroom x8191/8192 (-0.001 dB)
        // is absorbed in the gate. Stimulus is dual-mono, so L/R must match
        // within dither (±1 LSB each): any real inter-channel shift fails.
        ratioL = $sqrt(sum2L / run_total) * 1.41421356237310 / (AMP / 256.0);
        ratioR = $sqrt(sum2R / npair) * 1.41421356237310 / (AMP / 256.0);
        align = (npair > 0) ? $sqrt(diff2 / npair) : 1.0e9;
        $display("1kHz L/R out/in RMS = %.5f / %.5f (expect 0.99988)", ratioL, ratioR);
        $display("1kHz L-R RMS = %.3f LSB over %0d pairs (gate < 2)", align, npair);
        if (ratioL < 0.994 || ratioL > 1.005) begin
            $display("FAIL: 1kHz L passband"); err = err + 1;
        end
        if (ratioR < 0.994 || ratioR > 1.005) begin
            $display("FAIL: 1kHz R passband"); err = err + 1;
        end
        if (npair < run_total - 2 || align > 2.0) begin
            $display("FAIL: L/R inter-channel alignment"); err = err + 1;
        end

        run_freq(8000.0, 480, 480);
        ratioL = $sqrt(sum2L / run_total) * 1.41421356237310 / (AMP / 256.0);
        $display("8kHz out/in RMS ratio = %.5f (expect 0.99988, no filter)", ratioL);
        if (ratioL < 0.994 || ratioL > 1.005) begin
            $display("FAIL: 8kHz passthrough"); err = err + 1;
        end

        run_freq(30000.0, 512, 512);
        ratioL = $sqrt(sum2L / run_total) * 1.41421356237310 / (AMP / 256.0);
        $display("30kHz out/in RMS ratio = %.5f (expect 0.99988, no filter)", ratioL);
        if (ratioL < 0.994 || ratioL > 1.005) begin
            $display("FAIL: 30kHz passthrough"); err = err + 1;
        end

        if (dut.u_det.sel !== 2'b01) begin
            $display("FAIL: detector not X1 (sel=%b)", dut.u_det.sel);
            err = err + 1;
        end else begin
            $display("detector X1 ok");
        end

        if (err == 0) $display("PASS tb_top");
        else $display("FAIL tb_top (%0d)", err);
        $finish;
    end
endmodule
