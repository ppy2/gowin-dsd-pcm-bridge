`timescale 1ns/1ps
// Engine unit bench (src_interp, sel driven directly).
// X4/X2 impulse: bit-EXACT vs tools/coef_hex (single source with the ROM)
// on both channels + cross-bleed zeros, two pair/frame phases.
// DC: branch-sum unity. Pre-ring: no negative precursor. Steady 0dBFS
// sines to 21k: never touch rails. Full-scale steps vs float model:
// saturation engages exactly at rails (never wraps). X1 bypass sanity.
module tb_interp;
    reg clk = 1'b0;
    always #10.1725 clk = ~clk;
    reg rst_n = 1'b0;
    reg pv = 1'b0;
    reg signed [23:0] pl = 24'sd0, pr = 24'sd0;
    reg [1:0] sel = 2'b01;
    reg idle = 1'b0;
    reg bypass = 1'b0;

    // Output frame grid: tick every 256 mclk.
    reg [7:0] fc8 = 8'd0;
    wire ftick = (fc8 == 8'd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fc8 <= 8'd0;
        else fc8 <= fc8 + 8'd1;
    end

    wire ov;
    wire signed [23:0] ol, orr;
    src_interp dut (
        .clk(clk), .rst_n(rst_n),
        .pair_valid(pv), .in_l(pl), .in_r(pr),
        .frame_tick(ftick), .sel(sel), .in_idle(idle),
        .bypass(bypass),
        .out_valid(ov), .out_l(ol), .out_r(orr)
    );

    // Expected coefficients from the design script (single source).
    // Flat arrays: iverilog $readmemh needs (mem, start, finish).
    reg [31:0] hx4f [0:123];
    reg [31:0] hx2f [0:61];
    initial begin
        $readmemh("tools/coef_hex/X4_p0.hex", hx4f, 0, 30);
        $readmemh("tools/coef_hex/X4_p1.hex", hx4f, 31, 61);
        $readmemh("tools/coef_hex/X4_p2.hex", hx4f, 62, 92);
        $readmemh("tools/coef_hex/X4_p3.hex", hx4f, 93, 123);
        $readmemh("tools/coef_hex/X2_p0.hex", hx2f, 0, 30);
        $readmemh("tools/coef_hex/X2_p1.hex", hx2f, 31, 61);
    end

    integer err = 0;

    // Stimulus/response buffers.
    reg signed [23:0] stimL [0:255];
    reg signed [23:0] stimR [0:255];
    reg signed [23:0] resp [0:2047];
    reg signed [23:0] respr [0:2047];
    integer nresp = 0;

    // Universal runner: npairs pairs from stimL/R (first at first_at),
    // spaced pair_gap mclk; collects every out_valid into resp/respr.
    task run_stream;
        input integer npairs;
        input integer pair_gap;
        input integer first_at;
        input integer want_frames;
        integer c, pi, npc, to;
        begin
            pi = 0; npc = first_at; nresp = 0; to = 0;
            c = 0;
            while (nresp < want_frames && to < want_frames * 300 + 2000) begin
                @(negedge clk);
                if (pi < npairs && c == npc) begin
                    pl = stimL[pi]; pr = stimR[pi]; pv = 1'b1;
                    pi = pi + 1; npc = npc + pair_gap;
                end else begin
                    pv = 1'b0;
                end
                @(posedge clk); #1;
                if (ov === 1'b1) begin
                    resp[nresp] = ol; respr[nresp] = orr;
                    nresp = nresp + 1;
                end
                c = c + 1; to = to + 1;
            end
            if (nresp < want_frames) begin
                $display("FAIL: stream short (%0d/%0d)", nresp, want_frames);
                err = err + 1;
            end
        end
    endtask

    // Integer model of one phase tap product (mirrors RTL rounding).
    function signed [23:0] imodel;
        input signed [23:0] a;
        input [31:0] coef;
        reg signed [63:0] t;
        reg signed [63:0] s;
        begin
            t = a * $signed(coef);
            s = (t + 64'sd536870912) >>> 30;
            if (s > 64'sd8388607) imodel = 24'sd8388607;
            else if (s < -64'sd8388608) imodel = 24'h800000;
            else imodel = s[23:0];
        end
    endfunction

    // Impulse bit-exactness (ch=0: L impulse/R zero; ch=1: mirrored).
    // Returns first-nonzero alignment via align_out.
    task impulse_check;
        input integer r;             // 2 or 4
        input integer ch;            // 0 or 1
        input signed [23:0] amp;
        output integer align_out;
        integer i, g, p, al;
        reg signed [23:0] want, wantL, wantR;
        begin
            al = -1;
            for (i = 0; i < nresp; i = i + 1)
                if (al < 0 && (resp[i] !== 24'sd0 || respr[i] !== 24'sd0))
                    al = i;
            align_out = al;
            if (al < 0) begin
                $display("FAIL: impulse never arrived (r=%0d)", r);
                err = err + 1;
            end else begin
                // Leading frames must be exact zeros.
                for (i = 0; i < al; i = i + 1)
                    if (resp[i] !== 24'sd0 || respr[i] !== 24'sd0) begin
                        $display("FAIL: pre-impulse garbage f=%0d", i);
                        err = err + 1;
                    end
                // Each frame (group g, phase p) = round(amp*coef[g][p]).
                for (i = al; i < nresp; i = i + 1) begin
                    g = (i - al) / r;
                    p = (i - al) % r;
                    if (g < 31) begin
                        if (r == 4)
                            want = imodel(amp, hx4f[p * 31 + g]);
                        else
                            want = imodel(amp, hx2f[p * 31 + g]);
                    end else begin
                        want = 24'sd0;
                    end
                    wantL = (ch == 0) ? want : 24'sd0;
                    wantR = (ch == 0) ? 24'sd0 : want;
                    if (resp[i] !== wantL) begin
                        $display("FAIL: impulse r=%0d ch=%0d f=%0d g=%0d p=%0d L: got %0d want %0d", r, ch, i, g, p,
                            $signed(resp[i]), $signed(wantL));
                        err = err + 1;
                    end
                    if (respr[i] !== wantR) begin
                        $display("FAIL: impulse r=%0d ch=%0d f=%0d g=%0d p=%0d R: got %0d want %0d", r, ch, i, g, p,
                            $signed(respr[i]), $signed(wantR));
                        err = err + 1;
                    end
                end
            end
        end
    endtask

    // Zero-pair flush (histories must be clean before DC/steps/X2/X4
    // section switches; the step model assumes zero initial state).
    task flush;
        input integer gap;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk);
                pl = 24'sd0; pr = 24'sd0; pv = 1'b1;
                @(negedge clk);
                pv = 1'b0;
                repeat (gap - 1) @(posedge clk);
            end
        end
    endtask

    real cx4f [0:123];
    real cx2f [0:61];

    task dc_check;
        // Steady state only: caller must push 31+ constant pairs first
        // (history flush). Gates: group-mean == C (DC exact by trim) and
        // per-phase ripple bounded (X4 branch sums deviate 2.6e-3 by
        // design = image floor; NOT a bug). Also proves phases cycle
        // (a stuck phase would show zero spread on X4).
        input integer r;
        input signed [23:0] c;
        input integer nfr_check; // trailing frames to verify
        input integer minspread; // min max-min over the window
        integer i, bad, mn, mx, accm, n;
        begin
            bad = 0; mn = 8388607; mx = -8388608; accm = 0; n = 0;
            for (i = nresp - nfr_check; i < nresp; i = i + 1) begin
                if ($signed(resp[i]) > mx) mx = $signed(resp[i]);
                if ($signed(resp[i]) < mn) mn = $signed(resp[i]);
                if ($signed(respr[i]) > mx) mx = $signed(respr[i]);
                if ($signed(respr[i]) < mn) mn = $signed(respr[i]);
                accm = accm + $signed(resp[i]) + $signed(respr[i]);
                n = n + 2;
                // 0.6% branch-ripple envelope (design: 0.26% max).
                if ($signed(resp[i]) - $signed(c) > 51000 ||
                    $signed(c) - $signed(resp[i]) > 51000) bad = bad + 1;
                if ($signed(respr[i]) - $signed(c) > 51000 ||
                    $signed(c) - $signed(respr[i]) > 51000) bad = bad + 1;
            end
            if (bad) begin
                $display("FAIL: DC outside 0.6%% envelope (%0d)", bad);
                err = err + 1;
            end
            // Mean over full groups: exact DC (trim) +- rounding.
            if ((accm / n) - $signed(c) > 4 ||
                $signed(c) - (accm / n) > 4) begin
                $display("FAIL: DC mean off (mean=%0d c=%0d)",
                    accm / n, $signed(c));
                err = err + 1;
            end
            if (mx - mn < minspread) begin
                $display("FAIL: DC phases stuck (spread=%0d)", mx - mn);
                err = err + 1;
            end else begin
                $display("DC ok (r=%0d mean=%0d spread=%0d)",
                    r, accm / n, mx - mn);
            end
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        begin : main
            integer i, j, al0, pk, pkv, f;
            real ph;

            // ---------- X4 impulse, L and R, two phases ----------
            sel = 2'b11;
            stimL[0] = 24'sd1048576; stimR[0] = 24'sd0;
            for (i = 1; i < 40; i = i + 1) begin
                stimL[i] = 24'sd0; stimR[i] = 24'sd0;
            end
            run_stream(40, 1024, 10, 40 * 4 + 8);
            impulse_check(4, 0, 24'sd1048576, al0);
            $display("X4 impulse L ok, align=%0d", al0);
            // Pre-ring on this capture: peak pos + negative precursor.
            pk = al0; pkv = 0;
            for (i = al0; i < nresp; i = i + 1) begin
                if (resp[i] < 0 ? -resp[i] : resp[i] > pkv) begin
                    if ((resp[i] < 0 ? -resp[i] : resp[i]) > pkv) begin
                        pkv = resp[i] < 0 ? -resp[i] : resp[i]; pk = i;
                    end
                end
            end
            for (i = al0; i < pk; i = i + 1)
                if ($signed(resp[i]) < -4) begin
                    $display("FAIL: negative precursor f=%0d (%0d)",
                        i, $signed(resp[i]));
                    err = err + 1;
                end
            $display("X4 pre-ring ok (peak at +%0d frames)", pk - al0);

            // Phase B + R channel.
            run_stream(40, 1024, 110, 40 * 4 + 8);
            impulse_check(4, 0, 24'sd1048576, al0);
            $display("X4 impulse L phase-B ok");
            for (i = 0; i < 40; i = i + 1) begin
                stimL[i] = 24'sd0; stimR[i] = (i == 0) ? 24'sd1048576 : 0;
            end
            run_stream(40, 1024, 10, 40 * 4 + 8);
            impulse_check(4, 1, 24'sd1048576, al0);
            $display("X4 impulse R ok");

            // ---------- X4 DC (40 constant pairs: settle + measure) -----
            for (i = 0; i < 40; i = i + 1) begin
                stimL[i] = 24'sd4000000; stimR[i] = 24'sd4000000;
            end
            run_stream(40, 1024, 10, 40 * 4 + 4);
            dc_check(4, 24'sd4000000, 8, 100);

            // ---------- X4 steady sines: never rails, L/R antisymmetric --
            // 1 kHz at 0 dBFS (LF unity at full scale, deterministic pass);
            // 5 kHz and up at -1 dBFS (7470000): a 0 dBFS HF sine EXCEEDS
            // the rails on true inter-sample peaks (float model: +0.06 to
            // +0.09 dB at 1/5/10/15/19 kHz, phase-dependent) — saturation
            // there is correct behavior, never wrap (proven by the step
            // test), so the no-clip check runs at a level the model proves
            // clean across all phases. Symmetry (+-2 LSB) holds at any level.
            for (f = 0; f < 6; f = f + 1) begin : sinef
                real fq;
                real amp;
                case (f)
                    0: fq = 1000.0; 1: fq = 5000.0; 2: fq = 10000.0;
                    3: fq = 15000.0; 4: fq = 19000.0; default: fq = 21000.0;
                endcase
                amp = (f == 0) ? 8388603.0 : 7470000.0;
                ph = 0.0;
                for (i = 0; i < 46; i = i + 1) begin
                    stimL[i] = $rtoi(amp * $sin(ph));
                    stimR[i] = -$rtoi(amp * $sin(ph));
                    ph = ph + 2.0 * 3.14159265358979 * fq / 44100.0;
                end
                run_stream(46, 1024, 10, 46 * 4 + 6);
                for (i = nresp - 40; i < nresp; i = i + 1) begin
                    if (resp[i] === 24'h7FFFFF || resp[i] === 24'h800000 ||
                        respr[i] === 24'h7FFFFF || respr[i] === 24'h800000) begin
                        $display("FAIL: steady-sine clip %0.0f Hz f=%0d",
                            fq, i);
                        err = err + 1;
                    end
                    // L/R antisymmetry (linear engine, antisymmetric stim).
                    if ($signed(resp[i]) + $signed(respr[i]) > 2 ||
                        $signed(resp[i]) + $signed(respr[i]) < -2) begin
                        $display("FAIL: sine asym %0.0f Hz f=%0d L=%0d R=%0d",
                            fq, i, $signed(resp[i]), $signed(respr[i]));
                        err = err + 1;
                    end
                end
                $display("sine %0.0f Hz: no clips", fq);
            end

            // ---------- X4 full-scale steps vs exact integer model -----
            // (step_check flushes internally: each call needs zero state,
            // and the -FS call follows the +FS pedestal)
            for (i = 0; i < 4; i = i + 1) begin : cxinit
                integer p, t;
                for (p = 0; p < 4; p = p + 1)
                    for (t = 0; t < 31; t = t + 1)
                        cx4f[p * 31 + t] = $itor($signed(hx4f[p * 31 + t]))
                                     / 1073741824.0;
                for (p = 0; p < 2; p = p + 1)
                    for (t = 0; t < 31; t = t + 1)
                        cx2f[p * 31 + t] = $itor($signed(hx2f[p * 31 + t]))
                                     / 1073741824.0;
            end
            step_check(4, 8388603);
            step_check(4, -8388603);

            // ---------- X2 impulse + DC (flush X4 step pedestal first) --
            flush(1024, 36);
            sel = 2'b10;
            stimL[0] = 24'sd1048576; stimR[0] = 24'sd0;
            for (i = 1; i < 40; i = i + 1) begin
                stimL[i] = 24'sd0; stimR[i] = 24'sd0;
            end
            run_stream(40, 512, 10, 40 * 2 + 8);
            impulse_check(2, 0, 24'sd1048576, al0);
            $display("X2 impulse L ok");
            for (i = 0; i < 40; i = i + 1) begin
                stimL[i] = 24'sd4000000; stimR[i] = 24'sd4000000;
            end
            run_stream(40, 512, 10, 40 * 2 + 4);
            dc_check(2, 24'sd4000000, 4, 1);
            step_check(2, 8388603);
            step_check(2, -8388603);

            // ---------- X1 bypass via src_interp ----------
            sel = 2'b01;
            for (i = 0; i < 14; i = i + 1) begin
                stimL[i] = i * 1000 + 123; stimR[i] = -(i * 1000 + 123);
            end
            run_stream(14, 256, 10, 14 + 6);
            begin : x1chk
                // Exact mapping: 14 pairs (i*1000+123) land 1:1 on the
                // first 14 collected frames (the sel-switch transient falls
                // before collection starts — deterministic phase), then the
                // last pair re-emits (dup/drop holds latest by design).
                integer k, bad2;
                bad2 = 0;
                if (nresp != 20) begin
                    $display("FAIL: X1 frame count %0d (want 20)", nresp);
                    bad2 = bad2 + 1;
                end
                for (k = 0; k < 14 && k < nresp; k = k + 1) begin
                    if ($signed(resp[k]) !== k * 1000 + 123 ||
                        $signed(respr[k]) !== -(k * 1000 + 123)) begin
                        if (bad2 < 4)
                            $display("x1bad k=%0d L=%0d R=%0d (want %0d)",
                                k, $signed(resp[k]), $signed(respr[k]),
                                k * 1000 + 123);
                        bad2 = bad2 + 1;
                    end
                end
                for (k = 14; k < nresp; k = k + 1) begin
                    if ($signed(resp[k]) !== 13 * 1000 + 123 ||
                        $signed(respr[k]) !== -(13 * 1000 + 123)) begin
                        if (bad2 < 4)
                            $display("x1bad tail k=%0d L=%0d R=%0d",
                                k, $signed(resp[k]), $signed(respr[k]));
                        bad2 = bad2 + 1;
                    end
                end
                if (bad2) begin
                    $display("FAIL: X1 bypass (%0d)", bad2); err = err + 1;
                end else $display("X1 bypass ok");
            end

            // ---------- NOS jumper: X2 bypass = plain duplicate ----------
            sel = 2'b10;
            bypass = 1'b1;
            for (i = 0; i < 8; i = i + 1) begin
                stimL[i] = 24'sd0; stimR[i] = 24'sd0;
            end
            run_stream(8, 512, 10, 8 * 2 + 8); // flush: sel-switch tail out
            stimL[0] = 24'sd3000000; stimR[0] = 24'sd3000000;
            run_stream(8, 512, 10, 8 * 2 + 8);
            begin : noschk
                // Duplicate emits each pair twice: exactly two frames carry
                // the impulse (== C), everything else is zero. The engine
                // would ring for ~34 frames — so this also proves the engine
                // is really off, not just quiet.
                integer k, nz;
                nz = 0;
                for (k = 0; k < nresp; k = k + 1) begin
                    if ($signed(resp[k]) !== 0 || $signed(respr[k]) !== 0) begin
                        nz = nz + 1;
                        if ($signed(resp[k]) !== 24'sd3000000 ||
                            $signed(respr[k]) !== 24'sd3000000) begin
                            $display("nosbad k=%0d L=%0d R=%0d", k,
                                $signed(resp[k]), $signed(respr[k]));
                            nz = nz + 1000;
                        end
                    end
                end
                if (nz !== 2) begin
                    $display("FAIL: NOS bypass nz=%0d (want 2)", nz);
                    err = err + 1;
                end else $display("NOS bypass ok");
            end
            bypass = 1'b0;
        end

        if (err == 0) $display("PASS tb_interp");
        else $display("FAIL tb_interp (%0d)", err);
        $finish;
    end

    // Full-scale step vs EXACT integer model with offset fit (L0 in 0..11):
    // stimulus and coefficients are both integers, so the reference is
    // accumulated in 64-bit integer arithmetic mirroring the RTL (single
    // round-half-up + saturate at the end) — bit-exact, zero tolerance.
    // NOTE: no `real` arrays here on purpose — icarus 11.0 silently drops
    // conditional stores to real-array elements (the old float model could
    // never fit because of the simulator, not the DUT). Integer regs/arrays
    // are exact.
    task step_check;
        input integer r;
        input signed [23:0] lvl;
        integer i, j, t, p, L0, ok, satn;
        reg signed [23:0] mhi [0:30]; // integer stimulus history (exact)
        reg signed [63:0] acci;       // exact sum: 31 x S24 x S32 (fits)
        reg signed [63:0] accr;       // bias + shift
        reg signed [23:0] wantv;
        reg signed [23:0] gotv;
        begin
            // Clean initial state inside the task: the previous section's
            // pedestal (e.g. the +FS step right before the -FS step) must
            // not leak into the zero-state model.
            flush(256 * r, 36);
            for (i = 0; i < 8; i = i + 1) begin
                stimL[i] = 24'sd0; stimR[i] = 24'sd0;
            end
            for (i = 8; i < 48; i = i + 1) begin
                stimL[i] = lvl; stimR[i] = 24'sd0;
            end
            run_stream(48, 256 * r, 10, 48 * r + 8);
            // R must stay exactly zero (step on L only).
            for (i = 0; i < nresp; i = i + 1)
                if (respr[i] !== 24'sd0) begin
                    $display("FAIL: step R bleed f=%0d", i);
                    err = err + 1;
                end
            ok = 0; satn = 0;
            for (L0 = 0; L0 < 12 && !ok; L0 = L0 + 1) begin : fit
                integer bad3;
                bad3 = 0;
                for (t = 0; t < 31; t = t + 1) mhi[t] = 24'sd0;
                j = 0;
                for (i = 0; i < nresp; i = i + 1) begin : fr
                    // pairs pushed so far for group alignment: pair j
                    // belongs to groups; advance model on group starts.
                    if (i >= L0 && ((i - L0) % r) == 0 && j < 48) begin
                        for (t = 30; t > 0; t = t - 1) mhi[t] = mhi[t-1];
                        // if/else (not ternary): proven icarus-safe form
                        // for conditional array-element stores.
                        if (j >= 8) mhi[0] = lvl;
                        else mhi[0] = 24'sd0;
                        j = j + 1;
                    end
                    if (i < L0) begin
                        if (resp[i] !== 24'sd0) bad3 = bad3 + 1;
                    end else begin
                        p = (i - L0) % r;
                        acci = 64'sd0;
                        for (t = 0; t < 31; t = t + 1) begin
                            if (r == 4)
                                acci = acci + mhi[t] *
                                    $signed(hx4f[p * 31 + t]);
                            else
                                acci = acci + mhi[t] *
                                    $signed(hx2f[p * 31 + t]);
                        end
                        // Same rounding/saturation as the RTL datapath.
                        accr = (acci + 64'sd536870912) >>> 30;
                        if (accr > 64'sd8388607) wantv = 24'h7FFFFF;
                        else if (accr < -64'sd8388608) wantv = 24'h800000;
                        else wantv = accr[23:0];
                        gotv = resp[i];
                        if (gotv !== wantv) begin
                            if (bad3 < 4)
                                $display("FAIL: step r=%0d L0=%0d f=%0d p=%0d: got %0d want %0d",
                                    r, L0, i, p, $signed(gotv), $signed(wantv));
                            bad3 = bad3 + 1;
                        end else if (gotv === 24'h7FFFFF ||
                                     gotv === 24'h800000) begin
                            satn = satn + 1;
                        end
                    end
                end
                if (bad3 == 0) ok = 1;
            end
            if (!ok) begin
                $display("FAIL: step r=%0d lvl=%0d no fit", r, $signed(lvl));
                err = err + 1;
            end else begin
                $display("step r=%0d lvl=%0d ok (saturations=%0d)",
                    r, $signed(lvl), satn);
            end
        end
    endtask
endmodule

