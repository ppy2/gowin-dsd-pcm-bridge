`timescale 1ns/1ps
// Stage-1 SRC bench: rate_detect + src_dupdrop on the 256-grid.
// sel codes are stage-LOCAL ratios (00=X1 01=X2 10=X4):
//   N=256  sel=X1(00): out steps +1/frame (pass)
//   N=512  sel=X2(01): runs of 2, steps +1  (duplicate)
//   N=1024 sel=X4(10): runs of 4, steps +1  (duplicate x4)
// (Overall-X1/N=128 bypasses stage-1 in top — receiver-direct to stage-2 —
// so N=128 is not a stage-1 operating point and is not tested here.
// Overall-X8 maps to stage-1 X4, so sel=11 never reaches an instance.)
// Every frame: out_r == -out_l (pair coherence + channel integrity).
// Plus: glitch ride-through (one wrong period must not switch sel),
// idle mute (no LRCK -> zeros, valid still ticking), relock, and X1 at
// two extra pair/frame phases (locked clocks, arbitrary phase must work).
module tb_src;
    reg clk = 1'b0;
    always #10.1725 clk = ~clk; // 49.152 MHz (ratios only — any MCLK works)

    reg rst_n = 1'b0;
    reg lrck = 1'b0;
    reg pv = 1'b0;
    reg signed [23:0] dl = 24'sd0;
    reg signed [23:0] dr = 24'sd0;

    wire [1:0] sel;
    wire idle;
    // Top maps overall->stage-1-local (s1sel = rate-1); mirror it here so
    // the dupdrop sees what stage-1 sees in hardware.
    wire [1:0] ls1 = (sel == 2'b00) ? 2'b00 : sel - 2'b01;
    wire svalid;
    wire signed [23:0] sl, sr;

    rate_detect u_det(.clk(clk), .rst_n(rst_n),
        .lrck_in(lrck), .sel(sel), .in_idle(idle));

    // Output frame grid: tick every 256 mclk, phase-loadable.
    reg [7:0] fcnt8 = 8'd0;
    reg frame_en = 1'b0;
    reg [7:0] fphase = 8'd0;
    wire frame_tick = frame_en && (fcnt8 == 8'd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) fcnt8 <= 8'd0;
        else if (!frame_en) fcnt8 <= fphase;
        else fcnt8 <= fcnt8 + 8'd1;
    end

    src_dupdrop u_src(.clk(clk), .rst_n(rst_n),
        .pair_valid(pv), .in_l(dl), .in_r(dr),
        .frame_tick(frame_tick), .sel(ls1), .in_idle(idle),
        .out_valid(svalid), .out_l(sl), .out_r(sr));

    integer err = 0;
    integer idx = 1;

    // N input periods; pair_valid pulse at offset poff; L=idx, R=-idx.
    task input_frames;
        input [10:0] period;
        input integer nfr;
        input integer poff;
        integer f, c;
        begin
            for (f = 0; f < nfr; f = f + 1) begin
                @(negedge clk);
                lrck = 1'b1; pv = 1'b0;
                for (c = 1; c < period; c = c + 1) begin
                    if (c == poff) begin
                        pv = 1'b1; dl = idx; dr = -idx; idx = idx + 1;
                    end
                    @(negedge clk);
                    if (c == poff) pv = 1'b0;
                    if (c == period / 2) lrck = 1'b0;
                end
            end
        end
    endtask

    // Collector: stores out pairs while collect=1.
    reg collect = 1'b0;
    reg signed [23:0] got[0:79];
    reg signed [23:0] gotr[0:79];
    integer gcnt = 0;
    always @(posedge clk) begin
        #1;
        if (collect && svalid && gcnt < 80) begin
            got[gcnt] = sl; gotr[gcnt] = sr; gcnt = gcnt + 1;
        end
    end

    task wait_sel;
        input [1:0] want;
        integer to;
        begin
            to = 0;
            while (sel !== want && to < 40) begin
                input_frames(curN, 1, 30);
                to = to + 1;
            end
            if (sel !== want) begin
                $display("FAIL: sel=%b, want %b (N=%0d)", sel, want, curN);
                err = err + 1;
            end
        end
    endtask

    reg [10:0] curN = 11'd256;

    task check_run;
        input [1:0] want_sel;
        input integer mode; // 0:+1 2:x2 3:x4
        integer i, step, k, R, ok, expv, fit;
        begin
            wait_sel(want_sel);
            // settle past the sel-switch transient, then collect 40 frames
            input_frames(curN, 6, 30);
            collect = 1'b1; gcnt = 0;
            input_frames(curN, (40 * 256) / curN + 2, 30);
            collect = 1'b0;
            if (gcnt < 40) begin
                $display("FAIL: only %0d frames collected (N=%0d)", gcnt, curN);
                err = err + 1;
            end
            for (i = 2; i < 40; i = i + 1) begin
                if (gotr[i] !== -got[i]) begin
                    $display("FAIL: L/R pair broken f=%0d L=%0d R=%0d",
                        i, $signed(got[i]), $signed(gotr[i]));
                    err = err + 1;
                end
                step = $signed(got[i]) - $signed(got[i-1]);
                if (mode == 0 && step !== 1) begin
                    $display("FAIL: X1 step=%0d at f=%0d", step, i);
                    err = err + 1;
                end
            end
            if (mode >= 2) begin
                // Runs of R identical frames; collection may start mid-run,
                // so try every offset k: e(i) = got[2] + (i+k)/R - (2+k)/R.
                R = (mode == 2) ? 2 : 4;
                fit = 0;
                for (k = 0; k < R; k = k + 1) begin
                    ok = 1;
                    for (i = 2; i < 40; i = i + 1) begin
                        expv = $signed(got[2]) + (i + k) / R - (2 + k) / R;
                        if ($signed(got[i]) !== expv) ok = 0;
                    end
                    if (ok) fit = 1;
                end
                if (!fit) begin
                    $display("FAIL: X%0d run structure broken (N=%0d)",
                        (mode == 2) ? 2 : 4, curN);
                    err = err + 1;
                end
            end
            $display("ratio N=%0d sel=%b mode=%0d ok (40 frames)", curN, sel, mode);
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        frame_en = 1'b1;
        repeat (5) @(posedge clk);

        curN = 256;  check_run(2'b01, 0); // overall X2 -> s1 X1 pass
        curN = 512;  check_run(2'b10, 2); // overall X4 -> s1 X2 dup
        curN = 1024; check_run(2'b11, 3); // overall X8 -> s1 X4 dup

        // X1 at two more pair/frame phases (locked, arbitrary phase).
        // (poff counts from 1: the c-loop starts at 1.)
        curN = 256;
        wait_sel(2'b01);
        input_frames(curN, 6, 1);
        collect = 1'b1; gcnt = 0;
        input_frames(curN, 42, 1);
        collect = 1'b0;
        begin
            integer i;
            for (i = 2; i < 40; i = i + 1)
                if ($signed(got[i]) - $signed(got[i-1]) !== 1) begin
                    $display("FAIL: X1 phase0 step at f=%0d", i);
                    err = err + 1;
                end
        end
        $display("X1 phase0 ok");
        input_frames(curN, 6, 100);
        collect = 1'b1; gcnt = 0;
        input_frames(curN, 42, 100);
        collect = 1'b0;
        begin
            integer i;
            for (i = 2; i < 40; i = i + 1)
                if ($signed(got[i]) - $signed(got[i-1]) !== 1) begin
                    $display("FAIL: X1 phase100 step at f=%0d", i);
                    err = err + 1;
                end
        end
        $display("X1 phase100 ok");

        // Glitch ride-through: one wrong-length period must not switch sel.
        begin
            integer c;
            @(negedge clk); lrck = 1'b1; pv = 1'b0;
            for (c = 1; c < 300; c = c + 1) begin
                @(negedge clk);
                if (c == 150) lrck = 1'b0;
            end
        end
        input_frames(256, 2, 30);
        if (sel !== 2'b01) begin
            $display("FAIL: glitch switched sel to %b", sel);
            err = err + 1;
        end else $display("glitch ride-through ok");

        // Idle: stop everything -> zeros, valid still ticking.
        pv = 1'b0; lrck = 1'b0;
        repeat (2200) @(posedge clk);
        if (idle !== 1'b1) begin
            $display("FAIL: idle not flagged"); err = err + 1;
        end
        collect = 1'b1; gcnt = 0;
        repeat (256 * 10) @(posedge clk);
        collect = 1'b0;
        begin
            integer i;
            if (gcnt < 8) begin
                $display("FAIL: no frames during idle"); err = err + 1;
            end
            for (i = 0; i < gcnt; i = i + 1)
                if (got[i] !== 0 || gotr[i] !== 0) begin
                    $display("FAIL: idle not zero f=%0d", i);
                    err = err + 1;
                end
        end
        $display("idle mute ok (%0d zero frames)", gcnt);

        // Relock after idle.
        curN = 256;
        wait_sel(2'b01);
        $display("relock ok");

        if (err == 0) $display("PASS tb_src");
        else $display("FAIL tb_src (%0d)", err);
        $finish;
    end
endmodule
