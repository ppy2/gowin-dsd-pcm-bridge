`timescale 1ns/1ps
// shaper_24_16 bench (DITH_ATTN=1 ship dose; ATTN=0 legacy is covered by
// identical RTL with full dither): boundedness/stability/DC/no-wrap +
// dump for the spectral proof (in-band shaped power vs flat TPDF,
// analyzed offline with the calibrated LPF meter).
// NTF shape itself is proven by the dump FFT, not by asserts here.
module tb_shaper_24_16;
    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;
    reg vld = 1'b0;
    reg signed [23:0] pl = 24'sd0, pr = 24'sd0;

    wire ov;
    wire signed [15:0] ol, orr;
    shaper_24_16 #(.DITH_ATTN(1)) dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(vld), .in_l(pl), .in_r(pr),
        .out_valid(ov), .out_l(ol), .out_r(orr)
    );

    integer err = 0;
    integer sat_y = 0;   // y at rails (informative: legal on hot steps)
    integer i, fd;
    integer sum0;
    reg signed [31:0] mxe;

    task step1;
        begin
            @(posedge clk);
            vld <= 1'b1;
            @(posedge clk);
            vld <= 1'b0;
            #1;
        end
    endtask

    // Drive one pair, return outputs via ol/orr/ov (sampled after edge).
    task feed;
        input signed [23:0] l;
        input signed [23:0] r;
        begin
            pl <= l; pr <= r;
            step1();
            if (ov !== 1'b1) begin
                $display("FAIL: valid gap"); err = err + 1;
            end
            if (^ol === 1'bx || ^orr === 1'bx) begin
                $display("FAIL: X on output"); err = err + 1;
            end
            if (ol === 16'sd32767 || ol === -16'sd32768) sat_y = sat_y + 1;
            // wrap discriminator: a wrap lands ~65536 away from input
            if ((($signed(ol) <<< 8) - l > 60000) ||
                (($signed(ol) <<< 8) - l < -60000)) begin
                $display("FAIL: wrap L in=%0d out=%0d", l, $signed(ol));
                err = err + 1;
            end
        end
    endtask

    initial begin
        #(100);
        rst_n <= 1'b1;
        #(40);

        // ---- 1. idle zeros: bounded, zero-mean, dump for spectrum ----
        fd = $fopen("build/sim/shaper_idle.hex", "w");
        sum0 = 0;
        mxe = 0;
        for (i = 0; i < 500000; i = i + 1) begin
            feed(24'sd0, 24'sd0);
            if (ol > 8 || ol < -8 || orr > 8 || orr < -8) begin
                $display("FAIL: idle unbounded #%0d L=%0d R=%0d", i, $signed(ol), $signed(orr));
                err = err + 1;
                i = 500000;
            end
            sum0 = sum0 + ol;
            if (i < 500000) $fwrite(fd, "%04h\n", ol & 16'hffff);
            if ($signed(dut.e1l) > mxe) mxe = $signed(dut.e1l);
            if (-$signed(dut.e1l) > mxe) mxe = -$signed(dut.e1l);
        end
        $fclose(fd);
        $display("idle: mean=%0d max|e1|=%0d (bound 2047)", sum0 / 500000, mxe);
        if (sum0 / 500000 > 2 || sum0 / 500000 < -2) begin
            $display("FAIL: idle DC"); err = err + 1;
        end else $display("idle bounded ok");

        // ---- 2. DC exactness across range ----
        // Model: safe/256 + ~1 LSB known bias (round-half-up +0.5 plus the
        // EF error mean +0.5: same sub-LSB class as the proven TPDF's +0.5,
        // inaudible, DAC offsets dominate — see header). Tolerance +-3
        // still catches any wrap (65536 off) or instability.
        begin : dcchk
            integer k, acc;
            real want_r;
            integer dcvals [0:5];
            dcvals[0] = 0; dcvals[1] = 256000; dcvals[2] = -256000;
            dcvals[3] = 4000000; dcvals[4] = -4000000; dcvals[5] = 8388607;
            for (k = 0; k < 6; k = k + 1) begin
                acc = 0;
                for (i = 0; i < 20000; i = i + 1) begin
                    feed(dcvals[k], dcvals[k]);
                    if (i > 1000) acc = acc + ol;
                end
                want_r = ($signed(dcvals[k]) - ($signed(dcvals[k]) >>> 13)) / 256.0 + 1.0;
                if (acc / 19000.0 - want_r > 3.0 || acc / 19000.0 - want_r < -3.0) begin
                    $display("FAIL: DC %0d mean=%0d want~%0.1f", dcvals[k], acc / 19000, want_r);
                    err = err + 1;
                end
            end
            $display("DC ok (sat_y so far=%0d)", sat_y);
        end

        // ---- 3. -6 dBFS 1 kHz sine: no rail saturation, tracks input ----
        begin : sinechk
            integer n, bad;
            real ph;
            reg signed [23:0] s;
            integer worst;
            bad = 0; worst = 0;
            sat_y = 0;
            for (n = 0; n < 200000; n = n + 1) begin
                ph = 6.283185307179586 * 1000.0 * n / 384000.0;
                s = $rtoi(4194304.0 * $sin(ph));
                feed(s, s);
                if (ol === 16'sd32767 || ol === -16'sd32768) bad = bad + 1;
                if ((($signed(ol) <<< 8) - s > worst))
                    worst = ($signed(ol) <<< 8) - s;
                if ((s - ($signed(ol) <<< 8)) > worst)
                    worst = s - ($signed(ol) <<< 8);
            end
            $display("sine -6dB: rail-hits=%0d worst-track-err=%0d", bad, worst);
            if (bad != 0) begin
                $display("FAIL: saturation on -6dB sine"); err = err + 1;
            end
            if (worst > 4000) begin
                $display("FAIL: sine not tracked"); err = err + 1;
            end else $display("sine track ok");
        end

        // ---- 4. full-scale steps: saturate (never wrap), recover fast ----
        begin : stepchk
            integer n, settled;
            sat_y = 0;
            for (n = 0; n < 5000; n = n + 1) feed(24'sd8388607, -24'sd8388608);
            settled = 0;
            for (n = 0; n < 200; n = n + 1) begin
                feed(24'sd0, 24'sd0);
                if (ol < 512 && ol > -512) settled = settled + 1;
            end
            $display("step: rail-hits=%0d settled-frames=%0d/200", sat_y, settled);
            if (settled < 150) begin
                $display("FAIL: overload not recovered"); err = err + 1;
            end else $display("step recover ok");
        end

        if (err == 0) $display("PASS tb_shaper_24_16");
        else $display("FAIL tb_shaper_24_16 (%0d)", err);
        $finish;
    end
endmodule
