`timescale 1ns/1ps
// DATA2 source-select bench (unit on dsd_to_pcm, DSD64 = 16 mclk/bit).
// Transport 1 (Amanero): DATA2 rides LRCLK, alt pin idle -> use_alt=0.
// Transport 2: DATA2 on the dedicated pin, LRCLK static -> use_alt=1.
// Sparse alt crosstalk (< 16 transitions) must NOT hijack (use_alt=0).
// Idle (dsd_on=0) re-arms the decision every session.
// Unit truth (swap to match PCM lives in top): SDATA bit -> stage-L.
module tb_dsd_srcsel;
    reg mclk = 1'b0;
    always #11.072 mclk = ~mclk;

    reg rst_n = 1'b0;
    reg dsd_on = 1'b0;
    reg dclk = 1'b0, d1 = 1'b0, d2lr = 1'b0, d2alt = 1'b0;

    wire sv;
    wire signed [31:0] sl, sr;
    dsd_to_pcm dut (
        .mclk(mclk), .rst_n(rst_n), .dsd_on(dsd_on),
        .dsd_clk_in(dclk), .dsd_data1(d1),
        .dsd_data2_alt(d2alt), .lrck_in(d2lr),
        .sample_valid(sv), .sample_l(sl), .sample_r(sr)
    );

    integer err = 0;

    // One DSD bit: data stable around the rising edge.
    task bit1;
        input b1, b2lr, b2alt;
        begin
            d1 = b1; d2lr = b2lr; d2alt = b2alt; dclk = 1'b0;
            repeat (8) @(posedge mclk);
            dclk = 1'b1;
            repeat (8) @(posedge mclk);
        end
    endtask

    task sniff32;
        output signed [31:0] ml;
        output signed [31:0] mr;
        integer k, n;
        reg signed [63:0] a, b;
        begin
            a = 0; b = 0; n = 0;
            while (n < 32) begin
                @(posedge mclk); #1;
                if (sv === 1'b1) begin
                    a = a + $signed(sl); b = b + $signed(sr); n = n + 1;
                end
            end
            ml = a / 32; mr = b / 32;
        end
    endtask

    reg signed [31:0] ml, mr;
    integer k;

    initial begin
        repeat (100) @(posedge mclk);
        rst_n = 1'b1;
        repeat (10) @(posedge mclk);

        // ---- A. Amanero: DATA2(DC 0) on LRCLK, alt idle ----
        dsd_on = 1'b1;
        for (k = 0; k < 6000; k = k + 1) bit1(1'b1, 1'b0, 1'b0);
        if (dut.use_alt !== 1'b0) begin
            $display("FAIL: A use_alt=1 on idle alt"); err = err + 1;
        end
        sniff32(ml, mr);
        $display("A: use_alt=%b L=%0d R=%0d", dut.use_alt,
            $signed(ml), $signed(mr));
        if (!($signed(ml) > 536870912)) begin
            $display("FAIL: A L not +rail"); err = err + 1;
        end
        if (!($signed(mr) < -536870912)) begin
            $display("FAIL: A R not -rail"); err = err + 1;
        end

        // ---- B. transport 2: DATA2(silence) on alt, LRCLK static ----
        dsd_on = 1'b0;
        repeat (2000) @(posedge mclk);   // idle re-arm
        if (dut.use_alt !== 1'b0) begin
            $display("FAIL: idle did not re-arm"); err = err + 1;
        end
        dsd_on = 1'b1;
        for (k = 0; k < 6000; k = k + 1)
            bit1(1'b1, 1'b1, k[0]);     // d1 DC, lr static, alt 0101
        if (dut.use_alt !== 1'b1) begin
            $display("FAIL: B use_alt=0 on live alt"); err = err + 1;
        end
        sniff32(ml, mr);
        $display("B: use_alt=%b L=%0d R=%0d", dut.use_alt,
            $signed(ml), $signed(mr));
        if (!($signed(ml) > 536870912)) begin
            $display("FAIL: B L not +rail"); err = err + 1;
        end
        if (!($signed(mr) > -4194304 && $signed(mr) < 4194304)) begin
            $display("FAIL: B R not silence"); err = err + 1;
        end

        // ---- C. sparse alt crosstalk must not hijack ----
        dsd_on = 1'b0;
        repeat (2000) @(posedge mclk);
        dsd_on = 1'b1;
        for (k = 0; k < 6000; k = k + 1)
            // lr DC 0, alt: 5 isolated blips = 10 transitions (< 16)
            bit1(1'b1, 1'b0, (k == 500 || k == 1000 || k == 1500
                || k == 2000 || k == 2500) ? 1'b1 : 1'b0);
        if (dut.use_alt !== 1'b0) begin
            $display("FAIL: C use_alt=1 on crosstalk"); err = err + 1;
        end
        sniff32(ml, mr);
        $display("C: use_alt=%b L=%0d R=%0d", dut.use_alt,
            $signed(ml), $signed(mr));
        if (!($signed(ml) > 536870912)) begin
            $display("FAIL: C L not +rail"); err = err + 1;
        end
        if (!($signed(mr) < -536870912)) begin
            $display("FAIL: C R not -rail (LRCLK lost)"); err = err + 1;
        end

        // ---- D. decision re-taken every session ----
        dsd_on = 1'b0;
        repeat (2000) @(posedge mclk);
        if (dut.use_alt !== 1'b0) begin
            $display("FAIL: D no re-arm"); err = err + 1;
        end
        dsd_on = 1'b1;
        for (k = 0; k < 3000; k = k + 1) bit1(1'b1, 1'b1, k[0]);
        if (dut.use_alt !== 1'b1) begin
            $display("FAIL: D no re-decide"); err = err + 1;
        end else $display("D: re-decide ok");

        if (err == 0) $display("PASS tb_dsd_srcsel");
        else $display("FAIL tb_dsd_srcsel (%0d)", err);
        $finish;
    end

    initial begin
        repeat (30000000) @(posedge mclk);
        $display("WATCHDOG hang"); $finish;
    end
endmodule
