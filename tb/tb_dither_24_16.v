`timescale 1ns/1ps
// Dither contract (S24 -> S16), adapted from the proven Q31 tb in
// rebuild_i2s/stage2_16bit_dither/tb/tb_quantize16_tpdf.v:
// - no out_valid without in_valid;
// - legal 0 dBFS endpoints keep guard headroom: outputs in +-32763..32765,
//   NEVER the S16 rails (saturation solved by construction, not by clamp);
// - exact-grid codes move by at most one LSB and the dither visibly varies;
// - L/R sequences are decorrelated (identical inputs still diverge).
module tb_dither_24_16;
    reg clk = 1'b0;
    always #10.1725 clk = ~clk; // 49.152 MHz
    reg rst_n = 1'b0;
    reg in_valid = 1'b0;
    reg signed [23:0] in_l = 24'sd0;
    reg signed [23:0] in_r = 24'sd0;
    wire out_valid;
    wire signed [15:0] out_l;
    wire signed [15:0] out_r;

    dither_24_16 dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_l(in_l), .in_r(in_r),
        .out_valid(out_valid), .out_l(out_l), .out_r(out_r)
    );

    integer err = 0;
    integer got_l, got_r, i;
    integer min_seen, max_seen, div_seen;

    task push;
        input signed [23:0] vl;
        input signed [23:0] vr;
        begin
            @(negedge clk);
            in_l = vl; in_r = vr; in_valid = 1'b1;
            @(negedge clk);
            in_valid = 1'b0;
            if (out_valid !== 1'b1) begin
                $display("FAIL: no out_valid for a pair"); err = err + 1;
            end
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        if (out_valid !== 1'b0) begin
            $display("FAIL: valid while idle"); err = err + 1;
        end

        // +0 dBFS: headroom, never rails
        for (i = 0; i < 48; i = i + 1) begin
            push(24'h7FFFFF, 24'h7FFFFF);
            got_l = $signed(out_l); got_r = $signed(out_r);
            if (got_l < 32763 || got_l > 32765 ||
                out_l === 16'h7fff || out_l === 16'h8000) begin
                $display("FAIL +0dBFS L: %0d", got_l); err = err + 1;
            end
            if (got_r < 32763 || got_r > 32765 ||
                out_r === 16'h7fff || out_r === 16'h8000) begin
                $display("FAIL +0dBFS R: %0d", got_r); err = err + 1;
            end
        end
        // -0 dBFS
        for (i = 0; i < 48; i = i + 1) begin
            push(24'h800000, 24'h800000);
            got_l = $signed(out_l); got_r = $signed(out_r);
            if (got_l < -32765 || got_l > -32763 ||
                out_l === 16'h7fff || out_l === 16'h8000) begin
                $display("FAIL -0dBFS L: %0d", got_l); err = err + 1;
            end
            if (got_r < -32765 || got_r > -32763 ||
                out_r === 16'h7fff || out_r === 16'h8000) begin
                $display("FAIL -0dBFS R: %0d", got_r); err = err + 1;
            end
        end
        // +0.5FS boundary: nominal 16382 +-1, must vary
        min_seen = 99999; max_seen = -99999;
        for (i = 0; i < 48; i = i + 1) begin
            push(24'h400000, 24'h400000);
            got_l = $signed(out_l);
            if (got_l < 16381 || got_l > 16383) begin
                $display("FAIL +0.5FS bound: %0d", got_l); err = err + 1;
            end
            if (got_l < min_seen) min_seen = got_l;
            if (got_l > max_seen) max_seen = got_l;
        end
        if (min_seen == max_seen) begin
            $display("FAIL: dither static at boundary (%0d)", min_seen);
            err = err + 1;
        end
        // -0.5FS boundary
        for (i = 0; i < 48; i = i + 1) begin
            push(24'hC00000, 24'hC00000);
            got_l = $signed(out_l);
            if (got_l < -16383 || got_l > -16381) begin
                $display("FAIL -0.5FS bound: %0d", got_l); err = err + 1;
            end
        end
        // Digital silence: unbiased, within +-1
        min_seen = 99999; max_seen = -99999;
        for (i = 0; i < 48; i = i + 1) begin
            push(24'sd0, 24'sd0);
            got_l = $signed(out_l);
            if (got_l < -1 || got_l > 1) begin
                $display("FAIL silence bound: %0d", got_l); err = err + 1;
            end
            if (got_l < min_seen) min_seen = got_l;
            if (got_l > max_seen) max_seen = got_l;
        end
        if (min_seen == max_seen) begin
            $display("FAIL: silence never dithered"); err = err + 1;
        end
        // L/R decorrelation: identical inputs must sometimes diverge
        div_seen = 0;
        for (i = 0; i < 48; i = i + 1) begin
            push(24'h123456, 24'h123456);
            if (out_l !== out_r) div_seen = div_seen + 1;
        end
        if (div_seen == 0) begin
            $display("FAIL: L/R dither locked"); err = err + 1;
        end

        if (err == 0) $display("PASS tb_dither_24_16 (div=%0d/48)", div_seen);
        else $display("FAIL tb_dither_24_16 (%0d)", err);
        $finish;
    end
endmodule
