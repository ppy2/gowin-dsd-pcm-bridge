`timescale 1ns/1ps
// Dual-DUT equivalence: parallel lpf_8k_4th (reference) vs sequential
// lpf_8k_4th_seq. Same stimulus into both; the seq latency is unknown
// a priori, so lock it on the first output (scan the ref history for a
// match), then demand bit-exact out_valid + L/R on every cycle.
module tb_seq_diff;
    reg clk = 1'b0;
    always #10.1725 clk = ~clk; // 49.152 MHz

    reg rst_n = 1'b0;
    reg in_valid = 1'b0;
    reg signed [23:0] in_data = 24'sd0;

    wire ref_valid;
    wire signed [23:0] ref_l, ref_r;
    wire dut_valid;
    wire signed [23:0] dut_l, dut_r;

    lpf_8k_4th refm (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_l(in_data), .in_r(in_data),
        .out_valid(ref_valid), .out_l(ref_l), .out_r(ref_r)
    );
    lpf_8k_4th_seq dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_l(in_data), .in_r(in_data),
        .out_valid(dut_valid), .out_l(dut_l), .out_r(dut_r)
    );

    // reference history (valid + data), depth covers any sane latency
    reg rh_v [0:63];
    reg signed [23:0] rh_l [0:63];
    reg signed [23:0] rh_r [0:63];
    integer i;
    integer lat = -1;
    integer err = 0;
    integer ncmp = 0;
    integer to;
    reg locking = 1'b1;

    always @(posedge clk) begin
        for (i = 63; i > 0; i = i - 1) begin
            rh_v[i] <= rh_v[i-1];
            rh_l[i] <= rh_l[i-1];
            rh_r[i] <= rh_r[i-1];
        end
        rh_v[0] <= ref_valid;
        rh_l[0] <= ref_l;
        rh_r[0] <= ref_r;
    end

    // race-free stimulus: setup on negedge, capture after NBA
    task put_sample;
        input [23:0] v;
        begin
            @(negedge clk);
            in_data = v;
            in_valid = 1'b1;
            @(negedge clk);
            in_valid = 1'b0;
            repeat (3) @(posedge clk);
            #1;
        end
    endtask

    task check_point;
        input integer do_idle;
        integer j;
        begin
            // called 3 cycles after deassert; dut output may come later:
            // wait for it (bounded), then compare at locked latency
            to = 0;
            while (dut_valid !== 1'b1 && to < 120) begin
                @(posedge clk); #1;
                to = to + 1;
            end
            if (to >= 120) begin
                $display("FAIL: dut out_valid timeout");
                err = err + 1;
            end else if (locking) begin
                // lock latency: exactly one history match required
                // (counter stimulus makes every value unique)
                lat = -1;
                for (j = 0; j < 64; j = j + 1) begin
                    if (rh_v[j] === 1'b1 &&
                        rh_l[j] === dut_l && rh_r[j] === dut_r) begin
                        if (lat < 0) lat = j;
                        else lat = -2; // ambiguous, keep scanning
                    end
                end
                if (lat >= 0) begin
                    locking = 1'b0;
                    $display("locked latency = %0d cycles", lat);
                end else begin
                    lat = -1;
                end
            end else begin
                    ncmp = ncmp + 1;
                    if (rh_v[lat] !== 1'b1 ||
                        rh_l[lat] !== dut_l || rh_r[lat] !== dut_r) begin
                        $display("FAIL: mismatch n=%0d dut=%0d/%0d ref=%0d/%0d",
                            ncmp, $signed(dut_l), $signed(dut_r),
                            $signed(rh_l[lat]), $signed(rh_r[lat]));
                        err = err + 1;
                    end
                end
            if (do_idle)
                repeat (40) @(posedge clk);
        end
    endtask

    real ph;
    integer k;
    integer rtmp;
    reg signed [23:0] rv;

    initial begin
        for (i = 0; i < 64; i = i + 1) begin
            rh_v[i] = 1'b0; rh_l[i] = 24'sd0; rh_r[i] = 24'sd0;
        end
        repeat (10) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (50) @(posedge clk);

        // lock phase: unique counter values -> unambiguous latency lock
        for (k = 0; k < 70; k = k + 1) begin
            put_sample((k + 1) * 1000);
            check_point(1);
        end
        if (locking) begin
            $display("FAIL: latency never locked");
            err = err + 1;
            $finish;
        end
        // DC
        for (k = 0; k < 200; k = k + 1) begin
            put_sample(24'sd4000000);
            check_point(1);
        end
        // sines 1k / 8k / 15k
        ph = 0.0;
        for (k = 0; k < 300; k = k + 1) begin
            put_sample($rtoi(7000000.0 * $sin(ph)));
            ph = ph + 2.0 * 3.14159265358979 * 1000.0 / 48000.0;
            check_point(1);
        end
        ph = 0.0;
        for (k = 0; k < 300; k = k + 1) begin
            put_sample($rtoi(7000000.0 * $sin(ph)));
            ph = ph + 2.0 * 3.14159265358979 * 8000.0 / 48000.0;
            check_point(1);
        end
        ph = 0.0;
        for (k = 0; k < 300; k = k + 1) begin
            put_sample($rtoi(7000000.0 * $sin(ph)));
            ph = ph + 2.0 * 3.14159265358979 * 15000.0 / 48000.0;
            check_point(1);
        end
        // random full-scale (low 24 bits of $random: all patterns incl. negative)
        for (k = 0; k < 1500; k = k + 1) begin
            rtmp = $random;
            rv = rtmp[23:0];
            put_sample(rv);
            check_point(1);
        end
        // rails + zero
        for (k = 0; k < 200; k = k + 1) begin
            if (k % 3 == 0) put_sample(24'sd8388607);
            else if (k % 3 == 1) put_sample(24'h800000);
            else put_sample(24'sd0);
            check_point(1);
        end
        // one overlapping pair (pending path): second pulse mid-run
        put_sample(24'sd1000000);
        @(negedge clk);
        in_data = 24'sd2000000;
        in_valid = 1'b1;
        @(negedge clk);
        in_valid = 1'b0;
        repeat (3) @(posedge clk);
        #1;
        check_point(0);
        check_point(1);

        $display("compared %0d outputs, latency %0d", ncmp, lat);
        if (err == 0) $display("PASS tb_seq_diff");
        else $display("FAIL tb_seq_diff (%0d)", err);
        $finish;
    end
endmodule
