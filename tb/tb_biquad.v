// Biquad/filter bench: lpf_8k_4th_seq driven at the 192 kHz sample grid.
// Race-free: stimulus on negedge, capture after NBA; the sequential
// filter answers ~30 cycles after in_valid, so each sample waits for
// out_valid (cadence exactness is irrelevant: the filter is
// sample-based, tone phase advances per sample).
// Gates: 1 kHz ~0 dB, 8 kHz -3.01 dB, 16 kHz <= -24 dB, DC gain = 1.
`timescale 1ns/1ps
module tb_biquad;
    reg clk = 1'b0;
    always #10.1725 clk = ~clk; // 49.152 MHz

    reg rst_n = 1'b0;
    reg in_valid = 1'b0;
    reg signed [23:0] in_data = 24'sd0;
    wire out_valid;
    wire signed [23:0] out_l;

    lpf_8k_4th_seq dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_l(in_data), .in_r(in_data),
        .out_valid(out_valid), .out_l(out_l), .out_r()
    );

    integer err = 0;
    integer i;
    real phase, sum2, rms, gain_db, mean;
    real A = 7000000.0;

    // One sample: pulse in_valid (seen exactly once: sampling posedge
    // sits between the two negedges), wait for out_valid, capture.
    task put_sample;
        integer to;
        begin
            @(negedge clk);
            in_valid = 1'b1;
            @(negedge clk);
            in_valid = 1'b0;
            to = 0;
            while (out_valid !== 1'b1 && to < 500) begin
                @(posedge clk);
                to = to + 1;
                #1;
            end
            if (to >= 500) begin
                $display("FAIL: out_valid timeout");
                err = err + 1;
                $finish;
            end
        end
    endtask

    task idle_tail;
        begin
            repeat (100) @(posedge clk);
        end
    endtask

    task run_tone;
        input real freq;
        input integer nsamp;
        input integer nskip;
        output real gout;
        integer k;
        begin
            phase = 0.0;
            sum2 = 0.0;
            for (k = 0; k < nsamp; k = k + 1) begin
                in_data = $rtoi(A * $sin(phase));
                phase = phase + 2.0 * 3.14159265358979 * freq / 192000.0;
                put_sample;
                if (k >= nskip)
                    sum2 = sum2 + $itor($signed(out_l)) * $itor($signed(out_l));
                idle_tail;
            end
            rms = $sqrt(sum2 / (nsamp - nskip));
            gout = rms * 1.41421356237310 / A;
        end
    endtask

    real g1k, g8k, g15k;

    initial begin
        repeat (10) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (100) @(posedge clk);

        // 1 kHz @192k: period = 192 samples; measured window must hold an
        // integer number of periods (768 = 4), else RMS reads high.
        run_tone(1000.0, 960, 192, g1k);
        gain_db = 20.0 * $log10(g1k);
        $display("1kHz gain = %+7.3f dB", gain_db);
        if (gain_db > 0.3 || gain_db < -0.3) begin
            $display("FAIL: 1kHz out of +-0.3 dB"); err = err + 1;
        end

        run_tone(8000.0, 480, 240, g8k);
        gain_db = 20.0 * $log10(g8k);
        $display("8kHz gain = %+7.3f dB (expect -3.01)", gain_db);
        if (gain_db > -2.5 || gain_db < -3.5) begin
            $display("FAIL: 8kHz not at -3 dB"); err = err + 1;
        end

        run_tone(16000.0, 960, 480, g15k);
        gain_db = 20.0 * $log10(g15k);
        $display("16kHz gain = %+7.3f dB (gate <= -24)", gain_db);
        if (gain_db > -24.0 || gain_db < -32.0) begin
            $display("FAIL: 16kHz stopband gate"); err = err + 1;
        end

        // DC gain = 1
        mean = 0.0;
        for (i = 0; i < 400; i = i + 1) begin
            in_data = 24'sd4000000;
            put_sample;
            if (i >= 200) mean = mean + $itor($signed(out_l));
            idle_tail;
        end
        mean = mean / 200.0 / 4000000.0;
        $display("DC gain = %+.5f (expect 1.0)", mean);
        if (mean > 1.01 || mean < 0.99) begin
            $display("FAIL: DC gain"); err = err + 1;
        end

        if (err == 0) $display("PASS tb_biquad");
        else $display("FAIL tb_biquad (%0d)", err);
        $finish;
    end
endmodule
