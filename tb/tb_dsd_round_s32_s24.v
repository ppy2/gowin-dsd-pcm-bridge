`timescale 1ns/1ps
// Regression for the DSD S32(Q31) -> S24 boundary.
// This must scale by >>> 8 before S24 saturation. Rail-only tests cannot
// catch an accidental pre-shift clamp: ordinary music levels then collapse
// to a rail and create a harmonic comb.
module tb_dsd_round_s32_s24;
    reg signed [31:0] in_s32;
    wire signed [23:0] out_s24;
    integer errors = 0;

    dsd_round_s32_s24 dut (
        .in_s32(in_s32),
        .out_s24(out_s24)
    );

    task check;
        input signed [31:0] value;
        input signed [23:0] want;
        begin
            in_s32 = value;
            #1;
            if (out_s24 !== want) begin
                $display("FAIL: in=%0d got=%0d want=%0d",
                    $signed(value), $signed(out_s24), $signed(want));
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        check(32'sh00000000, 24'sh000000);
        check(32'sh0000007F, 24'sh000000); // +127/256 rounds down
        check(32'sh00000080, 24'sh000001); // +0.5 LSB rounds up
        check(32'shFFFFFF7F, 24'shFFFFFF); // -129/256 rounds to -1
        check(32'sh10000000, 24'sh100000); // +1/8 FS: must not rail
        check(32'sh20000000, 24'sh200000); // +1/4 FS: must not rail
        check(32'sh40000000, 24'sh400000); // +1/2 FS: must not rail
        check(32'shE0000000, 24'shE00000); // -1/4 FS: must not rail
        check(32'sh7FFFFFFF, 24'sh7FFFFF); // positive rail saturates
        check(32'sh80000000, 24'sh800000); // negative rail is exact

        if (errors == 0)
            $display("PASS tb_dsd_round_s32_s24");
        else
            $display("FAIL tb_dsd_round_s32_s24 (%0d)", errors);
        $finish;
    end
endmodule
