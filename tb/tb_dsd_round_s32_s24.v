`timescale 1ns/1ps
// Regression for the DSD S32(Q31) -> +6 dB -> S24 boundary.
// x2 with S24 saturation (commercial-DAC DSD loudness match): ordinary
// levels double without railing; SACD-0dB peaks (+1/2 FS) land on the
// rail; rails stay railed. Rail-only tests cannot catch an accidental
// pre-shift clamp: music would collapse to a rail (harmonic comb).
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
        check(32'sh0000007F, 24'sh000001); // 127*2+128 rounds up to 1
        check(32'sh00000040, 24'sh000001); // 64*2=128, +128>>8 = 1
        check(32'shFFFFFF7F, 24'shFFFFFF); // -129*2-130, >>>8 = -1
        check(32'sh10000000, 24'sh200000); // +1/8 FS doubles, must not rail
        check(32'sh20000000, 24'sh400000); // +1/4 FS doubles, must not rail
        check(32'sh40000000, 24'sh7FFFFF); // +1/2 FS (SACD 0dB peak) on rail
        check(32'shE0000000, 24'shC00000); // -1/4 FS doubles, must not rail
        check(32'shC0000000, 24'sh800000); // -1/2 FS doubles to exact rail
        check(32'sh7FFFFFFF, 24'sh7FFFFF); // positive rail saturates
        check(32'sh80000000, 24'sh800000); // negative rail saturates

        if (errors == 0)
            $display("PASS tb_dsd_round_s32_s24");
        else
            $display("FAIL tb_dsd_round_s32_s24 (%0d)", errors);
        $finish;
    end
endmodule
