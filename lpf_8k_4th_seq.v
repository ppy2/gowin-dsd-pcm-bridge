`timescale 1ns/1ps
// Stereo 4th-order Butterworth LPF, fc = 8 kHz @ fs = 192 kHz.
// SEQUENTIAL single-MAC version: one registered 24x32 multiply and one
// 64-bit accumulator are time-shared across 4 biquad units x 5 MAC ops
// = 20 steps. ~34 mclk per stereo pair (budget is 256 @192 kHz).
// Drop-in replacement for lpf_8k_4th (same ports). Bit-exact vs the
// parallel Direct Form I: identical integer math, see tb_seq_diff.
// DSP cost: 1 multiplier (was 14 MULT27X36 macros / 100% of DSP).
module lpf_8k_4th_seq (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    output reg         out_valid,
    output reg  signed [23:0] out_l,
    output reg  signed [23:0] out_r
);
    // Stage coefficients, Q2.30 (RBJ cookbook, Q = 0.5412 / 1.3066).
    // Designed for fs = 192 kHz:
    //   python3 tools/gen_lpf_coeffs.py --fc 8000 --fs 192000
    localparam [31:0] S1B0 = 32'h00E1450A;
    localparam [31:0] S1B1 = 32'h01C28A14;
    localparam [31:0] S1B2 = 32'h00E1450A;
    localparam [31:0] S1A1 = 32'h9C38744F;
    localparam [31:0] S1A2 = 32'h274C9FD8;
    localparam [31:0] S2B0 = 32'h00FDFAE1;
    localparam [31:0] S2B1 = 32'h01FBF5C1;
    localparam [31:0] S2B2 = 32'h00FDFAE1;
    localparam [31:0] S2A1 = 32'h8F80F701;
    localparam [31:0] S2A2 = 32'h3476F481;

    // Unit order: 0 = L/sec1, 1 = L/sec2, 2 = R/sec1, 3 = R/sec2.
    // unit[0] = 0 -> stage 1, 1 -> stage 2.
    reg [1:0] unit;
    reg [2:0] k;
    reg busy;
    reg pending;

    // Per-unit delay states.
    reg signed [23:0] x1a, x2a, y1a, y2a; // L sec1
    reg signed [23:0] x1b, x2b, y1b, y2b; // L sec2
    reg signed [23:0] x1c, x2c, y1c, y2c; // R sec1
    reg signed [23:0] x1d, x2d, y1d, y2d; // R sec2
    reg signed [23:0] mid_l, mid_r;       // sec1 outputs
    reg signed [23:0] bl, br;             // input buffer
    // Run snapshot: X0 of every unit must come from the SAME pair even
    // if a new in_valid lands mid-run (L/R coherence).
    reg signed [23:0] rl0, rr0;

    // Shared datapath.
    reg signed [23:0] mb_r;
    reg signed [31:0] co_r;
    reg signed [55:0] prod;
    reg signed [63:0] acc;

    // Operand muxes for the active unit.
    wire signed [31:0] B0u = unit[0] ? $signed(S2B0) : $signed(S1B0);
    wire signed [31:0] B1u = unit[0] ? $signed(S2B1) : $signed(S1B1);
    wire signed [31:0] B2u = unit[0] ? $signed(S2B2) : $signed(S1B2);
    wire signed [31:0] A1u = unit[0] ? $signed(S2A1) : $signed(S1A1);
    wire signed [31:0] A2u = unit[0] ? $signed(S2A2) : $signed(S1A2);
    wire signed [23:0] X0u = (unit == 2'd0) ? rl0 :
                             (unit == 2'd1) ? mid_l :
                             (unit == 2'd2) ? rr0 : mid_r;
    wire signed [23:0] X1u = (unit == 2'd0) ? x1a :
                             (unit == 2'd1) ? x1b :
                             (unit == 2'd2) ? x1c : x1d;
    wire signed [23:0] X2u = (unit == 2'd0) ? x2a :
                             (unit == 2'd1) ? x2b :
                             (unit == 2'd2) ? x2c : x2d;
    wire signed [23:0] Y1u = (unit == 2'd0) ? y1a :
                             (unit == 2'd1) ? y1b :
                             (unit == 2'd2) ? y1c : y1d;
    wire signed [23:0] Y2u = (unit == 2'd0) ? y2a :
                             (unit == 2'd1) ? y2b :
                             (unit == 2'd2) ? y2c : y2d;

    // Round-half-up + saturate (same as the parallel version).
    wire signed [63:0] acc_sh = (acc + 64'sd536870912) >>> 30;
    wire signed [23:0] y0w = (acc_sh > 64'sd8388607) ? 24'sd8388607 :
                             (acc_sh < -64'sd8388608) ? 24'h800000 :
                             acc_sh[23:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            unit <= 2'd0;
            k <= 3'd0;
            busy <= 1'b0;
            pending <= 1'b0;
            x1a <= 24'sd0; x2a <= 24'sd0; y1a <= 24'sd0; y2a <= 24'sd0;
            x1b <= 24'sd0; x2b <= 24'sd0; y1b <= 24'sd0; y2b <= 24'sd0;
            x1c <= 24'sd0; x2c <= 24'sd0; y1c <= 24'sd0; y2c <= 24'sd0;
            x1d <= 24'sd0; x2d <= 24'sd0; y1d <= 24'sd0; y2d <= 24'sd0;
            mid_l <= 24'sd0; mid_r <= 24'sd0;
            bl <= 24'sd0; br <= 24'sd0;
            rl0 <= 24'sd0; rr0 <= 24'sd0;
            mb_r <= 24'sd0; co_r <= 32'sd0;
            prod <= 56'sd0; acc <= 64'sd0;
            out_valid <= 1'b0;
            out_l <= 24'sd0; out_r <= 24'sd0;
        end else begin
            out_valid <= 1'b0;
            prod <= mb_r * co_r;
            if (in_valid) begin
                bl <= in_l;
                br <= in_r;
                pending <= 1'b1;
            end
            if (!busy) begin
                if (pending) begin
                    pending <= 1'b0;
                    busy <= 1'b1;
                    rl0 <= bl;
                    rr0 <= br;
                    unit <= 2'd0;
                    k <= 3'd0;
                end
            end else begin
                // Product pipeline is TWO stages (operand regs + prod
                // reg): issued at k, consumed at k+2. The add/sub select
                // at the consume edge matches the issue two cycles back.
                case (k)
                    3'd0: begin acc <= 64'sd0; co_r <= B0u; mb_r <= X0u; end
                    3'd1: begin co_r <= B1u; mb_r <= X1u; end
                    3'd2: begin acc <= acc + prod; co_r <= B2u; mb_r <= X2u; end
                    3'd3: begin acc <= acc + prod; co_r <= A1u; mb_r <= Y1u; end
                    3'd4: begin acc <= acc + prod; co_r <= A2u; mb_r <= Y2u; end
                    3'd5: begin acc <= acc - prod; end
                    3'd6: begin acc <= acc - prod; end
                    3'd7: begin
                        case (unit)
                            2'd0: begin
                                x2a <= x1a; x1a <= rl0;
                                y2a <= y1a; y1a <= y0w;
                                mid_l <= y0w;
                            end
                            2'd1: begin
                                x2b <= x1b; x1b <= mid_l;
                                y2b <= y1b; y1b <= y0w;
                            end
                            2'd2: begin
                                x2c <= x1c; x1c <= rr0;
                                y2c <= y1c; y1c <= y0w;
                                mid_r <= y0w;
                            end
                            2'd3: begin
                                x2d <= x1d; x1d <= mid_r;
                                y2d <= y1d; y1d <= y0w;
                                out_l <= y1b;
                                out_r <= y0w;
                                out_valid <= 1'b1;
                                busy <= 1'b0;
                            end
                        endcase
                    end
                    default: begin end
                endcase
                if (k == 3'd7) begin
                    k <= 3'd0;
                    if (unit != 2'd3)
                        unit <= unit + 2'd1;
                end else begin
                    k <= k + 3'd1;
                end
            end
        end
    end
endmodule
