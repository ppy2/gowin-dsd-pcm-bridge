`timescale 1ns/1ps
// Stage-1 SRC: plain duplicate-or-drop to the fixed output grid
// (output frame = 256 mclk in BOTH domains: 176.4 kHz @45.1584 MHz,
// 192 kHz @49.152 MHz). No FIR, no interpolation in this stage — just
// sample repetition / decimation, exactly as specified for stage 1.
//
// Rule: at every output frame_tick the emitter latches the latest input
// pair when (frame_cnt % R == 0), else re-emits the latched pair:
//   sel=X1/D2 (R=1): latch every frame (D2 sees 2 pairs/frame, latest wins)
//   sel=X2    (R=2): latch every 2nd frame -> each pair out twice
//   sel=X4    (R=4): latch every 4th frame -> each pair out four times
// Input and output are frequency-locked (one MCLK net), so this is exact;
// only the constant phase is arbitrary, which dup/drop tolerates.
// L/R travel atomically (single edges) — pair coherence by construction.
// sel change: counter resets and the fresh pair is taken immediately.
// in_idle: emit zeros (the downstream TPDF dither turns them into proper
// dithered digital silence, +-1 LSB).
module src_dupdrop (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              pair_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    input  wire              frame_tick,   // 1-mclk pulse, output frame start
    input  wire [1:0]        sel,          // x384 codes: 00=X1 01=X2 10=X4
    input  wire              in_idle,      // (11=X8 overall: dup x4 @dd)
    output reg               out_valid,
    output reg  signed [23:0] out_l,
    output reg  signed [23:0] out_r
);
    reg signed [23:0] hold_l, hold_r;  // latest input pair (atomic)
    reg [1:0] fcnt;                    // frame counter mod 4
    reg [1:0] prev_sel;

    // Latch on fcnt%R==0. R in {1,2,4} all divide the mod-4 counter.
    // sel=11 never arrives (top maps overall-X8 to per-stage X4/X2);
    // it falls through to latch-every-frame.
    wire latch_by_cnt = (sel == 2'b10) ? (fcnt == 2'd0) :
                        (sel == 2'b01) ? (fcnt[0] == 1'b0) : 1'b1;
    wire do_latch = (sel != prev_sel) || latch_by_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hold_l <= 24'sd0; hold_r <= 24'sd0;
            fcnt <= 2'd0;
            prev_sel <= 2'b00;
            out_valid <= 1'b0;
            out_l <= 24'sd0; out_r <= 24'sd0;
        end else begin
            out_valid <= 1'b0;
            if (pair_valid) begin
                hold_l <= in_l;
                hold_r <= in_r;
            end
            if (frame_tick) begin
                prev_sel <= sel;
                if (sel != prev_sel)
                    fcnt <= 2'd0;
                else
                    fcnt <= fcnt + 2'd1;
                if (in_idle) begin
                    out_l <= 24'sd0;
                    out_r <= 24'sd0;
                end else if (do_latch) begin
                    // NOTE: when pair_valid lands on this same edge, hold
                    // still has the previous pair (NBA). One frame of extra
                    // latency, both channels coherent — always a valid pick.
                    out_l <= hold_l;
                    out_r <= hold_r;
                end
                out_valid <= 1'b1;
            end
        end
    end
endmodule
