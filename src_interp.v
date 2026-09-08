`timescale 1ns/1ps
// Stage-2 SRC: quality polyphase interpolation for X2/X4, plain
// duplicate-or-drop for X1/D2 (reused proven src_dupdrop), one image.
//
// NOTE ON CODING STYLE: histories are scalar register chains with a
// case-multiplexer tap readout — deliberately NOT indexed arrays. A
// dynamic array index infers $mem (synchronous RAM on silicon needs the
// address a cycle earlier; sim would still pass — the worst kind of bug).
// Scalars + case mux = LUT logic everywhere, sim/silicon identical.
//
// X1 (native rate): bit-transparent bypass (only the final TPDF dither
// touches the word downstream) — native hires keeps its NOS character.
// D2 (8x in): drop every 2nd pair (aliasing of >88k content accepted,
// stated openly; 8x material has ~nothing up there).
// X2/X4: minimum-phase polyphase FIR interpolation (TDA1541 path):
//   X4 prototype N=121 (31 taps/phase), stop 59.7 dB from 24.1k,
//   ripple 0.09 dB, ZERO precursor (neg-precursor 0.00%, step preshoot 0);
//   X2 prototype N=25 (13 taps/phase, zero-padded to 31), stop 125 dB.
//   Coefficients Q2.30, DC exact by construction (see tools/).
// Engine: ONE shared 24x32 MAC + 64-bit accumulator, time-multiplexed
// across 2 channels x 31 taps (~70 mclk per output frame of 256).
// Sync scheme (robust to any constant pair/frame phase — clocks locked):
// depth-2 pair FIFO decouples arrival from frame_tick; each frame job
// pushes at most one pair into history, resets phase to 0 on push.
// Interpolator overshoot on hot transients SATURATES to S24 rails
// (never wraps); steady full-scale sines never clip (verified tb_interp).

module src_interp (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              pair_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    input  wire              frame_tick,   // 1-mclk pulse, output frame start
    input  wire [1:0]        sel,          // 00=D2 01=X1 10=X2 11=X4
    input  wire              in_idle,
    output wire              out_valid,
    output wire signed [23:0] out_l,
    output wire signed [23:0] out_r
);
    localparam integer TAPN = 31;

    // Coefficient ROM (module scope: Gowin Verilog-2001 forbids root-scope
    // declarations, so the include lives here, not at file top).
`include "interp_coefs.vh"

    // ---- proven X1/D2 path (also runs during X2/X4, output ignored) ----
    wire dd_valid;
    wire signed [23:0] dd_l, dd_r;
    src_dupdrop u_dd (
        .clk(clk), .rst_n(rst_n),
        .pair_valid(pair_valid), .in_l(in_l), .in_r(in_r),
        .frame_tick(frame_tick), .sel(sel), .in_idle(in_idle),
        .out_valid(dd_valid), .out_l(dd_l), .out_r(dd_r)
    );

    wire use_engine = (sel == 2'b10) || (sel == 2'b11);
    wire [2:0] R = (sel == 2'b11) ? 3'd4 : 3'd2;

    // ---- engine state (scalars only, see header note) ----
    reg signed [23:0] fbL0, fbL1, fbR0, fbR1; // pair FIFO, depth 2
    reg fwptr, frptr;
    reg [1:0] occu;
    reg signed [23:0] hold_l, hold_r;   // latest pair (sel-change push)
    // History chains, [0] = newest. Shift via unrolled static loop.
    reg signed [23:0] hL00,hL01,hL02,hL03,hL04,hL05,hL06,hL07,hL08,hL09,hL10;
    reg signed [23:0] hL11,hL12,hL13,hL14,hL15,hL16,hL17,hL18,hL19,hL20;
    reg signed [23:0] hL21,hL22,hL23,hL24,hL25,hL26,hL27,hL28,hL29,hL30;
    reg signed [23:0] hR00,hR01,hR02,hR03,hR04,hR05,hR06,hR07,hR08,hR09,hR10;
    reg signed [23:0] hR11,hR12,hR13,hR14,hR15,hR16,hR17,hR18,hR19,hR20;
    reg signed [23:0] hR21,hR22,hR23,hR24,hR25,hR26,hR27,hR28,hR29,hR30;
    reg [1:0] phase;
    reg [1:0] prev_sel;
    reg signed [23:0] push_l, push_r;   // S0 snapshot for SPUSH
    // Pre-shift hL00 snapshot (= post-shift hL01). The L-t1 MAC load
    // (first hist_u use after the shift) must NOT read the function mux
    // combinationally: the shift and the st->SMACL flip settle in the
    // same step, and icarus samples the function-mux stale (pre-shift),
    // dropping the t1 term on push jobs (non-push histories are static,
    // R history settles ~35 cycles before SMACR — both were already exact).
    // On silicon the mux would settle in time, but the snapshot is
    // bit-identical there — sim/silicon stay the same by construction.
    reg signed [23:0] snap1_l;
    reg do_push;                        // latched in S0, executed in SPUSH
    // States: 0 idle, 1 push-history, 2 MAC_L, 3 MAC_R, 4 emit.
    // SPUSH exists because a same-edge push+t0-read would feed tap t0
    // with pre-push data (proven on silicon-trace: group-0 p0 came out 0).
    reg [2:0] st;
    reg [5:0] k;                        // MAC step 0..TAPN+1 (=32)
    reg zero_frame;                     // in_idle: emit zeros, skip MAC

    // Shared datapath (2-stage product pipeline, retired-filter recipe).
    reg signed [23:0] mb_r;
    reg signed [31:0] co_r;
    reg signed [55:0] prod;
    reg signed [63:0] acc;

    reg eng_valid;
    reg signed [23:0] eng_l, eng_r;

    // Tap readout: static case mux (no dynamic array index anywhere).
    function [23:0] tapL;
        input [4:0] idx;
        begin
            case (idx)
                5'd0: tapL = hL00; 5'd1: tapL = hL01; 5'd2: tapL = hL02;
                5'd3: tapL = hL03; 5'd4: tapL = hL04; 5'd5: tapL = hL05;
                5'd6: tapL = hL06; 5'd7: tapL = hL07; 5'd8: tapL = hL08;
                5'd9: tapL = hL09; 5'd10: tapL = hL10; 5'd11: tapL = hL11;
                5'd12: tapL = hL12; 5'd13: tapL = hL13; 5'd14: tapL = hL14;
                5'd15: tapL = hL15; 5'd16: tapL = hL16; 5'd17: tapL = hL17;
                5'd18: tapL = hL18; 5'd19: tapL = hL19; 5'd20: tapL = hL20;
                5'd21: tapL = hL21; 5'd22: tapL = hL22; 5'd23: tapL = hL23;
                5'd24: tapL = hL24; 5'd25: tapL = hL25; 5'd26: tapL = hL26;
                5'd27: tapL = hL27; 5'd28: tapL = hL28; 5'd29: tapL = hL29;
                default: tapL = hL30;
            endcase
        end
    endfunction
    function [23:0] tapR;
        input [4:0] idx;
        begin
            case (idx)
                5'd0: tapR = hR00; 5'd1: tapR = hR01; 5'd2: tapR = hR02;
                5'd3: tapR = hR03; 5'd4: tapR = hR04; 5'd5: tapR = hR05;
                5'd6: tapR = hR06; 5'd7: tapR = hR07; 5'd8: tapR = hR08;
                5'd9: tapR = hR09; 5'd10: tapR = hR10; 5'd11: tapR = hR11;
                5'd12: tapR = hR12; 5'd13: tapR = hR13; 5'd14: tapR = hR14;
                5'd15: tapR = hR15; 5'd16: tapR = hR16; 5'd17: tapR = hR17;
                5'd18: tapR = hR18; 5'd19: tapR = hR19; 5'd20: tapR = hR20;
                5'd21: tapR = hR21; 5'd22: tapR = hR22; 5'd23: tapR = hR23;
                5'd24: tapR = hR24; 5'd25: tapR = hR25; 5'd26: tapR = hR26;
                5'd27: tapR = hR27; 5'd28: tapR = hR28; 5'd29: tapR = hR29;
                default: tapR = hR30;
            endcase
        end
    endfunction

    // Tap index valid (0..30) exactly on the load steps (k<=30, see FSM).
    wire [4:0] tidx = k[4:0];
    wire [7:0] rom_addr = {(sel == 2'b11), phase, tidx};
    wire signed [23:0] hist_u = (st == 3'd2) ? $signed(tapL(tidx))
                                             : $signed(tapR(tidx));

    // Round-half-up + saturate (S24 rails; overshoot never wraps).
    wire signed [63:0] acc_r = acc + 64'sd536870912;
    wire signed [63:0] acc_sh = acc_r >>> 30;
    wire signed [23:0] y0w = (acc_sh > 64'sd8388607) ? 24'sd8388607 :
                        (acc_sh < -64'sd8388608) ? 24'h800000 :
                        acc_sh[23:0];

    // FIFO give/take (SINGLE occu assignment — the pair port and the
    // frame tick can coincide on the same clock edge (locked clocks,
    // arbitrary constant phase); separate occu++/occu-- in one block
    // would resolve by statement order and lose the arriving pair's
    // count, desyncing fr/fw permanently. One combined update is
    // race-free: coincident arrival + consume nets zero, data paths use
    // pre-edge pointers on both sides (consume oldest, buffer newest).
    wire give = pair_valid && (occu != 2'd2);
    wire s0fire = (st == 3'd0) && frame_tick && use_engine;
    wire take = s0fire && (occu != 2'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hold_l <= 24'sd0; hold_r <= 24'sd0;
            fbL0 <= 24'sd0; fbL1 <= 24'sd0;
            fbR0 <= 24'sd0; fbR1 <= 24'sd0;
            fwptr <= 1'b0; frptr <= 1'b0; occu <= 2'd0;
            push_l <= 24'sd0; push_r <= 24'sd0;
            snap1_l <= 24'sd0;
            hL00<=24'sd0;hL01<=24'sd0;hL02<=24'sd0;hL03<=24'sd0;hL04<=24'sd0;
            hL05<=24'sd0;hL06<=24'sd0;hL07<=24'sd0;hL08<=24'sd0;hL09<=24'sd0;
            hL10<=24'sd0;hL11<=24'sd0;hL12<=24'sd0;hL13<=24'sd0;hL14<=24'sd0;
            hL15<=24'sd0;hL16<=24'sd0;hL17<=24'sd0;hL18<=24'sd0;hL19<=24'sd0;
            hL20<=24'sd0;hL21<=24'sd0;hL22<=24'sd0;hL23<=24'sd0;hL24<=24'sd0;
            hL25<=24'sd0;hL26<=24'sd0;hL27<=24'sd0;hL28<=24'sd0;hL29<=24'sd0;
            hL30<=24'sd0;
            hR00<=24'sd0;hR01<=24'sd0;hR02<=24'sd0;hR03<=24'sd0;hR04<=24'sd0;
            hR05<=24'sd0;hR06<=24'sd0;hR07<=24'sd0;hR08<=24'sd0;hR09<=24'sd0;
            hR10<=24'sd0;hR11<=24'sd0;hR12<=24'sd0;hR13<=24'sd0;hR14<=24'sd0;
            hR15<=24'sd0;hR16<=24'sd0;hR17<=24'sd0;hR18<=24'sd0;hR19<=24'sd0;
            hR20<=24'sd0;hR21<=24'sd0;hR22<=24'sd0;hR23<=24'sd0;hR24<=24'sd0;
            hR25<=24'sd0;hR26<=24'sd0;hR27<=24'sd0;hR28<=24'sd0;hR29<=24'sd0;
            hR30<=24'sd0;
            phase <= 2'd0; prev_sel <= 2'b01;
            do_push <= 1'b0;
            st <= 3'd0; k <= 6'd0; zero_frame <= 1'b0;
            mb_r <= 24'sd0; co_r <= 32'sd0;
            prod <= 56'sd0; acc <= 64'sd0;
            eng_valid <= 1'b0; eng_l <= 24'sd0; eng_r <= 24'sd0;
        end else begin
            eng_valid <= 1'b0;
            prod <= mb_r * co_r;
            if (pair_valid) begin
                hold_l <= in_l;
                hold_r <= in_r;
            end
            if (give) begin
                if (fwptr == 1'b0) begin
                    fbL0 <= in_l; fbR0 <= in_r;
                end else begin
                    fbL1 <= in_l; fbR1 <= in_r;
                end
                fwptr <= ~fwptr;
            end
            // Single occu update (see header note): coincident
            // give+take nets zero — the pair is buffered AND the oldest
            // is consumed in the same cycle, pointers stay in sync.
            case ({give, take})
                2'b10: occu <= occu + 2'd1;
                2'b01: occu <= occu - 2'd1;
                default: ;
            endcase
            case (st)
                3'd0: begin
                    if (s0fire) begin
                        // Consume oldest unread pair if any; else, on a
                        // ratio switch, push the latest pair for a fast
                        // relock transient (one group max).
                        do_push <= take || (sel != prev_sel);
                        if (take) begin
                            if (frptr == 1'b0) begin
                                push_l <= fbL0; push_r <= fbR0;
                            end else begin
                                push_l <= fbL1; push_r <= fbR1;
                            end
                            frptr <= ~frptr;
                        end else begin
                            push_l <= hold_l;
                            push_r <= hold_r;
                        end
                        // t1 snapshot for the L MAC (see decl note):
                        // push job -> pre-shift hL00 (= post-shift hL01);
                        // quiet job -> hL01 itself (no shift happens).
                        snap1_l <= (take || (sel != prev_sel)) ? hL00 : hL01;
                        if (sel != prev_sel)
                            prev_sel <= sel;
                        // Fresh data (or fresh ratio) starts a group.
                        if (take || (sel != prev_sel))
                            phase <= 2'd0;
                        k <= 6'd1;
                        if (in_idle) begin
                            zero_frame <= 1'b1;
                            st <= 3'd4;
                        end else begin
                            st <= 3'd1; // SPUSH always: push + L-t0 preload
                        end
                    end
                end
                3'd1: begin
                    // Dedicated history-push cycle (see header note).
                    // Runs EVERY job (also when do_push=0 — then histories
                    // are untouched and t0 comes from the settled hL00).
                    // L-channel t0 operands come straight from the S0
                    // snapshot, never via the history read: the t0 tap
                    // must use the just pushed sample even though the
                    // shift settles the same cycle the MAC would read it.
                    if (do_push) begin
                    hL30<=hL29;hL29<=hL28;hL28<=hL27;hL27<=hL26;
                    hL26<=hL25;hL25<=hL24;hL24<=hL23;hL23<=hL22;
                    hL22<=hL21;hL21<=hL20;hL20<=hL19;hL19<=hL18;
                    hL18<=hL17;hL17<=hL16;hL16<=hL15;hL15<=hL14;
                    hL14<=hL13;hL13<=hL12;hL12<=hL11;hL11<=hL10;
                    hL10<=hL09;hL09<=hL08;hL08<=hL07;hL07<=hL06;
                    hL06<=hL05;hL05<=hL04;hL04<=hL03;hL03<=hL02;
                    hL02<=hL01;hL01<=hL00;hL00<=push_l;
                    hR30<=hR29;hR29<=hR28;hR28<=hR27;hR27<=hR26;
                    hR26<=hR25;hR25<=hR24;hR24<=hR23;hR23<=hR22;
                    hR22<=hR21;hR21<=hR20;hR20<=hR19;hR19<=hR18;
                    hR18<=hR17;hR17<=hR16;hR16<=hR15;hR15<=hR14;
                    hR14<=hR13;hR13<=hR12;hR12<=hR11;hR11<=hR10;
                    hR10<=hR09;hR09<=hR08;hR08<=hR07;hR07<=hR06;
                    hR06<=hR05;hR05<=hR04;hR04<=hR03;hR03<=hR02;
                    hR02<=hR01;hR01<=hR00;hR00<=push_r;
                        mb_r <= push_l;
                    end else begin
                        mb_r <= hL00;
                    end
                    co_r <= $signed(interp_rom({(sel == 2'b11), phase, 5'd0}));
                    acc <= 64'sd0;
                    st <= 3'd2;
                end
                3'd2, 3'd3: begin
                    // 31 taps, SMACL enters at k=1 (L-t0 preloaded by
                    // SPUSH from the S0 snapshot): k=1 loads t1 (pipeline
                    // fill, NO accumulate), k=2..30 accumulate + load,
                    // k=31..32 drain, k=33 latches eng (acc settled).
                    // SMACR enters at k=0 (R-t0 from settled history).
                    if (k == 6'd0) begin
                        acc <= 64'sd0;
                        mb_r <= hist_u;
                        co_r <= $signed(interp_rom(rom_addr));
                    end else if (k == 6'd1) begin
                        // L t1 from the S0 snapshot (never the function
                        // mux — see snap1_l note); R t1 from settled hist.
                        mb_r <= (st == 3'd2) ? snap1_l : hist_u;
                        co_r <= $signed(interp_rom(rom_addr));
                    end else if (k <= TAPN - 1) begin
                        acc <= acc + prod;
                        mb_r <= hist_u;
                        co_r <= $signed(interp_rom(rom_addr));
                    end else if (k <= TAPN + 1) begin
                        acc <= acc + prod;
                    end
                    if (k == TAPN + 2) begin
                        k <= 6'd0;
                        if (st == 3'd2) begin
                            eng_l <= y0w;
                            st <= 3'd3;
                        end else begin
                            eng_r <= y0w;
                            st <= 3'd4;
                        end
                    end else begin
                        k <= k + 6'd1;
                    end
                end
                3'd4: begin
                    if (zero_frame) begin
                        eng_l <= 24'sd0;
                        eng_r <= 24'sd0;
                    end else if (phase + 2'd1 >= R) begin
                        phase <= 2'd0;
                    end else begin
                        phase <= phase + 2'd1;
                    end
                    eng_valid <= 1'b1;
                    st <= 3'd0;
                end
                default: st <= 3'd0;
            endcase
        end
    end

    assign out_valid = use_engine ? eng_valid : dd_valid;
    assign out_l = use_engine ? eng_l : dd_l;
    assign out_r = use_engine ? eng_r : dd_r;
endmodule
