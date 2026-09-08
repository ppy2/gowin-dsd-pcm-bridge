`timescale 1ns/1ps
// I2S slave receiver, Philips format with the standard 1-BCLK delay.
// Shifts on BCLK rising edges; 32 BCLK per slot is the nominal grid,
// 16..32 BCLK slots are tolerated (word is left-aligned).
// Output: 24-bit signed (slot MSB-aligned), one pair strobe per frame.
// The true Philips word straddles the LRCK edge (MSB lands one BCLK after
// it), so the word is completed on the first BCLK rise after the edge.
module i2s_receiver (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        bclk_in,
    input  wire        lrck_in,
    input  wire        sdata_in,
    output reg         out_valid,
    output reg  signed [23:0] out_l,
    output reg  signed [23:0] out_r
);
    // input synchronizers (source clocks are asynchronous to mclk_in)
    reg [1:0] bclk_s;
    reg [1:0] lrck_s;
    reg [1:0] sdata_s;
    reg bclk_d;
    reg lrck_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bclk_s <= 2'b00;
            lrck_s <= 2'b00;
            sdata_s <= 2'b00;
            bclk_d <= 1'b0;
            lrck_d <= 1'b0;
        end else begin
            bclk_s <= {bclk_s[0], bclk_in};
            lrck_s <= {lrck_s[0], lrck_in};
            sdata_s <= {sdata_s[0], sdata_in};
            bclk_d <= bclk_s[1];
            lrck_d <= lrck_s[1];
        end
    end

    wire bclk_rise = bclk_s[1] & ~bclk_d;
    wire lrck_edge = lrck_s[1] ^ lrck_d;
    wire lrck_now  = lrck_s[1];
    wire sdata_now = sdata_s[1];

    reg [31:0] shift;     // bits after the straddle bit of current slot
    reg [5:0]  cnt;       // valid bits in shift
    reg [31:0] pending;   // shift content at the last LRCK edge
    reg [5:0]  pend_cnt;
    reg        pend_is_l; // edge 0->1 ended the L slot
    reg        have_pending;

    // pending holds r1..rN of the ended slot (N = pend_cnt, top zero);
    // pending[31] is the previous word's LSB (Philips straddle) and is
    // dropped via [30:0]; the appended bit is the first rise of the new
    // slot = the current word's LSB. shift itself was cleared at the
    // edge and must NOT be used here.
    wire [31:0] concat_w = {pending[30:0], sdata_now};
    wire [5:0]  tot = (pend_cnt == 6'd32) ? 6'd32 : pend_cnt + 6'd1;
    wire [31:0] aligned = concat_w << (6'd32 - tot);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift <= 32'd0;
            cnt <= 6'd0;
            pending <= 32'd0;
            pend_cnt <= 6'd0;
            pend_is_l <= 1'b0;
            have_pending <= 1'b0;
            out_valid <= 1'b0;
            out_l <= 24'sd0;
            out_r <= 24'sd0;
        end else begin
            out_valid <= 1'b0;
            if (lrck_edge) begin
                // slot ended: park it, complete on the next BCLK rise
                pending <= shift;
                pend_cnt <= cnt;
                pend_is_l <= lrck_now; // now high -> L just finished
                have_pending <= 1'b1;
                shift <= 32'd0;
                cnt <= 6'd0;
            end else if (bclk_rise) begin
                if (have_pending) begin
                    // straddle bit arrived: word is complete
                    have_pending <= 1'b0;
                    if (tot >= 6'd16) begin
                        if (pend_is_l)
                            out_l <= aligned[31:8];
                        else begin
                            out_r <= aligned[31:8];
                            out_valid <= 1'b1;
                        end
                    end else begin
                        // runt slot: emit zeros to keep the pair cadence
                        if (pend_is_l)
                            out_l <= 24'sd0;
                        else begin
                            out_r <= 24'sd0;
                            out_valid <= 1'b1;
                        end
                    end
                    shift <= 32'd0;
                    cnt <= 6'd0;
                end else begin
                    shift <= {shift[30:0], sdata_now};
                    if (cnt < 6'd32)
                        cnt <= cnt + 6'd1;
                end
            end
        end
    end
endmodule
