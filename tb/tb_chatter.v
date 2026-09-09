`timescale 1ns/1ps
// dsd_on chatter regression for the DB_BITS debouncer: sub-window
// chatter must never disturb STEADY PCM (no mux flip, music keeps
// flowing), while a stable edge still switches modes.
module tb_chatter;
    reg mclk = 0;
    always #10 mclk = ~mclk;

    // X1-grid I2S master (same as tb_swap_diag).
    reg bclk = 0;
    always #40 bclk = ~bclk;
    reg [5:0] bpos = 6'd0;
    always @(posedge bclk) bpos <= (bpos == 6'd63) ? 6'd0 : bpos + 6'd1;
    reg lrck = 0, sdata = 0;
    reg [31:0] sh = 32'd0;
    always @(negedge bclk) begin
        if (bpos == 6'd63) begin lrck <= 1'b0; sh <= {24'h400000, 8'd0}; end
        else if (bpos == 6'd31) begin lrck <= 1'b1; sh <= {24'hC00000, 8'd0}; end
        else sh <= {sh[30:0], 1'b0};
        sdata <= sh[31];
    end

    reg dsd_on = 1'b0;
    wire b0, l0, d0;
    top #(.DB_BITS(6)) uut (
        .mclk_in(mclk), .i2s_bclk_in(bclk), .i2s_lrck_in(lrck),
        .i2s_sdata_in(sdata), .dsd_on(dsd_on), .dsd_data2_in(1'b0),
        .i2s_bclk_out(b0), .i2s_lrck_out(l0), .i2s_sdata_out(d0)
    );

    task automatic mclks;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge mclk);
        end
    endtask

    // capture one L/R pair from the output grid
    reg [15:0] capL, capR;
    task automatic cap_pair;
        integer i;
        begin
            @(negedge l0);
            @(posedge b0);
            capL = 0;
            for (i = 0; i < 16; i = i + 1) begin @(posedge b0); #1; capL = {capL[14:0], d0}; end
            @(posedge l0);
            @(posedge b0);
            capR = 0;
            for (i = 0; i < 16; i = i + 1) begin @(posedge b0); #1; capR = {capR[14:0], d0}; end
        end
    endtask

    task automatic chatter;
        input integer rounds;
        integer i;
        begin
            // sub-window toggles (window = 64 mclk): 20 high / 30 low.
            for (i = 0; i < rounds; i = i + 1) begin
                dsd_on = 1'b1; mclks(20);
                dsd_on = 1'b0; mclks(30);
            end
        end
    endtask

    integer bad;
    initial begin
        bad = 0;
        #11000000; // POR + HOLD + IN -> STEADY PCM
        if (uut.sw_state !== 2'd0 || uut.mux_frozen !== 1'b0) begin
            bad = 1; $display("FAIL no STEADY PCM st=%d mux=%b", uut.sw_state, uut.mux_frozen);
        end
        // 1. chatter around 0: PCM must not flinch.
        chatter(40); // 2000 mclk of sub-window bouncing
        if (uut.sw_state !== 2'd0 || uut.mux_frozen !== 1'b0) begin
            bad = 1; $display("FAIL chatter broke STEADY st=%d mux=%b", uut.sw_state, uut.mux_frozen);
        end
        cap_pair();
        $display("during chatter L=%04h R=%04h", capL, capR);
        if (capL[15:4] == 12'd0 || capR[15:4] == 12'd0 || capL == capR) begin
            bad = 1; $display("FAIL chatter muted/killed music");
        end
        // 2. stable 1: must switch (OUT fade = 128 frames, then mux flips).
        dsd_on = 1'b1;
        mclks(40000);
        if (uut.mux_frozen !== 1'b1) begin
            bad = 1; $display("FAIL stable 1 did not switch mux=%b st=%d", uut.mux_frozen, uut.sw_state);
        end
        // 3. chatter around 1: must stay.
        chatter(40);
        dsd_on = 1'b1; mclks(10);
        if (uut.mux_frozen !== 1'b1) begin
            bad = 1; $display("FAIL chatter around 1 unswitched mux=%b", uut.mux_frozen);
        end
        // 4. stable 0: must return (OUT fade, then mux flips back).
        dsd_on = 1'b0;
        mclks(40000);
        if (uut.mux_frozen !== 1'b0) begin
            bad = 1; $display("FAIL stable 0 did not return mux=%b st=%d", uut.mux_frozen, uut.sw_state);
        end
        if (bad) $display("FAIL tb_chatter");
        else $display("PASS tb_chatter");
        $finish;
    end

    initial begin #16000000; $display("FAIL tb_chatter watchdog"); $finish; end
endmodule
