`timescale 1ns/1ps
// Behavioral SDPB (GW5A 16-Kbit Block SRAM) model for the Icarus/Verilator
// simulation flow only. Faithful to Gowin's official prim_sim.v model
// (GW5A library, v1.9.10): byte-lane write enables on ADA[3:0] in 32-bit
// mode, one-cycle synchronous read in READ_MODE=0 (bypass).
// Hardware builds must use the Gowin IDE primitive library instead.
module SDPB #(
    parameter READ_MODE = 1'b0,  // 0: bypass, 1: pipeline
    parameter BIT_WIDTH_0 = 32,
    parameter BIT_WIDTH_1 = 32,
    parameter BLK_SEL_0 = 3'b000,
    parameter BLK_SEL_1 = 3'b000,
    parameter RESET_MODE = "SYNC",
    parameter [255:0] INIT_RAM_00 = 256'h0,
    parameter [255:0] INIT_RAM_01 = 256'h0,
    parameter [255:0] INIT_RAM_02 = 256'h0,
    parameter [255:0] INIT_RAM_03 = 256'h0,
    parameter [255:0] INIT_RAM_04 = 256'h0,
    parameter [255:0] INIT_RAM_05 = 256'h0,
    parameter [255:0] INIT_RAM_06 = 256'h0,
    parameter [255:0] INIT_RAM_07 = 256'h0,
    parameter [255:0] INIT_RAM_08 = 256'h0,
    parameter [255:0] INIT_RAM_09 = 256'h0,
    parameter [255:0] INIT_RAM_0A = 256'h0,
    parameter [255:0] INIT_RAM_0B = 256'h0,
    parameter [255:0] INIT_RAM_0C = 256'h0,
    parameter [255:0] INIT_RAM_0D = 256'h0,
    parameter [255:0] INIT_RAM_0E = 256'h0,
    parameter [255:0] INIT_RAM_0F = 256'h0,
    parameter [255:0] INIT_RAM_10 = 256'h0,
    parameter [255:0] INIT_RAM_11 = 256'h0,
    parameter [255:0] INIT_RAM_12 = 256'h0,
    parameter [255:0] INIT_RAM_13 = 256'h0,
    parameter [255:0] INIT_RAM_14 = 256'h0,
    parameter [255:0] INIT_RAM_15 = 256'h0,
    parameter [255:0] INIT_RAM_16 = 256'h0,
    parameter [255:0] INIT_RAM_17 = 256'h0,
    parameter [255:0] INIT_RAM_18 = 256'h0,
    parameter [255:0] INIT_RAM_19 = 256'h0,
    parameter [255:0] INIT_RAM_1A = 256'h0,
    parameter [255:0] INIT_RAM_1B = 256'h0,
    parameter [255:0] INIT_RAM_1C = 256'h0,
    parameter [255:0] INIT_RAM_1D = 256'h0,
    parameter [255:0] INIT_RAM_1E = 256'h0,
    parameter [255:0] INIT_RAM_1F = 256'h0,
    parameter [255:0] INIT_RAM_20 = 256'h0,
    parameter [255:0] INIT_RAM_21 = 256'h0,
    parameter [255:0] INIT_RAM_22 = 256'h0,
    parameter [255:0] INIT_RAM_23 = 256'h0,
    parameter [255:0] INIT_RAM_24 = 256'h0,
    parameter [255:0] INIT_RAM_25 = 256'h0,
    parameter [255:0] INIT_RAM_26 = 256'h0,
    parameter [255:0] INIT_RAM_27 = 256'h0,
    parameter [255:0] INIT_RAM_28 = 256'h0,
    parameter [255:0] INIT_RAM_29 = 256'h0,
    parameter [255:0] INIT_RAM_2A = 256'h0,
    parameter [255:0] INIT_RAM_2B = 256'h0,
    parameter [255:0] INIT_RAM_2C = 256'h0,
    parameter [255:0] INIT_RAM_2D = 256'h0,
    parameter [255:0] INIT_RAM_2E = 256'h0,
    parameter [255:0] INIT_RAM_2F = 256'h0,
    parameter [255:0] INIT_RAM_30 = 256'h0,
    parameter [255:0] INIT_RAM_31 = 256'h0,
    parameter [255:0] INIT_RAM_32 = 256'h0,
    parameter [255:0] INIT_RAM_33 = 256'h0,
    parameter [255:0] INIT_RAM_34 = 256'h0,
    parameter [255:0] INIT_RAM_35 = 256'h0,
    parameter [255:0] INIT_RAM_36 = 256'h0,
    parameter [255:0] INIT_RAM_37 = 256'h0,
    parameter [255:0] INIT_RAM_38 = 256'h0,
    parameter [255:0] INIT_RAM_39 = 256'h0,
    parameter [255:0] INIT_RAM_3A = 256'h0,
    parameter [255:0] INIT_RAM_3B = 256'h0,
    parameter [255:0] INIT_RAM_3C = 256'h0,
    parameter [255:0] INIT_RAM_3D = 256'h0,
    parameter [255:0] INIT_RAM_3E = 256'h0,
    parameter [255:0] INIT_RAM_3F = 256'h0
)(
    output wire [31:0] DO,
    input  wire [31:0] DI,
    input  wire [2:0]  BLKSELA,
    input  wire [2:0]  BLKSELB,
    input  wire [13:0] ADA,
    input  wire [13:0] ADB,
    input  wire        CLKA,
    input  wire        CLKB,
    input  wire        CEA,
    input  wire        CEB,
    input  wire        OCE,
    input  wire        RESET
);
    reg [31:0] mem [0:511];
    reg [31:0] bp_reg;
    reg [31:0] pl_reg;
    reg [31:0] DO_reg;
    integer i;

    initial begin
        // Power-on content from the INIT_RAM_xx parameters (zero by
        // default). Official Gowin convention (prim_sim.v ram_MEM):
        // INIT_RAM_00[31:0] = word 0, [63:32] = word 1, ...
        begin : init_load
            integer p, w;
            reg [255:0] param;
            for (p = 0; p < 64; p = p + 1) begin
                case (p)
                    0: param = INIT_RAM_00;
                    1: param = INIT_RAM_01;
                    2: param = INIT_RAM_02;
                    3: param = INIT_RAM_03;
                    4: param = INIT_RAM_04;
                    5: param = INIT_RAM_05;
                    6: param = INIT_RAM_06;
                    7: param = INIT_RAM_07;
                    8: param = INIT_RAM_08;
                    9: param = INIT_RAM_09;
                    10: param = INIT_RAM_0A;
                    11: param = INIT_RAM_0B;
                    12: param = INIT_RAM_0C;
                    13: param = INIT_RAM_0D;
                    14: param = INIT_RAM_0E;
                    15: param = INIT_RAM_0F;
                    16: param = INIT_RAM_10;
                    17: param = INIT_RAM_11;
                    18: param = INIT_RAM_12;
                    19: param = INIT_RAM_13;
                    20: param = INIT_RAM_14;
                    21: param = INIT_RAM_15;
                    22: param = INIT_RAM_16;
                    23: param = INIT_RAM_17;
                    24: param = INIT_RAM_18;
                    25: param = INIT_RAM_19;
                    26: param = INIT_RAM_1A;
                    27: param = INIT_RAM_1B;
                    28: param = INIT_RAM_1C;
                    29: param = INIT_RAM_1D;
                    30: param = INIT_RAM_1E;
                    31: param = INIT_RAM_1F;
                    32: param = INIT_RAM_20;
                    33: param = INIT_RAM_21;
                    34: param = INIT_RAM_22;
                    35: param = INIT_RAM_23;
                    36: param = INIT_RAM_24;
                    37: param = INIT_RAM_25;
                    38: param = INIT_RAM_26;
                    39: param = INIT_RAM_27;
                    40: param = INIT_RAM_28;
                    41: param = INIT_RAM_29;
                    42: param = INIT_RAM_2A;
                    43: param = INIT_RAM_2B;
                    44: param = INIT_RAM_2C;
                    45: param = INIT_RAM_2D;
                    46: param = INIT_RAM_2E;
                    47: param = INIT_RAM_2F;
                    48: param = INIT_RAM_30;
                    49: param = INIT_RAM_31;
                    50: param = INIT_RAM_32;
                    51: param = INIT_RAM_33;
                    52: param = INIT_RAM_34;
                    53: param = INIT_RAM_35;
                    54: param = INIT_RAM_36;
                    55: param = INIT_RAM_37;
                    56: param = INIT_RAM_38;
                    57: param = INIT_RAM_39;
                    58: param = INIT_RAM_3A;
                    59: param = INIT_RAM_3B;
                    60: param = INIT_RAM_3C;
                    61: param = INIT_RAM_3D;
                    62: param = INIT_RAM_3E;
                    63: param = INIT_RAM_3F;
                    default: param = 256'h0;
                endcase
                for (w = 0; w < 8; w = w + 1)
                    mem[p * 8 + w] = param[31 + w * 32 -: 32];
            end
        end
        bp_reg = 32'd0;
        pl_reg = 32'd0;
    end

    wire       write_active = CEA && (BLKSELA == BLK_SEL_0);
    wire       read_active  = CEB && (BLKSELB == BLK_SEL_1);
    wire [8:0] waddr = ADA[13:5];
    wire [8:0] raddr = ADB[13:5];
    wire [3:0] byte_en = ADA[3:0];

    always @(posedge CLKA) begin
        if (write_active) begin
            if (byte_en[0]) mem[waddr][7:0]   <= DI[7:0];
            if (byte_en[1]) mem[waddr][15:8]  <= DI[15:8];
            if (byte_en[2]) mem[waddr][23:16] <= DI[23:16];
            if (byte_en[3]) mem[waddr][31:24] <= DI[31:24];
        end
    end

    always @(posedge CLKB) begin
        if (RESET && (RESET_MODE == "SYNC")) begin
            bp_reg <= 32'd0;
            pl_reg <= 32'd0;
        end else begin
            if (OCE)
                pl_reg <= bp_reg;
            if (read_active)
                bp_reg <= mem[raddr];
        end
    end

    always @* begin
        DO_reg = (READ_MODE == 1'b0) ? bp_reg : pl_reg;
    end
    assign DO = DO_reg;

endmodule
