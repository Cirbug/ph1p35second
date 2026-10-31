// =============================================================================
//  mdio_scan - read PHY ID (register 2) from addresses 0..31 to find the PHY,
//              then read EXT_0xA003 (via 0x1E/0x1F) to verify the delay value.
//
//  Clause 22 read frame:
//      32 x 1  + 01 (ST) + 10 (OP read) + PHYAD[4:0] + REGAD[4:0]
//      + 2 bit turnaround (bus released) + 16 bit data (MSB first)
//
//  MDC = clk/50 (500 kHz at 25 MHz).
// =============================================================================
`timescale 1ns / 1ps

module mdio_scan (
    input  wire        clk,        // 25 MHz
    input  wire        rst,
    output reg         mdc,
    output reg         mdio_out,
    output reg         mdio_oe,    // 0 = 高阻（读回时）
    input  wire        mdio_in,
    output reg  [4:0]  phy_addr,   // 找到的 PHY 地址
    output reg  [15:0] phy_id,
    output reg  [15:0] rx_dly_reg, // 0xA003 读回值
    output reg         done
);

    reg [5:0]  div;
    reg [2:0]  st;
    reg [5:0]  bit_idx;         // 0..63
    reg [63:0] sh;
    reg [15:0] rxdata;
    reg [4:0]  scan;

    localparam ST_IDLE  = 3'd0,
               ST_LOAD  = 3'd1,
               ST_SEND  = 3'd2,
               ST_READ  = 3'd3,
               ST_NEXT  = 3'd4,
               ST_DLYW  = 3'd5,   // 写 0x1E = 0xA003
               ST_DLYR  = 3'd6,   // 读 0x1F
               ST_FIN   = 3'd7;

    // 一帧读：32x1 + 01 + 10 + PHYAD + REGAD + (2 拍空闲) + 16 拍数据
    function [63:0] rd_frame;
        input [4:0] pa;
        input [4:0] ra;
        begin
            rd_frame = {32'hFFFFFFFF, 2'b01, 2'b10, pa, ra, 2'b00, 16'h0000};
        end
    endfunction

    // 一帧写：32x1 + 01 + 01 + PHYAD + REGAD + 10 + DATA
    function [63:0] wr_frame;
        input [4:0] pa;
        input [4:0] ra;
        input [15:0] d;
        begin
            wr_frame = {32'hFFFFFFFF, 2'b01, 2'b01, pa, ra, 2'b10, d};
        end
    endfunction

    localparam [4:0] REG_ID  = 5'd2;
    localparam [4:0] REG_1E  = 5'h1E;
    localparam [4:0] REG_1F  = 5'h1F;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            div <= 0; st <= ST_IDLE; bit_idx <= 0; sh <= 0;
            mdc <= 1'b0; mdio_out <= 1'b1; mdio_oe <= 1'b0;
            phy_addr <= 5'h1F; phy_id <= 16'hFFFF; rx_dly_reg <= 16'h0000;
            done <= 1'b0; scan <= 5'd0; rxdata <= 16'h0;
        end else begin
            case (st)
                ST_IDLE: begin
                    scan <= 5'd0; done <= 1'b0;
                    st <= ST_LOAD;
                end

                ST_LOAD: begin
                    sh <= rd_frame(scan, REG_ID);
                    bit_idx <= 6'd0; div <= 6'd0; st <= ST_SEND;
                end

                ST_SEND: begin
                    if (bit_idx < 6'd48) begin
                        // 前 48 位：全部由 FPGA 驱动
                        mdio_oe <= 1'b1;
                        if (div == 6'd0) begin
                            mdio_out <= sh[63]; mdc <= 1'b0; div <= 6'd1;
                        end else if (div == 6'd24) begin
                            mdc <= 1'b1; div <= 6'd25;
                        end else if (div == 6'd49) begin
                            mdc <= 1'b0; div <= 6'd0; sh <= {sh[62:0], 1'b0};
                            bit_idx <= bit_idx + 6'd1;
                        end else div <= div + 6'd1;
                    end else begin
                        // 48..49 拍周转：释放总线
                        mdio_oe <= 1'b0;
                        if (div == 6'd49) begin
                            mdc <= 1'b0; div <= 6'd0;
                            bit_idx <= bit_idx + 6'd1;
                            if (bit_idx == 6'd49) st <= ST_READ;
                        end else if (div == 6'd24) begin
                            mdc <= 1'b1; div <= 6'd25;
                        end else div <= div + 6'd1;
                    end
                end

                ST_READ: begin
                    // 16 拍数据，上升沿后采样
                    mdio_oe <= 1'b0;
                    if (div == 6'd0) begin mdc <= 1'b0; div <= 6'd1; end
                    else if (div == 6'd24) begin
                        mdc <= 1'b1; div <= 6'd25;
                    end else if (div == 6'd40) begin
                        rxdata <= {rxdata[14:0], mdio_in};
                        div <= 6'd41;
                    end else if (div == 6'd49) begin
                        mdc <= 1'b0; div <= 6'd0;
                        bit_idx <= bit_idx + 6'd1;
                        if (bit_idx == 6'd65) st <= ST_NEXT;
                    end else div <= div + 6'd1;
                end

                ST_NEXT: begin
                    if (rxdata != 16'hFFFF && rxdata != 16'h0000 && phy_addr == 5'h1F) begin
                        phy_addr <= scan;
                        phy_id   <= rxdata;
                    end
                    if (scan == 5'd31) begin
                        // 用找到的地址写/读 0xA003
                        if (phy_addr != 5'h1F) begin
                            sh <= wr_frame(phy_addr, REG_1E, 16'hA003);
                            bit_idx <= 6'd0; div <= 6'd0; st <= ST_DLYW;
                        end else st <= ST_FIN;
                    end else begin
                        scan <= scan + 5'd1;
                        st <= ST_LOAD;
                    end
                end

                ST_DLYW: begin   // 写 0x1E（同写帧流程）
                    mdio_oe <= 1'b1;
                    if (bit_idx < 6'd62) begin
                        if (div == 6'd0) begin mdio_out <= sh[63]; mdc <= 1'b0; div <= 6'd1; end
                        else if (div == 6'd24) begin mdc <= 1'b1; div <= 6'd25; end
                        else if (div == 6'd49) begin
                            mdc <= 1'b0; div <= 6'd0; sh <= {sh[62:0], 1'b0};
                            bit_idx <= bit_idx + 6'd1;
                        end else div <= div + 6'd1;
                    end else begin
                        mdio_oe <= 1'b0;
                        sh <= rd_frame(phy_addr, REG_1F);
                        bit_idx <= 6'd0; div <= 6'd0; st <= ST_SEND;
                        // 注意：ST_SEND 会在读完后回到 ST_NEXT，用 scan==31 判断
                        done <= 1'b0;
                    end
                end

                ST_FIN: begin
                    mdc <= 1'b0; mdio_oe <= 1'b0; done <= 1'b1;
                end

                default: st <= ST_FIN;
            endcase
        end
    end

    // 读到的 0x1F 数据就是 0xA003
    always @(posedge clk) begin
        if (st == ST_READ && bit_idx == 6'd65 && div == 6'd48)
            rx_dly_reg <= {rxdata[14:0], mdio_in};
    end

endmodule
