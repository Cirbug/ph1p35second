// =============================================================================
//  mdio_probe - clause-22 MDIO, re-triggerable
//
//  First run  : scan PHY addresses 0..31 (reg 2) -> latch addr_found / phy_id,
//               then write 0x1E = 0xA003 and read 0x1F -> dly_pre
//  On I_start : write 0x1E = 0xA003, write 0x1F = I_val, read 0x1F -> dly_post
//
//  MDC = clk / 50 (500 kHz from 25 MHz).  Frame = 64 bits, MSB first.
//  Read : drive 46 bits -> release 2 bits (TA) -> sample 16 bits
//  Write: drive 64 bits
// =============================================================================
`timescale 1ns / 1ps

module mdio_probe (
    input  wire        clk,
    input  wire        rst,
    input  wire        I_start,
    input  wire [15:0] I_val,
    output reg         mdc,
    output reg         mdio_out,
    output reg         mdio_oe,
    input  wire        mdio_in,
    output reg  [4:0]  addr_found,
    output reg  [15:0] phy_id,
    output reg  [15:0] dly_pre,
    output reg  [15:0] dly_post,
    output reg         done
);

    localparam [2:0] S_IDLE  = 3'd0,
                     S_DRIVE = 3'd1,
                     S_TURN  = 3'd2,
                     S_READ  = 3'd3,
                     S_NEXT  = 3'd4,
                     S_FIN   = 3'd5;

    reg [2:0]  st;
    reg [5:0]  half;
    reg [6:0]  bit_idx;
    reg [63:0] sh;
    reg [15:0] rxdata;
    reg [4:0]  scan;
    reg [2:0]  step;
    reg        is_read;
    reg        busy;
    reg [15:0] val_lat;
    reg        first_done;

    wire [63:0] wr_1e = {32'hFFFFFFFF, 2'b01, 2'b01, addr_found, 5'h1E, 2'b10, 16'hA003};
    wire [63:0] wr_1f = {32'hFFFFFFFF, 2'b01, 2'b01, addr_found, 5'h1F, 2'b10, val_lat};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= S_IDLE; half <= 0; bit_idx <= 0; sh <= 0;
            mdc <= 1'b0; mdio_out <= 1'b1; mdio_oe <= 1'b0; rxdata <= 0;
            addr_found <= 5'h1F; phy_id <= 16'hFFFF;
            dly_pre <= 16'hFFFF; dly_post <= 16'hFFFF;
            scan <= 5'd0; step <= 3'd0; is_read <= 1'b1; done <= 1'b0;
            busy <= 1'b0; val_lat <= 16'h0000; first_done <= 1'b0;
        end else begin
            if (!busy && I_start) begin
                busy <= 1'b1; done <= 1'b0;
                val_lat <= I_val;
                if (!first_done) begin
                    scan <= 5'd0; step <= 3'd0; is_read <= 1'b1;
                    sh <= {32'hFFFFFFFF, 2'b01, 2'b10, 5'd0, 5'd2, 2'b00, 16'h0000};
                    bit_idx <= 7'd0; half <= 6'd0; st <= S_DRIVE;
                end else begin
                    step <= 3'd1; is_read <= 1'b0;
                    sh <= wr_1e; bit_idx <= 7'd0; half <= 6'd0; st <= S_DRIVE;
                end
            end

            case (st)
                S_DRIVE: begin
                    mdio_oe <= 1'b1;
                    if (half == 6'd0) begin
                        mdc <= 1'b0; mdio_out <= sh[63]; half <= 6'd1;
                    end else if (half == 6'd20) begin
                        mdc <= 1'b1; half <= 6'd21;
                    end else if (half == 6'd49) begin
                        mdc <= 1'b0; half <= 6'd0;
                        sh <= {sh[62:0], 1'b0};
                        bit_idx <= bit_idx + 7'd1;
                        if (is_read) begin
                            if (bit_idx == 7'd45) st <= S_TURN;
                        end else begin
                            if (bit_idx == 7'd63) begin
                                mdio_oe <= 1'b0;
                                st <= S_NEXT;
                            end
                        end
                    end else half <= half + 6'd1;
                end

                S_TURN: begin
                    mdio_oe <= 1'b0;
                    if (half == 6'd0) half <= 6'd1;
                    else if (half == 6'd20) begin mdc <= 1'b1; half <= 6'd21; end
                    else if (half == 6'd49) begin
                        mdc <= 1'b0; half <= 6'd0; bit_idx <= 7'd0;
                        rxdata <= 16'h0000; st <= S_READ;
                    end else half <= half + 6'd1;
                end

                S_READ: begin
                    mdio_oe <= 1'b0;
                    if (half == 6'd0) half <= 6'd1;
                    else if (half == 6'd20) begin mdc <= 1'b1; half <= 6'd21; end
                    else if (half == 6'd30) begin
                        rxdata <= {rxdata[14:0], mdio_in}; half <= 6'd31;
                    end else if (half == 6'd49) begin
                        mdc <= 1'b0; half <= 6'd0;
                        bit_idx <= bit_idx + 7'd1;
                        if (bit_idx == 7'd15) st <= S_NEXT;
                    end else half <= half + 6'd1;
                end

                S_NEXT: begin
                    case (step)
                        3'd0: begin
                            if (rxdata != 16'hFFFF && rxdata != 16'h0000 &&
                                addr_found == 5'h1F) begin
                                addr_found <= scan;
                                phy_id <= rxdata;
                                step <= 3'd1; is_read <= 1'b0;
                                sh <= {32'hFFFFFFFF, 2'b01, 2'b01, scan,
                                       5'h1E, 2'b10, 16'hA003};
                                bit_idx <= 7'd0; half <= 6'd0; st <= S_DRIVE;
                            end else if (scan == 5'd31) begin
                                busy <= 1'b0; done <= 1'b1; st <= S_FIN;
                            end else begin
                                scan <= scan + 5'd1;
                                is_read <= 1'b1;
                                sh <= {32'hFFFFFFFF, 2'b01, 2'b10, scan + 5'd1,
                                       5'd2, 2'b00, 16'h0000};
                                bit_idx <= 7'd0; half <= 6'd0; st <= S_DRIVE;
                            end
                        end
                        3'd1: begin
                            step <= 3'd2; is_read <= 1'b1;
                            sh <= {32'hFFFFFFFF, 2'b01, 2'b10, addr_found,
                                   5'h1F, 2'b00, 16'h0000};
                            bit_idx <= 7'd0; half <= 6'd0; st <= S_DRIVE;
                        end
                        3'd2: begin
                            if (!first_done) dly_pre <= rxdata;
                            step <= 3'd3; is_read <= 1'b0;
                            sh <= wr_1f;
                            bit_idx <= 7'd0; half <= 6'd0; st <= S_DRIVE;
                        end
                        3'd3: begin
                            step <= 3'd4; is_read <= 1'b1;
                            sh <= {32'hFFFFFFFF, 2'b01, 2'b10, addr_found,
                                   5'h1F, 2'b00, 16'h0000};
                            bit_idx <= 7'd0; half <= 6'd0; st <= S_DRIVE;
                        end
                        default: begin
                            dly_post <= rxdata;
                            first_done <= 1'b1;
                            busy <= 1'b0; done <= 1'b1; st <= S_FIN;
                        end
                    endcase
                end

                S_FIN: begin
                    mdc <= 1'b0;
                    mdio_oe <= 1'b0;
                    if (!busy && I_start) st <= S_IDLE;
                end

                default: st <= S_FIN;
            endcase
        end
    end

endmodule
