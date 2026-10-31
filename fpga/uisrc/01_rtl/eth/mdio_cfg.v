// =============================================================================
//  mdio_cfg - broadcast-write the YT8531 RGMII RX delay through MDIO
//
//  The PHY address is strap dependent, so the same extended-register write is
//  sent to every address 0..31; only the real PHY latches it.
//
//  For each address:  write 0x1E = 0xA003   (select RGMII_Config1)
//                     write 0x1F = I_val   (Rx_delay_sel in [13:10])
//
//  Triggered by a one-cycle pulse on I_start; O_done pulses when finished.
// =============================================================================
`timescale 1ns / 1ps

module mdio_cfg (
    input  wire        clk,          // 25 MHz
    input  wire        rst,
    input  wire        I_start,
    input  wire [15:0] I_val,
    output reg         mdc,
    output reg         mdio_out,
    output reg         mdio_oe,
    output reg         done
);

    localparam [2:0] S_IDLE  = 3'd0,
                     S_DRIVE = 3'd1,
                     S_NEXT  = 3'd2,
                     S_FIN   = 3'd3;

    reg [2:0]  st;
    reg [5:0]  half;
    reg [6:0]  bit_idx;
    reg [63:0] sh;
    reg [4:0]  phyad;
    reg        phase;       // 0 -> 0x1E, 1 -> 0x1F
    reg        busy;
    reg [15:0] val;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st <= S_IDLE; half <= 0; bit_idx <= 0; sh <= 0;
            mdc <= 1'b0; mdio_out <= 1'b1; mdio_oe <= 1'b0;
            phyad <= 5'd0; phase <= 1'b0; done <= 1'b0; busy <= 1'b0;
            val <= 16'h00F1;
        end else begin
            done <= 1'b0;

            if (!busy && I_start) begin
                busy <= 1'b1;
                val  <= I_val;
                phyad <= 5'd0; phase <= 1'b0;
                bit_idx <= 7'd0; half <= 6'd0;
                sh <= {32'hFFFFFFFF, 2'b01, 2'b01, 5'd0, 5'h1E, 2'b10, 16'hA003};
                st <= S_DRIVE;
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
                        if (bit_idx == 7'd63) st <= S_NEXT;
                    end else half <= half + 6'd1;
                end

                S_NEXT: begin
                    mdio_oe <= 1'b0;
                    if (phase == 1'b0) begin
                        phase <= 1'b1;
                        sh <= {32'hFFFFFFFF, 2'b01, 2'b01, phyad, 5'h1F, 2'b10, val};
                        bit_idx <= 7'd0; half <= 6'd0;
                        st <= S_DRIVE;
                    end else if (phyad == 5'd31) begin
                        busy <= 1'b0; done <= 1'b1;
                        st <= S_FIN;
                    end else begin
                        phyad <= phyad + 5'd1;
                        phase <= 1'b0;
                        sh <= {32'hFFFFFFFF, 2'b01, 2'b01, phyad + 5'd1,
                               5'h1E, 2'b10, 16'hA003};
                        bit_idx <= 7'd0; half <= 6'd0;
                        st <= S_DRIVE;
                    end
                end

                S_FIN: begin
                    mdc <= 1'b0; mdio_oe <= 1'b0;
                    if (!busy && I_start) st <= S_IDLE;
                end

                default: st <= S_FIN;
            endcase
        end
    end

endmodule
