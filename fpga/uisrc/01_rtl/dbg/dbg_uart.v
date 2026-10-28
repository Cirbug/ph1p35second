//==========================================================================
// dbg_uart.v  （精简诊断版）
//   每 ~100ms 打印一行，115200 8N1：
//     C=1 W=0960 D=2B M=04C7 H0=2B09 H1=6A3E F=01D2 V=9AB3 X=0
//
//   W  = 解析出的 WC          D  = 解析出的 DT
//   M  = 最近一次 HS 突发的拍数
//   H0 = valid 上升沿那一拍的数据（应为 {DI, WC_LSB}）
//   H1 = 紧接着那一拍的数据（应为 {WC_MSB, ECC}）
//   F  = 发送侧帧计数         V = RX 有效字节计数
//==========================================================================
module dbg_uart #(
    parameter CLK_HZ = 24000000,
    parameter BAUD   = 115200,
    parameter GAP_MS = 100
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_st_cfg,
    input  wire [15:0] I_st_wc,
    input  wire [7:0]  I_st_dt,
    input  wire [15:0] I_st_burst,
    input  wire [15:0] I_st_h0,
    input  wire [15:0] I_st_h1,
    input  wire [15:0] I_st_frame,
    input  wire [15:0] I_st_rxval,
    input  wire        I_st_werr,
    input  wire [15:0] I_st_gapmax,
    input  wire [15:0] I_st_pktcnt,

    output reg         O_uart_tx
);

    localparam DIV     = CLK_HZ / BAUD;
    localparam MS_CNT  = CLK_HZ / 1000;
    localparam GAP_CNT = MS_CNT * GAP_MS;
    localparam NBYTE   = 72;

    function [7:0] hx;
        input [3:0] n;
        begin
            hx = (n < 4'd10) ? (8'd48 + {4'd0, n}) : (8'd55 + {4'd0, n});
        end
    endfunction

    reg [7:0] idx;
    reg [7:0] msg_byte;

    always @(*) begin
        case (idx)
        8'd0 : msg_byte = 8'h43;                       // C
        8'd1 : msg_byte = 8'h3D;
        8'd2 : msg_byte = 8'd48 + { 7'd0, I_st_cfg };
        8'd3 : msg_byte = 8'h20;
        8'd4 : msg_byte = 8'h57;                       // W
        8'd5 : msg_byte = 8'h3D;
        8'd6 : msg_byte = hx(I_st_wc[15:12]);
        8'd7 : msg_byte = hx(I_st_wc[11:8]);
        8'd8 : msg_byte = hx(I_st_wc[7:4]);
        8'd9 : msg_byte = hx(I_st_wc[3:0]);
        8'd10: msg_byte = 8'h20;
        8'd11: msg_byte = 8'h44;                       // D
        8'd12: msg_byte = 8'h3D;
        8'd13: msg_byte = hx(I_st_dt[7:4]);
        8'd14: msg_byte = hx(I_st_dt[3:0]);
        8'd15: msg_byte = 8'h20;
        8'd16: msg_byte = 8'h4D;                       // M
        8'd17: msg_byte = 8'h3D;
        8'd18: msg_byte = hx(I_st_burst[15:12]);
        8'd19: msg_byte = hx(I_st_burst[11:8]);
        8'd20: msg_byte = hx(I_st_burst[7:4]);
        8'd21: msg_byte = hx(I_st_burst[3:0]);
        8'd22: msg_byte = 8'h20;
        8'd23: msg_byte = 8'h48;                       // H0
        8'd24: msg_byte = 8'h30;
        8'd25: msg_byte = 8'h3D;
        8'd26: msg_byte = hx(I_st_h0[15:12]);
        8'd27: msg_byte = hx(I_st_h0[11:8]);
        8'd28: msg_byte = hx(I_st_h0[7:4]);
        8'd29: msg_byte = hx(I_st_h0[3:0]);
        8'd30: msg_byte = 8'h20;
        8'd31: msg_byte = 8'h48;                       // H1
        8'd32: msg_byte = 8'h31;
        8'd33: msg_byte = 8'h3D;
        8'd34: msg_byte = hx(I_st_h1[15:12]);
        8'd35: msg_byte = hx(I_st_h1[11:8]);
        8'd36: msg_byte = hx(I_st_h1[7:4]);
        8'd37: msg_byte = hx(I_st_h1[3:0]);
        8'd38: msg_byte = 8'h20;
        8'd39: msg_byte = 8'h46;                       // F
        8'd40: msg_byte = 8'h3D;
        8'd41: msg_byte = hx(I_st_frame[15:12]);
        8'd42: msg_byte = hx(I_st_frame[11:8]);
        8'd43: msg_byte = hx(I_st_frame[7:4]);
        8'd44: msg_byte = hx(I_st_frame[3:0]);
        8'd45: msg_byte = 8'h20;
        8'd46: msg_byte = 8'h56;                       // V
        8'd47: msg_byte = 8'h3D;
        8'd48: msg_byte = hx(I_st_rxval[15:12]);
        8'd49: msg_byte = hx(I_st_rxval[11:8]);
        8'd50: msg_byte = hx(I_st_rxval[7:4]);
        8'd51: msg_byte = hx(I_st_rxval[3:0]);
        8'd52: msg_byte = 8'h20;
        8'd53: msg_byte = 8'h58;                       // X
        8'd54: msg_byte = 8'h3D;
        8'd55: msg_byte = 8'd48 + { 7'd0, I_st_werr };
        8'd56: msg_byte = 8'h20;
        8'd57: msg_byte = 8'h41;   // A = DT==0x00 的包计数
        8'd58: msg_byte = 8'h3D;
        8'd59: msg_byte = hx(I_st_gapmax[15:12]);
        8'd60: msg_byte = hx(I_st_gapmax[11:8]);
        8'd61: msg_byte = hx(I_st_gapmax[7:4]);
        8'd62: msg_byte = hx(I_st_gapmax[3:0]);
        8'd63: msg_byte = 8'h20;
        8'd64: msg_byte = 8'h54;   // T = TX 实际发出的拍数
        8'd65: msg_byte = 8'h3D;
        8'd66: msg_byte = hx(I_st_pktcnt[15:12]);
        8'd67: msg_byte = hx(I_st_pktcnt[11:8]);
        8'd68: msg_byte = hx(I_st_pktcnt[7:4]);
        8'd69: msg_byte = hx(I_st_pktcnt[3:0]);
        8'd70: msg_byte = 8'h0D;
        8'd71: msg_byte = 8'h0A;
        default: msg_byte = 8'h20;
        endcase
    end

    localparam S_GAP   = 2'd0;
    localparam S_LOAD  = 2'd1;
    localparam S_SHIFT = 2'd2;
    localparam S_NEXT  = 2'd3;

    reg [1:0]  s;
    reg [23:0] gap_cnt;
    reg [15:0] div_cnt;
    reg [3:0]  bitn;
    reg [9:0]  shft;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s <= S_GAP; gap_cnt <= 24'd0; div_cnt <= 16'd0;
            bitn <= 4'd0; shft <= 10'h3FF; idx <= 8'd0; O_uart_tx <= 1'b1;
        end else begin
            case (s)
            S_GAP: begin
                O_uart_tx <= 1'b1;
                if (gap_cnt >= GAP_CNT[23:0]) begin
                    gap_cnt <= 24'd0; idx <= 8'd0; s <= S_LOAD;
                end else gap_cnt <= gap_cnt + 24'd1;
            end
            S_LOAD: begin
                shft <= { 1'b1, msg_byte, 1'b0 };
                bitn <= 4'd0; div_cnt <= 16'd0;
                O_uart_tx <= 1'b0; s <= S_SHIFT;
            end
            S_SHIFT: begin
                if (div_cnt >= (DIV - 1)) begin
                    div_cnt <= 16'd0;
                    if (bitn == 4'd9) begin O_uart_tx <= 1'b1; s <= S_NEXT; end
                    else begin
                        bitn <= bitn + 4'd1;
                        shft <= { 1'b1, shft[9:1] };
                        O_uart_tx <= shft[1];
                    end
                end else div_cnt <= div_cnt + 16'd1;
            end
            S_NEXT: begin
                if (idx >= (NBYTE - 1)) s <= S_GAP;
                else begin idx <= idx + 8'd1; s <= S_LOAD; end
            end
            default: s <= S_GAP;
            endcase
        end
    end

endmodule
