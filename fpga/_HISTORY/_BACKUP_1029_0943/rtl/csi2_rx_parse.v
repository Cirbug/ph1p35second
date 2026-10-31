//==========================================================================
// csi2_rx_parse.v
//   CSI-2 包解析：抓包头、搬载荷、产出帧/行边界。
//
//   要点：
//     1) 只在 I_hs_valid 的【上升沿】抓包头第 1 拍；
//     2) 状态只在 valid 为高的拍推进，不因 valid 气泡复位；
//     3) 先看类型字节：DT=0x00/0x01 直接判帧边界，不碰其载荷
//        （本传感器发的帧尾包宽度字段非零，若按长包处理会吞掉后续行包）；
//     4) 长包尾部的 CRC16（2 字节 = 1 拍）显式消耗。
//
//   包头字节序（CSI-2 标准，实测与摄像头一致）：DI, WC_LSB, WC_MSB, ECC
//     第 1 拍 = {DI, WC_LSB}
//     第 2 拍 = {WC_MSB, ECC}
//==========================================================================
module csi2_rx_parse #(
    parameter LANE_NUM = 2
)(
    input  wire                   I_clk,
    input  wire                   I_rst,

    input  wire                   I_hs_valid,
    input  wire [LANE_NUM*8-1:0]  I_hs_data,

    output reg                    O_wr_en,
    output reg  [LANE_NUM*8-1:0]  O_wr_data,
    output reg                    O_pkt_end,
    output reg  [15:0]            O_pkt_wc,
    output reg  [7:0]             O_pkt_dt,
    output reg                    O_frame_start,
    output reg                    O_frame_end,
    output reg                    O_wc_err
);

    localparam S_IDLE = 2'd0;
    localparam S_HDR  = 2'd1;
    localparam S_PAY  = 2'd2;
    localparam S_TAIL = 2'd3;

    reg [1:0]  st;
    reg        v_d;
    reg [7:0]  wc_lsb;            // 包头第 1 拍低字节 = WC 低字节
    reg [15:0] pay_cnt;
    reg [15:0] pay_beats;
    reg        end_pend;

    wire [15:0] wc_new = { I_hs_data[15:8], wc_lsb };   // {WC_MSB, WC_LSB}
    wire [15:0] pb_new = (wc_new + 16'd1) >> 1;         // ceil(wc/2)

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            st            <= S_IDLE;
            v_d           <= 1'b0;
            wc_lsb        <= 8'd0;
            pay_cnt       <= 16'd0;
            pay_beats     <= 16'd0;
            end_pend      <= 1'b0;
            O_wr_en       <= 1'b0;
            O_wr_data     <= {LANE_NUM*8{1'b0}};
            O_pkt_end     <= 1'b0;
            O_pkt_wc      <= 16'd0;
            O_pkt_dt      <= 8'd0;
            O_frame_start <= 1'b0;
            O_frame_end   <= 1'b0;
            O_wc_err      <= 1'b0;
        end else begin
            O_wr_en       <= 1'b0;
            O_pkt_end     <= end_pend;
            end_pend      <= 1'b0;
            O_frame_start <= 1'b0;
            O_frame_end   <= 1'b0;

            v_d <= I_hs_valid;

            if (I_hs_valid) begin
                case (st)
                //---------------- 包头第 1 拍：{DI, WC_LSB} ----------------
                S_IDLE: begin
                    if (!v_d) begin
                        O_pkt_dt <= I_hs_data[15:8];
                        wc_lsb   <= I_hs_data[7:0];
                        st       <= S_HDR;
                    end
                end

                //---------------- 包头第 2 拍：{WC_MSB, ECC} --------------
                S_HDR: begin
                    O_pkt_wc  <= wc_new;
                    pay_beats <= pb_new;
                    pay_cnt   <= 16'd0;

                    if (O_pkt_dt[5:0] == 6'h00) begin
                        O_frame_start <= 1'b1;
                        st <= S_IDLE;
                    end else if (O_pkt_dt[5:0] == 6'h01) begin
                        O_frame_end   <= 1'b1;
                        st <= S_IDLE;
                    end else if (wc_new == 16'd0) begin
                        st <= S_IDLE;
                    end else begin
                        if (wc_new[0]) O_wc_err <= 1'b1;
                        st <= S_PAY;
                    end
                end

                //---------------- 载荷 ----------------
                S_PAY: begin
                    O_wr_en   <= 1'b1;
                    O_wr_data <= I_hs_data;

                    if (pay_cnt == (pay_beats - 16'd1)) begin
                        end_pend <= 1'b1;
                        st       <= S_TAIL;
                    end else begin
                        pay_cnt  <= pay_cnt + 16'd1;
                    end
                end

                //---------------- 吃掉 CRC 那一拍 ----------------
                S_TAIL: begin
                    st <= S_IDLE;
                end

                default: st <= S_IDLE;
                endcase
            end
        end
    end

endmodule
