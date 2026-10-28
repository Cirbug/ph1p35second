//==========================================================================
// csi2_tx_packet.v
//   CSI-2 发送打包器：把"一行载荷"重新封装成标准 CSI-2 包流
//
//   包结构（每个包是一次独立的 HS burst）：
//     帧起 FS : DI(0x00) WC(0x0000) ECC             —— 4 字节短包
//     长包    : DI      WC(高) WC(低) ECC            —— 包头 4 字节
//               载荷 ...                            —— WC 字节
//               CRC(高) CRC(低)                     —— 2 字节
//     帧尾 FE : DI(0x01) WC(0x0000) ECC             —— 4 字节短包
//
//   字节按 lane 轮询分发：byte0->lane0, byte1->lane1, ...
//   一个 beat 的 O_hs_data[L*8-1 -: 8] 是 lane0，即 MSB 侧是第一个字节
//
//   行缓冲：两个 bank 乒乓。写侧收满一行后置 line_ready，发送侧取走。
//
//   ⚠ 约束（v1）：
//     1) LANE_NUM 固定 2（CRC 用 2 字节/拍并行实现）
//     2) I_pkt_wc 必须是 LANE_NUM 的整数倍（RAW10 1024px=1280B、RGB888=3072B
//        都满足；不满足时 O_wc_err 置位，CRC 会算错）
//     3) I_pkt_end 必须在最后一个 I_wr_en 之后一拍给出（此时 wr_crc 已稳定）
//     4) I_hs_ready 在一次 burst 内保持为高（与 PHY 手册一致：只在 last 后拉低）
//==========================================================================
module csi2_tx_packet #(
    parameter LANE_NUM = 2,
    parameter VC       = 2'b00,
    parameter DT       = 8'h2B,      // RAW10
    parameter BUF_AW   = 10,         // 每个 bank 2^BUF_AW 个 word
    parameter LANE_SWAP= 0           // 1 = 反转 lane 顺序（应对排线反向）
)(
    input  wire                    I_clk,
    input  wire                    I_rst,

    // 写侧：来自 csi2_rx_parse 的载荷（每拍一个 word = LANE_NUM 字节）
    input  wire                    I_wr_en,
    input  wire [LANE_NUM*8-1:0]   I_wr_data,
    input  wire                    I_pkt_end,
    input  wire [15:0]             I_pkt_wc,

    // 帧边界
    input  wire                    I_frame_start,
    input  wire                    I_frame_end,

    // PHY 接口
    output wire                    O_hs_en,
    output wire [LANE_NUM*8-1:0]   O_hs_data,
    output wire                    O_hs_last,
    input  wire                    I_hs_ready,

    // 状态
    output reg                     O_overflow,
    output reg                     O_wc_err
);

    localparam L      = LANE_NUM;
    localparam LOG2L  = (L == 4) ? 2 : ((L == 2) ? 1 : 0);
    localparam DEPTH  = 1 << (BUF_AW + 1);
    localparam BANK_W = 1 << BUF_AW;          // 每个 bank 的 word 数

    // 进来这一行的 word 数（= ceil(WC/L)），用于检测是否超过 bank 深度
    wire [15:0] in_words = (I_pkt_wc + (L - 1)) >> LOG2L;

    //---------------- 行缓冲 ----------------
    reg [L*8-1:0]    mem [0:DEPTH-1];
    reg              wr_bank;      // 正在写入的 bank
    reg              rdy_bank;     // 刚填满、等待发送的 bank
    reg              cur_bank;     // 正在发送的 bank（发送期间必须冻结）
    reg [BUF_AW-1:0] wr_ptr;
    reg [15:0]       wr_crc;

    reg              line_ready;
    reg [15:0]       line_wc;
    reg [15:0]       line_crc;

    reg              fs_pend;
    reg              fe_pend;

    //---------------- 发送状态 ----------------
    reg              st;
    localparam S_IDLE = 1'b0;
    localparam S_SEND = 1'b1;

    reg [15:0]       byte_ptr;
    reg [15:0]       cur_len;
    reg [15:0]       cur_wc;
    reg [23:0]       hdr_data;      // {DI, WC[15:0]}
    reg [15:0]       crc_send;

    reg [L*8-1:0]    pay_word_q;    // 预取的行缓冲数据

    //---------------- 包头 ECC ----------------
    wire [7:0]  hdr_ecc;
    csi2_ecc u_ecc (
        .I_hdr ( hdr_data ),
        .O_ecc ( hdr_ecc  )
    );
    wire [31:0] hdr_word = { hdr_data, hdr_ecc };   // 4 个包头字节

    //---------------- 写侧并行 CRC ----------------
    wire [15:0] crc_next;
    crc16_par2 u_crc (
        .I_crc  ( wr_crc    ),
        .I_data ( I_wr_data ),
        .O_crc  ( crc_next  )
    );

    //---------------- 行缓冲预取（同步读，1 拍延迟） ----------------
    wire [15:0] nxt_ptr  = byte_ptr + L;
    wire [15:0] nxt_pidx = (nxt_ptr >= 16'd4) ? ((nxt_ptr - 16'd4) >> LOG2L) : 16'd0;

    always @(posedge I_clk) begin
        pay_word_q <= mem[{ cur_bank, nxt_pidx[BUF_AW-1:0] }];
    end

    //---------------- 当前 beat 的字节拼装 ----------------
    reg  [7:0]  beat_b [0:3];
    reg  [15:0] idx_t;
    integer     k;

    always @(*) begin
        for (k = 0; k < 4; k = k + 1)
            beat_b[k] = 8'h00;

        for (k = 0; k < L; k = k + 1) begin
            idx_t = byte_ptr + k;
            if (idx_t < 16'd4)
                beat_b[k] = hdr_word[8*(3 - idx_t) +: 8];
            else if (idx_t < (16'd4 + cur_wc))
                beat_b[k] = pay_word_q[8*(L - 1 - k) +: 8];
            else if (idx_t == (16'd4 + cur_wc))
                beat_b[k] = crc_send[15:8];
            else if (idx_t == (16'd4 + cur_wc + 16'd1))
                beat_b[k] = crc_send[7:0];
            else
                beat_b[k] = 8'h00;
        end
    end

    reg [L*8-1:0] beat_q;

    always @(*) begin
        beat_q = {L*8{1'b0}};
        for (k = 0; k < L; k = k + 1)
            beat_q[8*((LANE_SWAP ? k : (L - 1 - k))) +: 8] = beat_b[k];
    end

    //---------------- 输出到 PHY ----------------
    assign O_hs_en   = (st == S_SEND);
    assign O_hs_data = beat_q;
    assign O_hs_last = (st == S_SEND) && ((byte_ptr + L) >= cur_len);

    //---------------- 主状态机 ----------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            wr_bank    <= 1'b0;
            rdy_bank   <= 1'b0;
            cur_bank   <= 1'b0;
            wr_ptr     <= {BUF_AW{1'b0}};
            wr_crc     <= 16'hFFFF;
            line_ready <= 1'b0;
            line_wc    <= 16'd0;
            line_crc   <= 16'd0;
            fs_pend    <= 1'b0;
            fe_pend    <= 1'b0;
            st         <= S_IDLE;
            byte_ptr   <= 16'd0;
            cur_len    <= 16'd0;
            cur_wc     <= 16'd0;
            hdr_data   <= 24'd0;
            crc_send   <= 16'd0;
            O_overflow <= 1'b0;
            O_wc_err   <= 1'b0;
        end else begin
            //-------- 帧边界锁存 --------
            if (I_frame_start) fs_pend <= 1'b1;
            if (I_frame_end)   fe_pend <= 1'b1;

            //-------- 写侧 --------
            if (I_wr_en) begin
                mem[{ wr_bank, wr_ptr }] <= I_wr_data;
                wr_ptr <= wr_ptr + 1'b1;
                wr_crc <= crc_next;
            end

            if (I_pkt_end) begin
                if (line_ready)      O_overflow <= 1'b1;
                // 每行刷新，反映当行状态（不再锁存历史）
                O_wc_err <= I_pkt_wc[0] | (in_words > BANK_W);

                rdy_bank   <= wr_bank;
                line_wc    <= I_pkt_wc;
                line_crc   <= wr_crc;
                line_ready <= 1'b1;

                wr_bank    <= ~wr_bank;
                wr_ptr     <= {BUF_AW{1'b0}};
                wr_crc     <= 16'hFFFF;
            end

            //-------- 发送侧 --------
            case (st)
            S_IDLE: begin
                if (I_hs_ready) begin
                    if (fs_pend) begin
                        // 帧起短包
                        fs_pend  <= 1'b0;
                        hdr_data <= { VC, 6'h00, 16'h0000 };
                        cur_wc   <= 16'd0;
                        cur_len  <= 16'd4;
                        byte_ptr <= 16'd0;
                        st       <= S_SEND;
                    end else if (line_ready) begin
                        // 数据长包
                        // 关键：发送期间必须冻结读 bank。若等下一次 pkt_end 才更新，
                        // 当新行在上一次发送结束前到达时，读指针会突然跳到另一个
                        // bank，导致正在发送的行被静默污染。
                        line_ready <= 1'b0;
                        cur_bank   <= rdy_bank;
                        // CSI-2 包头字节序为 {DI, WC_LSB, WC_MSB, ECC}：先发低字节
        hdr_data   <= { VC, DT[5:0], line_wc[7:0], line_wc[15:8] };
                        cur_wc     <= line_wc;
                        cur_len    <= 16'd6 + line_wc;
                        crc_send   <= line_crc;
                        byte_ptr   <= 16'd0;
                        st         <= S_SEND;
                    end else if (fe_pend) begin
                        // 帧尾短包
                        fe_pend  <= 1'b0;
                        hdr_data <= { VC, 6'h01, 16'h0000 };
                        cur_wc   <= 16'd0;
                        cur_len  <= 16'd4;
                        byte_ptr <= 16'd0;
                        st       <= S_SEND;
                    end
                end
            end

            S_SEND: begin
                if ((byte_ptr + L) >= cur_len) begin
                    byte_ptr <= 16'd0;
                    st       <= S_IDLE;
                end else begin
                    byte_ptr <= byte_ptr + L;
                end
            end
            endcase
        end
    end

endmodule
