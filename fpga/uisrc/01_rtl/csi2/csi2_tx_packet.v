//==========================================================================
// csi2_tx_packet.v
//   行缓冲 + CSI-2 打包：把 csi2_rx_parse 送来的一行载荷重新封装成
//   FS / 长包 / FE 的标准包流，从 DPHY1 发出去。
//
//   关键约定（实测与摄像头/A 板一致，勿改）：
//     · 包头字节序：DI, WC_LSB, WC_MSB, ECC
//     · ECC 的输入字节序：{WC_MSB, WC_LSB, DI}
//   行缓冲乒乓：发送期间冻结读 bank（cur_bank），避免新行把正在发的行冲掉。
//   读写同属一个时钟域，合并进单个 always 块，避免多驱动。
//==========================================================================
module csi2_tx_packet #(
    parameter LANE_NUM  = 2,
    parameter VC        = 2'b00,
    parameter DT        = 8'h2B,
    parameter BUF_AW    = 10,
    parameter USE_ERAM_BUFFER = 0,
    parameter LANE_SWAP = 0
)(
    input  wire                   I_clk,
    input  wire                   I_rst,

    input  wire                   I_wr_en,
    input  wire [LANE_NUM*8-1:0]  I_wr_data,
    input  wire                   I_pkt_end,
    input  wire [15:0]            I_pkt_wc,

    input  wire                   I_frame_start,
    input  wire                   I_frame_end,

    output reg                    O_hs_en,
    output reg  [LANE_NUM*8-1:0]  O_hs_data,
    output reg                    O_hs_last,
    input  wire                   I_hs_ready,

    output reg                    O_overflow,
    output reg                    O_wc_err,
    output wire                   O_data_ready,
    output wire                   O_pkt_ready,

    output reg                    O_dbg_pkt_done,
    output reg  [7:0]             O_dbg_pkt_dt,
    output reg  [15:0]            O_dbg_pkt_wc
);

    localparam L      = LANE_NUM;
    localparam W      = L*8;
    localparam BANK_W = 1 << BUF_AW;

    localparam S_IDLE = 3'd0;
    localparam S_FS   = 3'd1;
    localparam S_LONG = 3'd2;
    localparam S_FE   = 3'd3;
    localparam S_GAP  = 3'd4;

    reg              wr_bank, rdy_bank, cur_bank;
    reg              line_ready;
    reg [BUF_AW-1:0] wptr;
    reg [15:0]       line_wc;
    reg              fs_pend, fe_pend;
    reg              fs_ever;      // 至少发过一个 FS 之后才允许发行包

    reg [2:0]        st;
    reg [15:0]       byte_cnt;
    reg [15:0]       pkt_bytes;
    reg [15:0]       crc_r;
    reg [15:0]       gap;
    reg [15:0]       wc_lat;
    reg [23:0]       hdr_data;

    //------------------------------------------------------------------
    // 包头 ECC 与 CRC
    //------------------------------------------------------------------
    wire [23:0] ecc_in = { hdr_data[7:0], hdr_data[15:8], hdr_data[23:16] };
    wire [7:0]  hdr_ecc;
    csi2_ecc u_ecc ( .I_hdr ( ecc_in ), .O_ecc ( hdr_ecc ) );

    function [15:0] crc_byte;
        input [15:0] s;
        input [7:0]  b;
        integer i;
        reg [15:0] c;
        reg fb;
        begin
            c = s;
            for (i = 7; i >= 0; i = i - 1) begin
                fb = c[15] ^ b[i];
                c  = { c[14:0], 1'b0 };
                if (fb) c = c ^ 16'h1021;
            end
            crc_byte = c;
        end
    endfunction

    wire [7:0] hdr_b0 = hdr_data[23:16];   // DI
    wire [7:0] hdr_b1 = hdr_data[15:8];    // WC_LSB
    wire [7:0] hdr_b2 = hdr_data[7:0];     // WC_MSB

    reg [7:0] byte_a, byte_b;

    wire [BUF_AW-1:0] pay_idx = (byte_cnt - 16'd4) >> 1;
    wire packet_start_prefetch = (st == S_IDLE) && I_hs_ready &&
                                 line_ready && fs_ever;
    wire packet_start_now = (st == S_IDLE) && I_hs_ready &&
                            line_ready && fs_ever;
    // Permit one line to be written while the other bank is transmitting.
    // Once the ping-pong write bank would alias the read bank, hold the input
    // token in the upstream FIFO until the current packet starts/finishes.
    assign O_data_ready = (wptr != BANK_W - 1) &&
                          !(line_ready && (wr_bank == cur_bank) &&
                            !packet_start_now);
    assign O_pkt_ready  = !line_ready || packet_start_now;
    wire [15:0] prefetch_byte_cnt = byte_cnt + 16'd2;
    wire [BUF_AW-1:0] prefetch_idx = packet_start_prefetch ? {BUF_AW{1'b0}} :
        ((st == S_LONG) && (prefetch_byte_cnt >= 16'd4) &&
         (prefetch_byte_cnt < (16'd4 + wc_lat)) ?
         ((prefetch_byte_cnt - 16'd4) >> 1) : {BUF_AW{1'b0}});
    wire prefetch_bank = packet_start_prefetch ? rdy_bank : cur_bank;
    wire prefetch_en = packet_start_prefetch || (st == S_LONG);
    wire [15:0] pay_word;

    generate
        if (USE_ERAM_BUFFER) begin : g_packet_buffer
`ifdef CNN_SIM
            reg [15:0] sim_mem [0:2*BANK_W-1];
            reg [15:0] sim_pay_word;

            always @(posedge I_clk) begin
                if (I_wr_en)
                    sim_mem[{wr_bank, wptr}] <= I_wr_data;
                if (prefetch_en)
                    sim_pay_word <= sim_mem[{prefetch_bank, prefetch_idx}];
            end

            assign pay_word = sim_pay_word;
`else
            if (BUF_AW == 11) begin : g_ph1p_eram
                wire [7:0] low_data [0:1];
                wire [7:0] high_data [0:1];
                reg read_bank_q;
                genvar bank;

                always @(posedge I_clk or posedge I_rst) begin
                    if (I_rst)
                        read_bank_q <= 1'b0;
                    else if (prefetch_en)
                        read_bank_q <= prefetch_bank;
                end

                for (bank = 0; bank < 2; bank = bank + 1) begin : g_line_bank
                    localparam BANK_INDEX = bank;
                    wire write_select = I_wr_en && (wr_bank == BANK_INDEX);
                    wire read_select = prefetch_en && (prefetch_bank == BANK_INDEX);

                    PH1P_LOGIC_ERAM #(
                        .DATA_WIDTH_A(8), .ADDR_WIDTH_A(BUF_AW), .DATA_DEPTH_A(BANK_W),
                        .DATA_WIDTH_B(8), .ADDR_WIDTH_B(BUF_AW), .DATA_DEPTH_B(BANK_W),
                        .MODE("PDPW"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
                        .IMPLEMENT("20K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
                        .INIT_FILE("NONE"), .FILL_ALL("NONE")
                    ) u_low_ram (
                        .dia(I_wr_data[7:0]), .dib(8'd0),
                        .addra(wptr), .addrb(prefetch_idx),
                        .cea(write_select), .ceb(read_select),
                        .ocea(1'b0), .oceb(1'b0),
                        .clka(I_clk), .clkb(I_clk),
                        .wea(write_select), .web(1'b0),
                        .bea(8'hFF), .beb(8'h00),
                        .rsta(1'b0), .rstb(1'b0),
                        .doa(), .dob(low_data[bank])
                    );

                    PH1P_LOGIC_ERAM #(
                        .DATA_WIDTH_A(8), .ADDR_WIDTH_A(BUF_AW), .DATA_DEPTH_A(BANK_W),
                        .DATA_WIDTH_B(8), .ADDR_WIDTH_B(BUF_AW), .DATA_DEPTH_B(BANK_W),
                        .MODE("PDPW"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
                        .IMPLEMENT("20K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
                        .INIT_FILE("NONE"), .FILL_ALL("NONE")
                    ) u_high_ram (
                        .dia(I_wr_data[15:8]), .dib(8'd0),
                        .addra(wptr), .addrb(prefetch_idx),
                        .cea(write_select), .ceb(read_select),
                        .ocea(1'b0), .oceb(1'b0),
                        .clka(I_clk), .clkb(I_clk),
                        .wea(write_select), .web(1'b0),
                        .bea(8'hFF), .beb(8'h00),
                        .rsta(1'b0), .rstb(1'b0),
                        .doa(), .dob(high_data[bank])
                    );
                end

                assign pay_word = {high_data[read_bank_q], low_data[read_bank_q]};
            end else begin : g_fallback_buffer
                reg [15:0] fallback_mem [0:2*BANK_W-1];
                always @(posedge I_clk)
                    if (I_wr_en)
                        fallback_mem[{wr_bank, wptr}] <= I_wr_data;
                assign pay_word = fallback_mem[{cur_bank, pay_idx}];
            end
`endif
        end else begin : g_lut_packet_buffer
            reg [15:0] mem [0:2*BANK_W-1];
            always @(posedge I_clk)
                if (I_wr_en)
                    mem[{wr_bank, wptr}] <= I_wr_data;
            assign pay_word = mem[{cur_bank, pay_idx}];
        end
    endgenerate

    always @(*) begin
        if (byte_cnt[15:1] == 15'd0) begin byte_a = hdr_b0; byte_b = hdr_b1; end
        else                         begin byte_a = hdr_b2; byte_b = hdr_ecc; end

        if (st == S_LONG && byte_cnt >= 16'd4) begin
            if (byte_cnt < (16'd4 + wc_lat)) begin
                byte_a = pay_word[15:8];
                byte_b = pay_word[7:0];
            end else begin
                byte_a = crc_r[15:8];
                byte_b = crc_r[7:0];
            end
        end
    end

    // LANE_SWAP=0：先发的字节在 bit[15:8]（与摄像头一致）
    // LANE_SWAP=1：两个字节对调（两块转接板的 D0/D1 相反时用）
    wire [7:0] beat_hi = LANE_SWAP ? byte_b : byte_a;
    wire [7:0] beat_lo = LANE_SWAP ? byte_a : byte_b;

    //------------------------------------------------------------------
    // 单一时序块：读侧 FSM 在前，写侧在后（写侧优先级更高）
    //------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            wr_bank    <= 1'b0;
            rdy_bank   <= 1'b0;
            cur_bank   <= 1'b0;
            line_ready <= 1'b0;
            wptr       <= {BUF_AW{1'b0}};
            line_wc    <= 16'd0;
            fs_pend    <= 1'b0;
            fe_pend    <= 1'b0;
            fs_ever    <= 1'b0;
            st         <= S_IDLE;
            byte_cnt   <= 16'd0;
            pkt_bytes  <= 16'd0;
            crc_r      <= 16'hFFFF;
            gap        <= 16'd0;
            wc_lat     <= 16'd0;
            hdr_data   <= 24'd0;
            O_hs_en    <= 1'b0;
            O_hs_data  <= {W{1'b0}};
            O_hs_last  <= 1'b0;
            O_overflow <= 1'b0;
            O_wc_err   <= 1'b0;
            O_dbg_pkt_done <= 1'b0;
            O_dbg_pkt_dt   <= 8'd0;
            O_dbg_pkt_wc   <= 16'd0;
        end else begin
            O_dbg_pkt_done <= 1'b0;

            //======================= 读侧 FSM =======================
            case (st)
            S_IDLE: begin
                O_hs_en   <= 1'b0;
                O_hs_last <= 1'b0;
                if (I_hs_ready) begin
                    // 优先级：行包 -> FE -> FS
                    //   · 行包先发：帧尾总在一帧所有行之后到达
                    //   · FE 在 FS 之前：帧结束 -> 下一帧开始
                    //   · 上电时还没有 FE，FS 会立刻发出去
                    if (line_ready && fs_ever) begin
                        hdr_data  <= { VC, DT[5:0], line_wc[7:0], line_wc[15:8] };
                        wc_lat    <= line_wc;
                        pkt_bytes <= 16'd4 + line_wc + 16'd2;
                        byte_cnt  <= 16'd0;
                        crc_r     <= 16'hFFFF;
                        cur_bank  <= rdy_bank;
                        line_ready<= 1'b0;
                        st        <= S_LONG;
                    end else if (fe_pend) begin
                        hdr_data  <= { VC, 6'h01, 16'h0000 };
                        pkt_bytes <= 16'd4;
                        byte_cnt  <= 16'd0;
                        fe_pend   <= 1'b0;
                        st        <= S_FE;
                    end else if (fs_pend) begin
                        hdr_data  <= { VC, 6'h00, 16'h0000 };
                        pkt_bytes <= 16'd4;
                        byte_cnt  <= 16'd0;
                        fs_pend   <= 1'b0;
                        fs_ever   <= 1'b1;
                        st        <= S_FS;
                    end
                end
            end

            S_FS, S_FE: begin
                O_hs_en   <= 1'b1;
                O_hs_data <= { beat_hi, beat_lo };
                O_hs_last <= (byte_cnt >= pkt_bytes - 16'd2);   // 只有最后一拍
                if (byte_cnt >= pkt_bytes - 16'd2) begin
                    O_dbg_pkt_done <= 1'b1;
                    O_dbg_pkt_dt   <= (st == S_FS) ? { VC, 6'h00 } : { VC, 6'h01 };
                    O_dbg_pkt_wc   <= 16'd0;
                    gap <= 16'd0;
                    st  <= S_GAP;          // 本拍仍输出，GAP 才拉低 hs_en
                end else begin
                    byte_cnt <= byte_cnt + 16'd2;
                end
            end

            S_LONG: begin
                O_hs_en   <= 1'b1;
                O_hs_data <= { beat_hi, beat_lo };
                O_hs_last <= (byte_cnt >= pkt_bytes - 16'd2);

                if (byte_cnt >= 16'd4 && byte_cnt < (16'd4 + wc_lat))
                    crc_r <= crc_byte(crc_byte(crc_r, byte_a), byte_b);

                if (byte_cnt >= pkt_bytes - 16'd2) begin
                    O_dbg_pkt_done <= 1'b1;
                    O_dbg_pkt_dt   <= { VC, DT[5:0] };
                    O_dbg_pkt_wc   <= wc_lat;
                    gap <= 16'd0;
                    st  <= S_GAP;          // 本拍仍输出
                end else begin
                    byte_cnt <= byte_cnt + 16'd2;
                end
            end

            S_GAP: begin
                O_hs_en   <= 1'b0;
                O_hs_last <= 1'b0;
                if (gap >= 16'd8) st <= S_IDLE;
                else                gap <= gap + 16'd1;
            end

            default: st <= S_IDLE;
            endcase

            //======================= 写侧 =======================
            if (I_frame_start) fs_pend <= 1'b1;
            if (I_frame_end)   fe_pend <= 1'b1;

            if (I_wr_en) begin
                if (wptr == BANK_W - 1) O_overflow <= 1'b1;
                else                    wptr <= wptr + 1'b1;
            end

            if (I_pkt_end) begin
                if (line_ready && !packet_start_now) O_overflow <= 1'b1;
                rdy_bank   <= wr_bank;
                line_wc    <= I_pkt_wc;
                line_ready <= 1'b1;
                wr_bank    <= ~wr_bank;
                wptr       <= {BUF_AW{1'b0}};
                O_wc_err   <= I_pkt_wc[0] | (((I_pkt_wc + (L-1)) >> 1) > BANK_W);
            end
        end
    end

endmodule
