//==========================================================================
// csi2_tx_top.v
//   B 板（MLKPAI-F02 / PH1P35MDG324）CSI-2 中继顶层
//
//   数据流：
//     SC500CS 摄像头 --2lane RAW10--> [CSI 座] DPHY0 RX
//        -> csi2_rx_parse（RX 时钟域）
//        -> 令牌打包 -> 异步 FIFO（跨时钟域）
//        -> 令牌解包（TX 时钟域）
//        -> csi2_tx_packet -> DPHY1 TX -> [DSI 座] ==> A 板 J9
//
//   时钟：
//     I_sys_clk 25MHz -> PLL -> clk0=100MHz（逻辑/DPLL 参考）、clk1=24MHz（摄像头 XCLK）
//     RX 字节时钟由摄像头 MIPI 时钟恢复（O_hs_rx_clk）
//     TX 字节时钟由 DPHY1 内部 DPLL 产生（O_hs_tx_clk，本设计 68.75MHz → 550Mbps/lane）
//
//   ⚠ 依赖：pll IP 必须重新生成为 25MHz 输入、clk0=100MHz、clk1=24MHz
//   ⚠ 刻意不驱动 O_screen_pwm(R11)：不约束即高阻，避免与 A 板第 29 脚冲突
//==========================================================================
module csi2_tx_top #(
    parameter DBG_MODE = 0,           // 1 = 三颗灯改为显示 I2C 内部状态
    // 两块 14→40 转接板的 D0/D1 顺序相反，实机必须为 1。
    // 提成参数是为了让 testbench 能传 0 来按自然字节序校验。
    parameter LANE_SWAP = 1
)(
    input  wire        I_sys_clk,          // 25MHz 单端，C4

    input  wire        I_rst_n,

    // 按键：I_button[0] 减小曝光，I_button[1] 增大曝光（低有效）
    input  wire [1:0]  I_button,

    // 摄像头边带
    output wire        O_cam_scl,          // T9
    inout  wire        IO_cam_sda,         // U9
    output wire        O_cam_24m,          // V9
    output wire        O_cam_rst,          // V8

    // MIPI DPHY1 TX -> DSI 座
    inout  wire        IO_tx_clk_pad_p,
    inout  wire        IO_tx_clk_pad_n,
    inout  wire [3:0]  IO_tx_data_pad_p,
    inout  wire [3:0]  IO_tx_data_pad_n,

    // MIPI DPHY0 RX <- CSI 座（接摄像头）
    inout  wire        IO_rx_clk_pad_p,
    inout  wire        IO_rx_clk_pad_n,
    inout  wire [3:0]  IO_rx_data_pad_p,
    inout  wire [3:0]  IO_rx_data_pad_n,

    // 调试指示（3 个 LED，构成上板诊断阶梯）
    output wire        O_led_cfg,      // J18：摄像头 I2C 配置完成
    output wire        O_led_beat,     // K16：帧活动心跳，约 1Hz 闪烁
    output wire        O_led_err,      // K17：overflow 或 wc_err
    output wire        O_uart_tx       // J3：调试串口 115200 8N1
);

    localparam LANES = 2;

    //==================================================================
    // 1. 时钟与复位
    //==================================================================
    wire clk_100m;
    wire clk_24m;
    wire pll_lock;

    pll u_pll (
        .refclk   ( I_sys_clk ),
        .reset    ( ~I_rst_n  ),
        .clk0_out ( clk_100m  ),
        .clk1_out ( clk_24m   ),
        .lock     ( pll_lock  )
    );

    wire sys_rst_n = I_rst_n & pll_lock;

    // 各时钟域复位同步（低有效）
    reg [1:0] rsync_100, rsync_24;

    always @(posedge clk_100m or negedge sys_rst_n)
        if (!sys_rst_n) rsync_100 <= 2'b00; else rsync_100 <= { rsync_100[0], 1'b1 };

    always @(posedge clk_24m or negedge sys_rst_n)
        if (!sys_rst_n) rsync_24 <= 2'b00; else rsync_24 <= { rsync_24[0], 1'b1 };

    wire rst_n_100 = rsync_100[1];
    wire rst_n_24  = rsync_24[1];

    //==================================================================
    // 2. 摄像头时钟与配置
    //==================================================================
    // V8 是 MIPI_CSI_PWDNB（低有效掉电）。
    // 厂家例程恒定拉高：摄像头随板上电即工作，等 FPGA 配置好时早已就绪，
    // I2C 配置可以直接开始。
    // 若改成由复位控制，复位期间摄像头会被掉电，而释放后需要数毫秒才能
    // 响应 I2C，配准会静默失败——这是必须避开的坑。
    assign O_cam_24m = clk_24m;
    assign O_cam_rst = 1'b1;

    wire cam_cfg_done;
    wire cam_ae_done;
    wire iic_busy_raw;
    wire iic_ack_raw;
    wire iic_err_raw;

    //------------------------------------------------------------------
    // 曝光/增益设定
    //   ae_set 的默认值是 ae=3000 / ag=60，且只有"按键改变数值"时才拉高
    //   O_ae_req（初始 r_ae == r_ae_set，条件不成立）。所以：
    //     - 不按键时行为与"完全不接 AE"完全一致（不写 AE 寄存器）
    //     - 按键即可手动调曝光，方便现场调亮度
    //------------------------------------------------------------------
    wire        ae_req;
    wire [15:0] ae_val;
    wire [15:0] ag_val;

    ae_set u_ae_set (
        .I_clk         ( clk_24m      ),
        .I_rst         ( ~rst_n_24    ),
        .I_btn         ( { I_button, 2'b11 } ),
        .I_ae_cfg_done ( cam_ae_done  ),
        .I_cam_cfg_done( cam_cfg_done ),
        .O_ae_req      ( ae_req       ),
        .O_ae          ( ae_val       ),
        .O_ag          ( ag_val       )
    );

    wire [8:0] dbg_regidx;     // 摄像头配置进度（仅调试用）

    uicfgcs500 #(
        .CLK_DIV ( 24000000 / 100000 - 1 )      // 24MHz -> 100kHz I2C
    ) u_cam_cfg (
        .I_clk        ( clk_24m      ),
        .I_rst_n      ( rst_n_24     ),
        .I_ae_req     ( ae_req       ),
        .I_ae         ( ae_val       ),
        .I_ag         ( ag_val       ),
        .O_cam_scl    ( O_cam_scl    ),
        .IO_cam_sda   ( IO_cam_sda   ),
        .O_cfg_done   ( cam_cfg_done ),
        .O_ae_cfg_done( cam_ae_done  ),
        .O_dbg_busy   ( iic_busy_raw ),
        .O_dbg_err    ( iic_err_raw  ),
        .O_dbg_regidx ( dbg_regidx   ),
        .O_dbg_ack    ( iic_ack_raw  )
    );

    //==================================================================
    // 3. DPHY0 接收
    //==================================================================
    //------------------------------------------------------------------
    // RX 数据采样延迟自动扫描：0..63，每 ~250ms 一步。
    // 摄像头在发时钟但收不到有效字节时，逐点扫描采样窗口。
    // 当前扫描值经串口 S=<n> 输出，V 一旦非 0 就说明找到了可用采样点。
    //------------------------------------------------------------------
    reg [22:0] swp_tick;
    reg [5:0]  swp;

    always @(posedge clk_24m or negedge rst_n_24) begin
        if (!rst_n_24) begin
            swp_tick <= 23'd0;
            swp      <= 6'd0;
        end else if (swp_tick >= 23'd5999999) begin
            swp_tick <= 23'd0;
            swp      <= swp;   // 固定为 0，与厂家例程一致
        end else begin
            swp_tick <= swp_tick + 23'd1;
        end
    end

    wire        rx_clk;
    wire        rx_valid;
    wire [15:0] rx_data;
    wire [1:0]  rx_lane_err;

    mipi_dphy_rx_ph1p_mipiio_wrapper #(
        .DPHY_RX_LOCATION ( "DPHY0" ),
        .HS_EQUALIZER     ( "0dB"   ),
        .HS_VGA_GAIN      ( "8dB"   ),
        .LANE_NUM         ( LANES   ),
        .BYTE_NUM         ( 1       )
    ) u_dphy_rx (
        .I_lp_clk              ( clk_100m        ),
        .I_rst                 ( ~rst_n_100      ),
        .I_clk_lane_in_delay   ( 6'd0            ),
        .I_data_lane0_in_delay ( 6'd0            ),
        .I_data_lane1_in_delay ( 6'd0            ),
        .I_data_lane2_in_delay ( 6'd0            ),
        .I_data_lane3_in_delay ( 6'd0            ),
        .I_lane_invert         ( 2'b00           ),
        .O_hs_rx_clk           ( rx_clk          ),
        .O_hs_rx_valid         ( rx_valid        ),
        .O_hs_rx_data          ( rx_data         ),
        .O_lp_rx_lane0_p       (                 ),
        .O_lp_rx_lane0_n       (                 ),
        .I_lp_tx_en            ( 1'b0            ),
        .I_lp_tx_lane0_p       ( 1'b1            ),
        .I_lp_tx_lane0_n       ( 1'b1            ),
        .O_lane_match_error    (                 ),
        .O_lane_error          ( rx_lane_err     ),
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p)
    );

    // RX 时钟域复位
    reg [1:0] rsync_rxq;
    always @(posedge rx_clk or negedge sys_rst_n)
        if (!sys_rst_n) rsync_rxq <= 2'b00; else rsync_rxq <= { rsync_rxq[0], 1'b1 };
    wire rst_n_rx = rsync_rxq[1];

    //==================================================================
    // 4. CSI-2 解析（RX 时钟域）
    //==================================================================
    wire        p_wr_en;
    wire [15:0] p_wr_data;
    wire        p_pkt_end;
    wire [15:0] p_pkt_wc;
    wire [7:0]  p_pkt_dt;
    wire        p_frm_start;
    wire        p_frm_end;
    wire        p_wc_err;

    csi2_rx_parse #( .LANE_NUM ( LANES ) ) u_rx_parse (
        .I_clk        ( rx_clk     ),
        .I_rst        ( ~rst_n_rx  ),
        .I_hs_valid   ( rx_valid   ),
        .I_hs_data    ( rx_data    ),
        .O_wr_en      ( p_wr_en    ),
        .O_wr_data    ( p_wr_data  ),
        .O_pkt_end    ( p_pkt_end  ),
        .O_pkt_wc     ( p_pkt_wc   ),
        .O_pkt_dt     ( p_pkt_dt   ),
        .O_frame_start( p_frm_start),
        .O_frame_end  ( p_frm_end  ),
        .O_wc_err     ( p_wc_err   )
    );

    //==================================================================
    // 5. 令牌打包（RX 域） -> 异步 FIFO
    //    令牌：{type[1:0], data[15:0]}
    //      0=载荷  1=frame_start  2=pkt_end(带WC)  3=frame_end
    //==================================================================
    reg         tk_we;
    reg  [17:0] tk_wdata;
    wire        tk_full;
    wire        tk_empty;
    wire [17:0] tk_rdata;
    wire        tk_re;

    //==================================================================
    // 5b. 帧边界
    //   实测：本传感器不发 DT=0x00（帧起），只发 DT=0x01（帧尾），
    //   每帧一个、20fps。所以由"帧尾"派生出帧起：
    //     收到帧尾包 -> 先发 FE 令牌，下一拍发 FS 令牌（新帧开始）
    //   两个令牌分两拍，避免令牌 FIFO 一拍只能写一个。
    //==================================================================
    // 独立的帧尾检测：每个 valid 上升沿都看类型字节，
    // 不受解析器当前状态影响（否则处于收载荷时会把帧尾包整个漏掉）
    reg        v_d2;
    reg        fe_det;
    reg [19:0] fe_hold;

    // 去抖：真正的帧间隔约 50ms，而同一次帧尾突发内的误触发间隔 <1us，
    // 用 ~356us 的屏蔽窗足以滤掉重复触发。
    localparam FE_DEB = 20'd20000;

    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            v_d2    <= 1'b0;
            fe_det  <= 1'b0;
            fe_hold <= 20'd0;
        end else begin
            v_d2 <= rx_valid;
            fe_det <= 1'b0;

            if (fe_hold != 20'd0) begin
                fe_hold <= fe_hold - 20'd1;
            end else if (!v_d2 && rx_valid && (rx_data[15:8] == 8'h01)) begin
                fe_det  <= 1'b1;
                fe_hold <= FE_DEB;
            end
        end
    end

    reg        frm_on;
    reg        fe_rq;
    reg        fs_rq;
    reg        gen_fs;
    reg        gen_fe;

    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            frm_on <= 1'b0;
            fe_rq  <= 1'b0;
            fs_rq  <= 1'b0;
            gen_fs <= 1'b0;
            gen_fe <= 1'b0;
        end else begin
            gen_fs <= 1'b0;
            gen_fe <= 1'b0;

            // 上电后第一行 -> 立刻开帧
            if (p_pkt_end && !frm_on) begin
                gen_fs <= 1'b1;
                frm_on <= 1'b1;
            end

            // 收到帧尾包 -> 排队 FE + FS
            if (fe_det) begin
                fe_rq <= 1'b1;
            end

            if (fe_rq) begin
                gen_fe <= 1'b1;
                fe_rq  <= 1'b0;
                fs_rq  <= 1'b1;
            end else if (fs_rq) begin
                gen_fs <= 1'b1;
                fs_rq  <= 1'b0;
            end
        end
    end
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            tk_we    <= 1'b0;
            tk_wdata <= 18'd0;
        end else if (!tk_full) begin
            tk_we <= 1'b1;
            // 00=载荷  01=帧起  10=行结束(带WC)  11=帧尾
            if (p_wr_en)
                tk_wdata <= { 2'b00, p_wr_data };
            else if (gen_fs)
                tk_wdata <= { 2'b01, 16'h0000 };
            else if (p_pkt_end)
                tk_wdata <= { 2'b10, p_pkt_wc };
            else if (gen_fe)
                tk_wdata <= { 2'b11, 16'h0000 };
            else
                tk_we <= 1'b0;
        end else begin
            tk_we <= 1'b0;
        end
    end

    //==================================================================
    // 6. DPHY1 发送
    //==================================================================
    wire tx_clk;
    wire tx_ready;
    wire tx_dpll_locked;

    // TX 复位必须等 DPHY1 的 DPLL 锁定：未锁定时 tx_clk 可能不稳，
    // 此时若让状态机跑起来会发出乱码。
    wire tx_rst_src_n = sys_rst_n & tx_dpll_locked;

    reg [1:0] rsync_txq;
    always @(posedge tx_clk or negedge tx_rst_src_n)
        if (!tx_rst_src_n) rsync_txq <= 2'b00; else rsync_txq <= { rsync_txq[0], 1'b1 };
    wire rst_n_tx = rsync_txq[1];

    // 令牌解包后送给 tx_packet 写侧
    reg         tx_wr_en;
    reg  [15:0] tx_wr_data;
    reg         tx_pkt_end;
    reg  [15:0] tx_pkt_wc;
    reg         tx_frm_start;
    reg         tx_frm_end;

    wire        tx_hs_en;
    wire [15:0] tx_hs_data;
    wire        tx_hs_last;

    // 令牌 FIFO 读侧：一直取
    assign tk_re = ~tk_empty;

    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx) begin
            tx_wr_en     <= 1'b0;
            tx_wr_data   <= 16'd0;
            tx_pkt_end   <= 1'b0;
            tx_pkt_wc    <= 16'd0;
            tx_frm_start <= 1'b0;
            tx_frm_end   <= 1'b0;
        end else begin
            tx_wr_en     <= 1'b0;
            tx_pkt_end   <= 1'b0;
            tx_frm_start <= 1'b0;
            tx_frm_end   <= 1'b0;

            if (!tk_empty) begin
                case (tk_rdata[17:16])
                2'b00: begin
                    tx_wr_en   <= 1'b1;
                    tx_wr_data <= tk_rdata[15:0];
                end
                2'b01: tx_frm_start <= 1'b1;
                2'b10: begin
                    tx_pkt_end <= 1'b1;
                    tx_pkt_wc  <= tk_rdata[15:0];
                end
                2'b11: tx_frm_end <= 1'b1;
                endcase
            end
        end
    end

    csi2_ptr_fifo #(
        .DW ( 18 ),
        .AW ( 11 )          // 2048 token，需覆盖一行 1205 word + 控制令牌
    ) u_tk_fifo (
        .I_wclk   ( rx_clk     ),
        .I_wrstn  ( rst_n_rx   ),
        .I_we     ( tk_we      ),
        .I_wdata  ( tk_wdata   ),
        .O_wfull  ( tk_full    ),
        .I_rclk   ( tx_clk     ),
        .I_rrstn  ( rst_n_tx   ),
        .I_re     ( tk_re      ),
        .O_rdata  ( tk_rdata   ),
        .O_rempty ( tk_empty   )
    );

    wire tx_ovf;
    wire tx_wcerr;

    // 行缓冲 + CSI-2 打包（B 板后续要在此处插入识别/OSD，需保留像素流）
    csi2_tx_packet #(
        .LANE_NUM ( LANES  ),
        .VC       ( 2'b00  ),
        .DT       ( 8'h2B  ),
        .LANE_SWAP( LANE_SWAP ),
        .BUF_AW   ( 11     )
    ) u_tx_packet (
        .I_clk        ( tx_clk      ),
        .I_rst        ( ~rst_n_tx   ),
        .I_wr_en      ( tx_wr_en    ),
        .I_wr_data    ( tx_wr_data  ),
        .I_pkt_end    ( tx_pkt_end  ),
        .I_pkt_wc     ( tx_pkt_wc   ),
        .I_frame_start( tx_frm_start),
        .I_frame_end  ( tx_frm_end  ),
        .O_hs_en      ( tx_hs_en    ),
        .O_hs_data    ( tx_hs_data  ),
        .O_hs_last    ( tx_hs_last  ),
        .I_hs_ready   ( tx_ready    ),
        .O_overflow   ( tx_ovf      ),
        .O_wc_err     ( tx_wcerr    )
    );

    //==================================================================
    // 7. 诊断指示灯
    //    J18 led_cfg  : 摄像头 I2C 配置完成（为 0 则后面全无意义）
    //    K16 led_beat : 每来 16 帧翻转一次，约 1Hz —— 说明视频流真的在动
    //    K17 led_err  : FIFO 溢出 或 WC 异常
    //==================================================================
    reg [9:0] frame_cnt;

    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx)          frame_cnt <= 10'd0;
        else if (tx_frm_start)  frame_cnt <= frame_cnt + 10'd1;
    end

    //------------------------------------------------------------------
    // 诊断模式（DBG_MODE=1）：把两颗灯改成看 I2C 内部状态
    //   K16 = iic_busy 连续 >4ms 不释放（控制器卡死）
    //   K17 = iic_bus_error 锁存（摄像头没回 ACK）
    // 正常模式（DBG_MODE=0）：帧心跳 + 数据通路错误
    //------------------------------------------------------------------
    reg [16:0] busy_cnt;
    reg        busy_stuck;
    reg        err_lat;

    always @(posedge clk_24m or negedge rst_n_24) begin
        if (!rst_n_24) begin
            busy_cnt   <= 17'd0;
            busy_stuck <= 1'b0;
        end else if (iic_busy_raw) begin
            if (busy_cnt != 17'h1FFFF) busy_cnt <= busy_cnt + 17'd1;
            else                       busy_stuck <= 1'b1;
        end else begin
            busy_cnt <= 17'd0;
        end
    end

    always @(posedge clk_24m or negedge rst_n_24) begin
        if (!rst_n_24)              err_lat <= 1'b0;
        else if (iic_err_raw)       err_lat <= 1'b1;
    end

    assign O_led_cfg  = cam_cfg_done;
    assign O_led_beat = DBG_MODE ? busy_stuck       : frame_cnt[4];
    assign O_led_err  = DBG_MODE ? err_lat          : (tx_ovf | tx_wcerr | ~tx_dpll_locked);

    mipi_dphy_tx_over_ph1p_mipiio_wrapper #(
        .DPHY_TX_LOCATION      ( "DPHY1"    ),
        .BYTE_NUM              ( 1          ),
        .LANE_NUM              ( LANES      ),
        .CLK_LANE_MODE         ( "CONTINUE" ),
        .HS_OUTPUT_VOD         ( "200mV"    ),
        .HS_OUTPUT_IMPEDANCE   ( "50ohm"    ),
        .HS_OUTPUT_DE_EMPHASIS ( "0dB"      ),
        .CK_OUTPUT_DELAY       ( 0          ),
        .L0_OUTPUT_DELAY       ( 0          ),
        .L1_OUTPUT_DELAY       ( 0          ),
        .L2_OUTPUT_DELAY       ( 0          ),
        .L3_OUTPUT_DELAY       ( 0          ),
        // 字节钟 = 100MHz * (multi+1) / 16，line rate = 8 * 字节钟
        //   6 -> 43.75MHz -> 350Mbps（2 lane 合 87.5MB/s）
        //  10 -> 68.75MHz -> 550Mbps（2 lane 合 137.5MB/s）
        // 1080p30 需要 77.8MB/s 平均，但行有效期内瞬时需要 120.6MB/s，
        // 6 只够 72%，每行都靠消隐期补债，稍微一抖就丢行；A 板丢一行即丢整帧。
        // 10 让瞬时也有 114% 余量，FIFO 不再需要靠消隐期排空。
        .DPLL_MULTI_RATIO      ( 10 ),
        .DPLL_DIV_RATIO        ( 1  ),
        .T_DATA_LPX            ( 10 ),
        .T_DATA_HS_PREPARE     ( 10 ),
        .T_DATA_HS_ZERO        ( 20 ),
        .T_DATA_HS_TRAIL       ( 10 ),
        .T_CLK_LPX             ( 10 ),
        .T_CLK_PREPARE         ( 10 ),
        .T_CLK_ZERO            ( 10 ),
        .T_CLK_PRE            ( 10 ),
        .T_CLK_POST           ( 10 ),
        .T_CLK_TRAIL           ( 10 )
    ) u_dphy_tx (
        .I_dphy_pll_ref_clk  ( clk_100m     ),
        .I_rst               ( ~rst_n_100   ),
        .O_dpll_locked       ( tx_dpll_locked ),
        .O_hs_tx_clk         ( tx_clk       ),
        .I_hs_tx_valid       ( tx_hs_en     ),
        .I_hs_tx_data        ( tx_hs_data   ),
        .I_hs_tx_last        ( tx_hs_last   ),
        .O_hs_tx_ready       ( tx_ready     ),
        .I_lp_tx_p           ( 1'b1         ),
        .I_lp_tx_n           ( 1'b1         ),
        .I_lp_rx_en          ( 1'b0         ),
        .O_lp_rx_p           (              ),
        .O_lp_rx_n           (              ),
        .IO_tx_clk_pad_n     ( IO_tx_clk_pad_n    ),
        .IO_tx_clk_pad_p     ( IO_tx_clk_pad_p    ),
        .IO_tx_data_pad_n    ( IO_tx_data_pad_n   ),
        .IO_tx_data_pad_p    ( IO_tx_data_pad_p   )
    );

    //==================================================================
    // 8. 调试串口（115200 8N1，每 ~100ms 打印一行）
    //    这些信号跨时钟域直接采样，仅用于观察，不参与逻辑
    //==================================================================
    reg [15:0] dbg_frame;
    reg [15:0] dbg_wc;
    reg [7:0]  dbg_dt;
    reg        dbg_ovf;
    reg        dbg_werr;
    reg [15:0] dbg_rxclk;
    reg [15:0] dbg_rxval;

    // 抓 valid 上升沿那一拍和紧随其后的一拍，用于核对包头字节序
    reg        v_d;
    reg [1:0]  hcap;
    reg [15:0] h0_r;
    reg [15:0] h1_r;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            v_d  <= 1'b0;
            hcap <= 2'd0;
            h0_r <= 16'd0;
            h1_r <= 16'd0;
        end else begin
            v_d <= rx_valid;
            if (!v_d && rx_valid) begin
                h0_r <= rx_data;
                hcap <= 2'd1;
            end else if (rx_valid && hcap == 2'd1) begin
                h1_r <= rx_data;
                hcap <= 2'd2;
            end
        end
    end

    reg [15:0] dbg_gapmax;
    reg [15:0] dbg_pktcnt;
    reg [15:0] dbg_h0;
    reg [15:0] dbg_h1;

    // 解析器识别到的帧起/帧尾包计数（用于判断传感器的 FS/FE 是否可用）
    reg [15:0] fs_cnt, fe_cnt;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            fs_cnt <= 16'd0;
            fe_cnt <= 16'd0;
        end else begin
            // 注意：改成统计"任意 WC 但 DT 匹配"的包，
            // 用来确认那些 20 拍突发是否就是帧标记
            if (p_pkt_end && p_pkt_dt[5:0] == 6'h00 && fs_cnt != 16'hFFFF)
                fs_cnt <= fs_cnt + 16'd1;
            if (p_pkt_end && p_pkt_dt[5:0] == 6'h01 && fe_cnt != 16'hFFFF)
                fe_cnt <= fe_cnt + 16'd1;
        end
    end

    // TX 实际发出的拍数：为 0 就说明一个字节都没发出去
    reg [15:0] txbeat_cnt;
    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx) txbeat_cnt <= 16'd0;
        else if (tx_hs_en && txbeat_cnt != 16'hFFFF) txbeat_cnt <= txbeat_cnt + 16'd1;
    end

    reg [15:0] dbg_fs, dbg_fe;

    // 每帧行数测量：统计相邻两个 DT=01 帧标记之间的行包数。
    // 这是摄像头的真实高度，用来核对 A 板的 SENSOR_HEIGHT。
    reg [15:0] lcnt_lpf, lpf_lat;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            lcnt_lpf <= 16'd0;
            lpf_lat  <= 16'd0;
        end else begin
            if (p_pkt_end)               lcnt_lpf <= lcnt_lpf + 16'd1;
            if (fe_det) begin
                lpf_lat  <= lcnt_lpf;
                lcnt_lpf <= 16'd0;
            end
        end
    end

    // 统计每个 HS 突发的拍数（valid 连续为高的长度），用于判断包尾结构
    reg [15:0] burst_cnt;
    reg [15:0] burst_last;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            burst_cnt  <= 16'd0;
            burst_last <= 16'd0;
        end else if (rx_valid) begin
            burst_cnt <= burst_cnt + 16'd1;
        end else begin
            if (burst_cnt != 16'd0) burst_last <= burst_cnt;
            burst_cnt <= 16'd0;
        end
    end

    reg [15:0] dbg_burst;

    // RX 帧尾计数（rx 域）：证明解析器识别到了 FE
    reg [15:0] rxfe_cnt;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx)                         rxfe_cnt <= 16'd0;
        else if (p_frm_end && rxfe_cnt != 16'hFFFF) rxfe_cnt <= rxfe_cnt + 16'd1;
    end

    // TX 时钟计数（tx 域）：T 不动就说明 TX 侧时钟没跑
    reg [15:0] txclk_cnt;
    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx) txclk_cnt <= 16'd0;
        else           txclk_cnt <= txclk_cnt + 16'd1;
    end

    reg [15:0] dbg_rxfe;
    reg [15:0] dbg_txcnt;

    // RX 时钟域计数：K 涨说明摄像头在发 MIPI，V 涨说明真的收到有效字节
    reg [15:0] rxclk_cnt;
    reg [15:0] rxvalid_cnt;

    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            rxclk_cnt   <= 16'd0;
            rxvalid_cnt <= 16'd0;
        end else begin
            rxclk_cnt <= rxclk_cnt + 16'd1;
            if (rx_valid) rxvalid_cnt <= rxvalid_cnt + 16'd1;
        end
    end

    // NACK 上升沿计数：量化配置期间到底有多少次没有应答
    reg        err_d;
    reg [15:0] nack_cnt;
    reg        ack_d;
    reg [15:0] ack_cnt;

    always @(posedge clk_24m or negedge rst_n_24) begin
        if (!rst_n_24) begin
            err_d    <= 1'b0;
            nack_cnt <= 16'd0;
            ack_d    <= 1'b0;
            ack_cnt  <= 16'd0;
        end else begin
            err_d <= iic_err_raw;
            if (iic_err_raw && !err_d && nack_cnt != 16'hFFFF)
                nack_cnt <= nack_cnt + 16'd1;

            ack_d <= iic_ack_raw;
            if (iic_ack_raw && !ack_d && ack_cnt != 16'hFFFF)
                ack_cnt <= ack_cnt + 16'd1;
        end
    end

    always @(posedge clk_24m) begin
        dbg_frame <= frame_cnt;
        dbg_wc    <= p_pkt_wc;
        dbg_dt    <= p_pkt_dt;
        dbg_ovf   <= tx_ovf;
        dbg_werr  <= tx_wcerr;
        dbg_rxclk <= rxclk_cnt;
        dbg_rxval <= rxvalid_cnt;
        dbg_rxfe  <= rxfe_cnt;
        dbg_txcnt <= txclk_cnt;
        dbg_burst <= burst_last;
        dbg_fs    <= lpf_lat;   // A 字段改为每帧行数
        dbg_fe    <= txbeat_cnt;
        dbg_h0    <= h0_r;
        dbg_h1    <= h1_r;
    end

    dbg_uart #(
        .CLK_HZ ( 24000000 ),
        .BAUD   ( 115200   ),
        .GAP_MS ( 100      )
    ) u_dbg_uart (
        .I_clk      ( clk_24m      ),
        .I_rst_n    ( rst_n_24     ),
        .I_st_cfg   ( cam_cfg_done ),
        .I_st_wc    ( dbg_wc       ),
        .I_st_dt    ( dbg_dt       ),
        .I_st_burst ( dbg_burst    ),
        .I_st_h0    ( dbg_h0       ),
        .I_st_h1    ( dbg_h1       ),
        .I_st_frame ( dbg_frame    ),
        .I_st_rxval ( dbg_rxval    ),
        .I_st_werr  ( dbg_werr     ),
        .I_st_gapmax( dbg_fs       ),
        .I_st_pktcnt( dbg_fe       ),
        .O_uart_tx  ( O_uart_tx    )
    );

endmodule
