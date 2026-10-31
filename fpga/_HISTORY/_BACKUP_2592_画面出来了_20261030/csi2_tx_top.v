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
//     TX 字节时钟由 DPHY1 内部 DPLL 产生（O_hs_tx_clk，本设计约 75MHz）
//
//   ⚠ 依赖：pll IP 必须重新生成为 25MHz 输入、clk0=100MHz、clk1=24MHz
//   ⚠ 刻意不驱动 O_screen_pwm(R11)：不约束即高阻，避免与 A 板第 29 脚冲突
//==========================================================================
module csi2_tx_top #(
    parameter DBG_MODE = 0,           // 1 = 三颗灯改为显示 I2C 内部状态
    // 两块 14→40 转接板的 D0/D1 顺序相反，实机必须为 1。
    // 提成参数是为了让 testbench 能传 0 来按自然字节序校验。
    parameter LANE_SWAP = 1,
    parameter CROP_EN   = 0            // 0 = 不裁剪，原样转发 2592x1944
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

    // DSI 座 I2C（接 A 板，40P 第 3/6 脚）—— A 板用它控制摄像头
    input  wire        I_dsi_scl,          // P11
    inout  wire        IO_dsi_sda,         // R12

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

    //------------------------------------------------------------------
    // 摄像头配置：B 板自己做初始化 + A 板控制曝光
    //   · 初始化表 = B 板的 F02 1920x1080 表（HTS=2852, VTS=1127, 20fps）
    //     —— B 自己配，保证模式和帧率稳定
    //   · A 板通过 40P 第 3/6 脚发来的 AE 写，只提取曝光/增益数值，
    //     喂给 B 自己的 AE 表（5 条，不写 VTS），不破坏已配好的模式
    //------------------------------------------------------------------
    wire        cam_cfg_done;
    wire        cam_ae_done;
    wire        iic_busy_raw;
    wire        iic_ack_raw;
    wire        iic_err_raw;
    wire [8:0]  dbg_regidx;
    wire [15:0] bridge_ok_cnt  = 16'd0;   // 调试字段占位
    wire [15:0] bridge_vts_drop= 16'd0;
    wire        vts_seen       = 1'b0;
    wire [7:0]  sn_301f = 8'hA5, sn_320c = 8'hA5, sn_320d = 8'hA5;
    wire [15:0] first_reg  = 16'hA5A5;
    wire [7:0]  first_data = 8'hA5;

    // ---- DSI 侧从机：收 A 板的寄存器写 ----
    wire [15:0] dsi_reg;
    wire [7:0]  dsi_data;
    wire        dsi_valid;
    wire        dsi_seen;

    i2c_slave #( .DEV_ID ( 8'h6C ) ) u_dsi_slave (
        .clk       ( clk_24m    ),
        .rstn      ( rst_n_24   ),
        .scl_i     ( I_dsi_scl  ),
        .sda_io    ( IO_dsi_sda ),
        .o_reg     ( dsi_reg    ),
        .o_data    ( dsi_data   ),
        .o_valid   ( dsi_valid  ),
        .o_seen    ( dsi_seen   ),
        .dbg_start (            ),
        .dbg_stop  (            )
    );

    // ---- 从 A 的写里提取曝光/增益 ----
    wire [15:0] ae_from_a;
    wire [15:0] ag_from_a;
    wire        ae_req_from_a;

    ae_from_dsi u_ae_from_dsi (
        .I_clk      ( clk_24m        ),
        .I_rstn     ( rst_n_24       ),
        .I_reg      ( dsi_reg        ),
        .I_data     ( dsi_data       ),
        .I_valid    ( dsi_valid      ),
        .I_cfg_done ( cam_cfg_done   ),
        .O_ae       ( ae_from_a      ),
        .O_ag       ( ag_from_a      ),
        .O_req      ( ae_req_from_a  )
    );

    uicfgcs500 #(
        .CLK_DIV ( 24000000 / 100000 - 1 )      // 24MHz -> 100kHz
    ) u_cam_cfg (
        .I_clk        ( clk_24m        ),
        .I_rst_n      ( rst_n_24       ),
        .I_ae_req     ( ae_req_from_a  ),       // A 板触发
        .I_ae         ( ae_from_a      ),       // A 板给的曝光
        .I_ag         ( ag_from_a      ),       // A 板给的增益
        .O_cam_scl    ( O_cam_scl      ),
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
    reg        fs_det;
    reg [19:0] fe_hold;

    // ★ 真帧边界是一个"只有 2 拍"的短包。只看首字节会把杂散突发误判成
    //   帧起/帧尾（2592 模式下行包被误判后 cam_fs_seen 被永久置 1，
    //   按行数合成的逻辑就被关掉 -> A 永远收不到帧边界 -> 黑屏）。
    //   这里同时记录突发长度，只有长度正好 2 拍才认。
    reg [15:0] bst_len;      // 当前突发已收到的拍数
    reg [7:0]  bst_hi;       // 当前突发的第一个字节（高字节）
    reg        bst_act;      // 当前是否在一个突发里

    // 去抖：真正的帧间隔约 33ms，同一次突发内误触发间隔 <1us
    localparam FE_DEB = 20'd20000;

    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            v_d2    <= 1'b0;
            fe_det  <= 1'b0;
            fs_det  <= 1'b0;
            fe_hold <= 20'd0;
            bst_len <= 16'd0;
            bst_hi  <= 8'd0;
            bst_act <= 1'b0;
        end else begin
            v_d2   <= rx_valid;
            fe_det <= 1'b0;
            fs_det <= 1'b0;

            //---- 突发计数：有效期间累加，结束后判决 ----
            if (rx_valid) begin
                if (!bst_act) begin
                    bst_hi  <= rx_data[15:8];   // 记下突发首字节
                    bst_len <= 16'd1;
                    bst_act <= 1'b1;
                end else if (bst_len != 16'hFFFF) begin
                    bst_len <= bst_len + 16'd1;
                end
            end else if (bst_act) begin
                // 突发刚结束：长度 2 拍 + 首字节是 00/01 才算真短包
                bst_act <= 1'b0;
                if (bst_len == 16'd2) begin
                    if (bst_hi == 8'h01 && fe_hold == 20'd0) begin
                        fe_det  <= 1'b1;
                        fe_hold <= FE_DEB;
                    end else if (bst_hi == 8'h00) begin
                        fs_det  <= 1'b1;
                    end
                end
            end

            if (fe_hold != 20'd0) fe_hold <= fe_hold - 20'd1;
        end
    end

    reg        frm_on;
    reg        fe_rq;
    reg        fs_rq;
    reg        gen_fs;
    reg        gen_fe;

    //------------------------------------------------------------------
    // 帧边界合成
    //   SC500CS 在 2592x1944 模式下不发 FS/FE 短包（1920 模式下会发）。
    //   A 板靠 FS/FE 划分帧，收不到就判每帧行数错 -> 黑屏。
    //   所以 B 按行数自己补：开帧后数满 FRAME_LINES 行就发一次 FE+FS。
    //------------------------------------------------------------------
    // 注意：这是 B 每帧实际转发的行数，必须等于裁剪后的 CROP_LINES（1080）。
    // 不能写成摄像头的 1944 —— 那样两帧之间会夹 1.8 个源帧，
    // A 板数到的行数永远对不上，每帧都判坏（LED3 亮、LED1 灭、黑屏）。
    // 全画幅 2592x1944：每帧 1944 行，必须和 A 的 SENSOR_HEIGHT 一致。
    // （之前是 1080，因为那时 CROP_EN=1 输出 1920x1080；改成不裁剪后忘了改回来，
    //   导致 B 每 1080 行就发一次 FE/FS，A 判 lines_now != 1944 -> 每帧判坏 -> LED3 亮）
    localparam integer FRAME_LINES = 1944;

    reg [11:0] frm_line;
    reg        cam_fs_seen;   // 摄像头发过 FS 就交给它；超时没再发就改自己补
    reg [21:0] fs_timeout;    // 约 37ms（112.5MHz 下 4.19M 拍）≈ 一帧

    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            frm_on   <= 1'b0;
            fe_rq    <= 1'b0;
            fs_rq    <= 1'b0;
            gen_fs   <= 1'b0;
            gen_fe   <= 1'b0;
            frm_line <= 12'd0;
            cam_fs_seen <= 1'b0;
            fs_timeout  <= 22'd0;
        end else begin
            gen_fs <= 1'b0;
            gen_fe <= 1'b0;

            // 帧边界完全由 B 按行数生成，不使用摄像头的 FS/FE。
            // 原因：RX 的 rx_valid 会在突发中间短暂断开，
            // 导致子突发被误数成 2 拍、fs_det/fe_det 频繁误触发
            // （实测产生 1002 帧/秒，而正确值是 33.7），
            // A 每帧收到 65 行就被判坏 -> 黑屏。
            cam_fs_seen <= 1'b0;
            fs_timeout  <= 22'd0;

            // 摄像头不发 FS 时，靠行数开帧/闭帧
            // ★ 关键：帧边界标记必须排队到"下一拍"再发，绝不能和 p_pkt_end 同拍。
            //   令牌打包的优先级是 p_wr_en > gen_fs > p_pkt_end > gen_fe，
            //   同拍时 gen_fe 会被 p_pkt_end 挤掉、gen_fs 会把该行的行结束挤掉，
            //   结果 A 板收不到帧起/帧尾 -> 永远凑不满一帧 -> 黑屏。
            if (p_pkt_end) begin
                if (!frm_on) begin
                    fs_rq    <= 1'b1;      // 下一拍补帧起
                    frm_on   <= 1'b1;
                    frm_line <= 12'd0;
                end else if (frm_line >= FRAME_LINES - 1) begin
                    fe_rq    <= 1'b1;      // 下一拍发帧尾，再下一拍发帧起
                    frm_line <= 12'd0;
                end else begin
                    frm_line <= frm_line + 12'd1;
                end
            end

            // 不使用摄像头的 FE（同上，避免误触发）

            if (fe_rq) begin
                gen_fe <= 1'b1;
                fe_rq  <= 1'b0;
                fs_rq  <= 1'b1;
            end else if (fs_rq) begin
                gen_fs <= 1'b1;
                fs_rq  <= 1'b0;
                frm_line <= 12'd0;
            end
        end
    end
    //------------------------------------------------------------------
    // 令牌打包
    //   CROP_EN=0 : 原样转发（仿真用，行为和裁剪前完全一致）
    //   CROP_EN=1 : 只转发 1920x1080 窗口（裁掉右侧 672 列、下方 864 行）
    //     摄像头 2592x1944@30fps 载荷 750Mbps/lane，链路利用率 83%，
    //     重打包开销挤不进去；裁到 1920x1080 后只有 311Mbps/lane，
    //     利用率 35%，余量充足。A 板回到已验证的 1920x1080 配置。
    //------------------------------------------------------------------
    localparam integer CROP_W     = 1920;                 // 保留列数
    localparam integer CROP_WWD   = CROP_W * 10 / 16;     // 每行 16bit 字数 = 1200
    localparam integer CROP_WC    = CROP_W * 10 / 8;      // 每行字节数   = 2400
    localparam integer CROP_LINES = 1080;                 // 保留行数
    localparam integer SRC_LINES  = 1944;                 // 摄像头每帧行数

    genvar gi;
    generate
    if (!CROP_EN) begin : g_nocrop
        //================= 原样转发（原代码，未改动）=================
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
    end else begin : g_crop
        //================= 裁剪到 1920x1080 =================
        reg [11:0] col_w;        // 当前行已处理到的字数
        reg [11:0] row_idx;      // 当前帧已处理到的行号
        wire       row_keep = (row_idx < CROP_LINES);

        always @(posedge rx_clk or negedge rst_n_rx) begin
            if (!rst_n_rx) begin
                tk_we    <= 1'b0;
                tk_wdata <= 18'd0;
                col_w    <= 12'd0;
                row_idx  <= 12'd0;
            end else if (!tk_full) begin
                tk_we <= 1'b1;
                if (p_wr_en) begin
                    if (row_keep && (col_w < CROP_WWD))
                        tk_wdata <= { 2'b00, p_wr_data };    // 窗口内：转发
                    else
                        tk_we <= 1'b0;                        // 窗口外：丢弃
                    if (col_w != 12'hFFF) col_w <= col_w + 12'd1;
                end else if (gen_fs) begin
                    tk_wdata <= { 2'b01, 16'h0000 };
                    col_w    <= 12'd0;
                    row_idx  <= 12'd0;
                end else if (p_pkt_end) begin
                    col_w <= 12'd0;
                    if (row_keep)
                        tk_wdata <= { 2'b10, CROP_WC[15:0] }; // WC 改写为 2400
                    else
                        tk_we <= 1'b0;
                    if (row_idx == SRC_LINES - 1) row_idx <= 12'd0;
                    else                          row_idx <= row_idx + 12'd1;
                end else if (gen_fe) begin
                    tk_wdata <= { 2'b11, 16'h0000 };
                end else begin
                    tk_we <= 1'b0;
                end
            end else begin
                tk_we <= 1'b0;
            end
        end
    end
    endgenerate

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
        .AW ( 12 )          // 4096 token：2592 宽的行约 1640 word，双缓冲需 ~3300
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
        .BUF_AW   ( 12     )
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
        .CLK_LANE_MODE         ( "INTERRUPT" ),  // 非连续时钟：消隐期停时钟，和摄像头一致。A 的 RX 可能靠 LP→HS 跳变重新同步。
        .HS_OUTPUT_VOD         ( "250mV"    ),   // 900Mbps 下取最大值（合法上限 250mV）
        .HS_OUTPUT_IMPEDANCE   ( "50ohm"    ),
        .HS_OUTPUT_DE_EMPHASIS ( "3dB"      ),   // 预加重补偿线缆损耗
        .CK_OUTPUT_DELAY       ( 0          ),
        .L0_OUTPUT_DELAY       ( 0          ),
        .L1_OUTPUT_DELAY       ( 0          ),
        .L2_OUTPUT_DELAY       ( 0          ),
        .L3_OUTPUT_DELAY       ( 0          ),
        // VCO = 100MHz * (17+1) = 1800MHz -> linerate 900Mbps -> byte clk 112.5MHz
        // 与 A 板 SDC 的 2.222ns(450MHz) 匹配。1050Mbps 会让 A 板时序失败（WNS -926ps）。
        // 与厂家 DSI 例程同值。2592x1944@30fps 载荷 750Mbps/lane，
        // 利用率 71%，比 900Mbps 的 84% 多出足够余量做重打包。
        // 匹配 A 板 DPHY0 RX 的实测字节时钟 112MHz（时序报告）：
        //   摄像头载荷 2592*1944*29.76*10bit/2lane/8 = 750Mbps/lane，
        //   发 900Mbps 留 20% 余量，字节时钟 112.5MHz 与 A 板一致。
        .DPLL_MULTI_RATIO      ( 17 ),
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

    //------------------------------------------------------------------
    // 收发行的实时计数（不依赖 FS，模态 65536 自动回绕）
    //   tx_line_cnt : TX 实际打包发出的行数  —— 涨说明数据真的在往 A 发
    //   rx_line_cnt : RX 解析出来的行包数     —— 涨说明摄像头数据在进来
    // 这两个都是自由计数，不会饱和，最适合判断链路活没活。
    //------------------------------------------------------------------
    reg [15:0] tx_line_cnt;
    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx)             tx_line_cnt <= 16'd0;
        else if (tx_pkt_end)       tx_line_cnt <= tx_line_cnt + 16'd1;
    end

    reg [15:0] rx_line_cnt;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx)             rx_line_cnt <= 16'd0;
        else if (p_pkt_end)        rx_line_cnt <= rx_line_cnt + 16'd1;
    end

    //------------------------------------------------------------------
    // 行周期测量（单位：rx_clk 拍）
    //   摄像头 29.76fps x 1944 行 => 每行约 1945 拍（rx_clk 112.5MHz）
    //   读数远大于 1945 => 摄像头本身就慢；
    //   接近 1945      => 摄像头是 30fps，是 B 在丢行。
    //------------------------------------------------------------------
    reg [15:0] line_ticks;
    reg [15:0] line_period;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            line_ticks  <= 16'd0;
            line_period <= 16'd0;
        end else if (p_pkt_end) begin
            line_ticks  <= 16'd0;
            line_period <= line_ticks;
        end else if (line_ticks != 16'hFFFF) begin
            line_ticks  <= line_ticks + 16'd1;
        end
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
        .I_st_rxval ( line_period   ),   // V = 行周期（rx_clk 拍，正常约 1945）
        .I_st_werr  ( dbg_werr     ),
        .I_st_gapmax( rx_line_cnt   ),   // A = RX 已解析的行数
        .I_st_pktcnt( tx_line_cnt   ),   // T = TX 已发出的行数
        .O_uart_tx  ( O_uart_tx    )
    );

endmodule
