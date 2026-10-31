//==========================================================================
// csi2_tx_top.v
//   B 板（MLKPAI-F02 / PH1P35MDG324）CSI-2 中继顶层
//
//   数据流：
//     SC500CS 摄像头 --2lane RAW10--> [CSI 座] DPHY0 RX
//        -> csi2_rx_parse（RX 时钟域）
//        -> RX 域裁剪/缩放为 1920x1080 RAW10 -> DDR 三帧缓存
//        -> DDR 读侧并行给 A 板 relay 和 CNN（CNN 再缩放到 320x180）
//        -> 1920x1080 relay -> 异步 FIFO -> CSI-2 发送包化
//        -> 令牌解包（TX 时钟域）
//        -> csi2_tx_packet -> DPHY1 TX -> [DSI 座] ==> A 板 J9
//
//   时钟：
//     I_sys_clk 25MHz -> PLL -> clk0=100MHz（逻辑/DPLL 参考）、clk1=24MHz（摄像头 XCLK）
//     RX 字节时钟由摄像头 MIPI 时钟恢复（O_hs_rx_clk）
//     TX 字节时钟由 DPHY1 内部 DPLL 产生（当前参数目标 75MHz byte clock）
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
    output wire        O_uart_tx,      // J3：调试串口 115200 8N1

    // PH1P35 内置 DDR2，复用既有 16-bit 控制器与板级约束。
    output wire [12:0] ddr_addr,
    output wire [1:0]  ddr_ba,
    output wire [0:0]  ddr_cke,
    output wire [0:0]  ddr_odt,
    output wire [0:0]  ddr_cs_n,
    output wire        ddr_ras_n,
    output wire        ddr_cas_n,
    output wire        ddr_we_n,
    output wire [0:0]  ddr_ck_p,
    output wire [0:0]  ddr_ck_n,
    inout  wire [1:0]  ddr_dm,
    inout  wire [15:0] ddr_dq,
    inout  wire [1:0]  ddr_dqs_p,
    inout  wire [1:0]  ddr_dqs_n
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

    // Crop/scale once before DDR so both consumers use the same 1920x1080 frame.
    localparam integer SENSOR_WIDTH = 2592;
    localparam integer SENSOR_HEIGHT = 1944;
    localparam integer CACHE_WIDTH = 1920;
    localparam integer CACHE_HEIGHT = 1080;
    localparam integer CACHE_LINE_WORDS16 = CACHE_WIDTH * 10 / 16;
    localparam integer CACHE_FRAME_WORDS16 = CACHE_LINE_WORDS16 * CACHE_HEIGHT;
    localparam [15:0] A_LINE_WC = CACHE_WIDTH * 10 / 8;
    wire ddr_clk;
    wire ddr_init_done;
    wire raw_line_end = p_pkt_end && (p_pkt_dt[5:0] == 6'h2B);
    wire raw_capture_frame_end;
    csi2_raw_frame_sync #(.FRAME_LINES(SENSOR_HEIGHT)) u_source_frame_sync (
        .clk(rx_clk), .rst_n(rst_n_rx),
        .frame_start(p_frm_start), .frame_end(p_frm_end),
        .word_valid(p_wr_en), .line_end(raw_line_end),
        .frame_boundary(raw_capture_frame_end)
    );

    wire        crop_wr_en;
    wire [15:0] crop_wr_data;
    wire        crop_frame_start;
    wire        crop_line_end;
    wire [15:0] crop_word_count;
    wire        crop_frame_end;
    wire        crop_format_error;

    csi2_raw10_scale_crop #(
        .SRC_WIDTH    ( 2592 ),
        .SRC_HEIGHT   ( 1944 ),
        .SRC_CROP_X   ( 8    ),
        .SCALE_WIDTH  ( 1920 ),
        .SCALE_HEIGHT ( 1440 ),
        .OUT_HEIGHT   ( 1080 ),
        .CROP_Y       ( 180  )
    ) u_crop_before_ddr (
        .clk            ( rx_clk ),
        .rst            ( ~rst_n_rx ),
        .in_valid       ( p_wr_en ),
        .in_data        ( p_wr_data ),
        .in_line_end    ( raw_line_end ),
        .in_word_count  ( p_pkt_wc ),
        .in_data_type   ( 8'h2B ),
        .in_frame_end   ( raw_capture_frame_end ),
        .out_valid       ( crop_wr_en ),
        .out_data        ( crop_wr_data ),
        .out_frame_start ( crop_frame_start ),
        .out_line_end    ( crop_line_end ),
        .out_word_count  ( crop_word_count ),
        .out_frame_end   ( crop_frame_end ),
        .format_error    ( crop_format_error )
    );

    wire        ddr_cached_frame_start;
    wire        ddr_cached_word_valid;
    wire [15:0] ddr_cached_word_data;
    wire        ddr_cached_line_end;
    wire        ddr_cached_frame_end;

    // Both the A-board relay and CNN consume this processed DDR read stream.
    wire        cnn_frame_start;
    wire        cnn_pixel_valid;
    wire [7:0]  cnn_pixel_data;
    wire        cnn_frame_complete;
    wire [9:0]  cnn_pixel_x;
    wire [8:0]  cnn_pixel_y;
    wire        cnn_format_error;

    cnn_raw10_resize_bin #(
        .THRESHOLD ( 8'd128 ),
        .SRC_WIDTH ( CACHE_WIDTH ),
        .SRC_HEIGHT( CACHE_HEIGHT ),
        .DST_WIDTH ( 320 ),
        .DST_HEIGHT( 180 )
    ) u_cnn_resize_bin (
        .clk          ( ddr_clk              ),
        .rst          ( ~sys_rst_n           ),
        .in_valid     ( ddr_cached_word_valid ),
        .in_data      ( ddr_cached_word_data ),
        .in_line_end  ( ddr_cached_line_end  ),
        .in_frame_end ( ddr_cached_frame_end ),
        .frame_start  ( cnn_frame_start ),
        .pixel_valid  ( cnn_pixel_valid ),
        .pixel_data   ( cnn_pixel_data ),
        .frame_complete( cnn_frame_complete ),
        .pixel_x      ( cnn_pixel_x ),
        .pixel_y      ( cnn_pixel_y ),
        .format_error ( cnn_format_error )
    );

    wire        cnn_digit_done;
    wire [3:0]  cnn_digit_class;
    wire [9:0]  cnn_digit_x;
    wire [8:0]  cnn_digit_y;
    wire [9:0]  cnn_digit_w;
    wire [8:0]  cnn_digit_h;
    wire        cnn_detector_frame_done;
    wire [4:0]  cnn_digit_count;
    wire        cnn_detector_busy;
    wire        cnn_model_check_done;
    wire [31:0] cnn_model_weight_crc32;
    reg         cnn_result_toggle;
    reg [9:0]   cnn_result_x, cnn_result_w;
    reg [8:0]   cnn_result_y, cnn_result_h;
    reg [3:0]   cnn_result_class;

    always @(posedge clk_100m or negedge rst_n_100) begin
        if (!rst_n_100) begin
            cnn_result_toggle <= 1'b0;
            cnn_result_x <= 10'd0; cnn_result_y <= 9'd0;
            cnn_result_w <= 10'd0; cnn_result_h <= 9'd0;
            cnn_result_class <= 4'd0;
        end else if (cnn_digit_done) begin
            cnn_result_x <= cnn_digit_x;
            cnn_result_y <= cnn_digit_y;
            cnn_result_w <= cnn_digit_w;
            cnn_result_h <= cnn_digit_h;
            cnn_result_class <= cnn_digit_class;
            cnn_result_toggle <= ~cnn_result_toggle;
        end
    end

    cnn_frame_digit_detector #(
        .DARK_THRESHOLD(8'd128),
        .MAX_BANDS(8),
        .MAX_DIGITS(16)
    ) u_cnn_detector (
        .capture_clk  ( ddr_clk      ),
        .infer_clk    ( clk_100m     ),
        .rst          ( ~sys_rst_n   ),
        .frame_start  ( cnn_frame_start ),
        .pixel_valid  ( cnn_pixel_valid ),
        .pixel_data   ( cnn_pixel_data ),
        .frame_complete( cnn_frame_complete ),
        .digit_done   ( cnn_digit_done ),
        .digit_class  ( cnn_digit_class ),
        .digit_x      ( cnn_digit_x ),
        .digit_y      ( cnn_digit_y ),
        .digit_w      ( cnn_digit_w ),
        .digit_h      ( cnn_digit_h ),
        .frame_done   ( cnn_detector_frame_done ),
        .frame_digit_count( cnn_digit_count ),
        .busy         ( cnn_detector_busy ),
        .model_check_done( cnn_model_check_done ),
        .model_weight_crc32( cnn_model_weight_crc32 ),
        .digit_patch_crc32(),
        .digit_conv1_signature(),
        .digit_pool1_signature(),
        .digit_conv2_signature(),
        .digit_pool2_signature(),
        .digit_conv3_signature(),
        .digit_pool3_signature(),
        .digit_conv1_input_signature(),
        .digit_conv1_weight_signature(),
        .digit_conv1_product_signature(),
        .digit_conv1_accumulator_signature(),
        .digit_conv1_raw_signature(),
        .digit_conv1_channel_signatures(),
        .digit_pool1_channel_signatures(),
        .digit_conv1_first_weight(),
        .digit_conv1_first_activation(),
        .digit_conv1_first_product()
    );

    // The resized relay stream crosses to the DPHY TX clock after DDR playback.
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

    //==================================================================
    // 6. DPHY1 发送
    //==================================================================
    wire tx_clk;
    wire tx_ready;
    wire tx_dpll_locked;
    wire tx_rst_src_n = sys_rst_n & tx_dpll_locked;

    reg [1:0] rsync_txq;
    always @(posedge tx_clk or negedge tx_rst_src_n)
        if (!tx_rst_src_n) rsync_txq <= 2'b00; else rsync_txq <= { rsync_txq[0], 1'b1 };
    wire rst_n_tx = rsync_txq[1];

    wire        ddr_user_wr_en;
    wire [24:0] ddr_user_wr_addr;
    wire [127:0] ddr_user_wr_data;
    wire        ddr_user_rd_en;
    wire [24:0] ddr_user_rd_addr;
    wire        ddr_user_ready;
    wire        ddr_user_rd_valid;
    wire [127:0] ddr_user_rd_data;
    wire        mc_app_en;
    wire [24:0] mc_app_addr;
    wire [2:0]  mc_app_cmd;
    wire [127:0] mc_app_wdf_data;
    wire [15:0]  mc_app_wdf_mask;
    wire        mc_app_wdf_wren;
    wire        mc_app_wdf_end;
    wire        mc_app_rdy;
    wire        mc_app_wdf_rdy;
    wire [127:0] mc_app_rd_data;
    wire        mc_app_rd_data_end;
    wire        mc_app_rd_data_valid;
    wire        tx_token_empty;
    wire [17:0] tx_token_data;
    wire        tx_token_re;
    wire        ddr_frame_busy;
    wire        ddr_frame_error;
    wire [15:0] ddr_dropped_frames;
    wire [15:0] ddr_committed_frames;
    wire [1:0]  ddr_ready_count;
    wire        relay_token_full;

    ddr_triple_frame_cache #(
        .FRAME_WORDS16   ( CACHE_FRAME_WORDS16 ),
        .LINE_WORDS16    ( CACHE_LINE_WORDS16 ),
        .FRAME_LINES     ( CACHE_HEIGHT ),
        .FRAME_PACK_WORDS( CACHE_FRAME_WORDS16 / 8 ),
        .FRAME_ADDR_SPAN ( CACHE_FRAME_WORDS16 )
    ) u_ddr_triple_frame_cache (
        .rx_clk             ( rx_clk              ),
        .rx_rst_n           ( rst_n_rx            ),
        .rx_frame_start     ( crop_frame_start    ),
        .rx_word_valid      ( crop_wr_en          ),
        .rx_word_data       ( crop_wr_data        ),
        .rx_line_end        ( crop_line_end       ),
        .rx_frame_end       ( crop_frame_end      ),
        .ddr_clk            ( ddr_clk             ),
        .ddr_rst_n          ( sys_rst_n           ),
        .ddr_init_done      ( ddr_init_done       ),
        .ddr_user_wr_en     ( ddr_user_wr_en      ),
        .ddr_user_wr_addr   ( ddr_user_wr_addr    ),
        .ddr_user_wr_data   ( ddr_user_wr_data    ),
        .ddr_user_rd_en     ( ddr_user_rd_en      ),
        .ddr_user_rd_addr   ( ddr_user_rd_addr    ),
        .ddr_user_ready     ( ddr_user_ready      ),
        .ddr_user_rd_valid  ( ddr_user_rd_valid   ),
        .ddr_user_rd_data   ( ddr_user_rd_data    ),
        .out_ready          ( !relay_token_full   ),
        .out_frame_start    ( ddr_cached_frame_start ),
        .out_word_valid     ( ddr_cached_word_valid  ),
        .out_word_data      ( ddr_cached_word_data   ),
        .out_line_end       ( ddr_cached_line_end    ),
        .out_frame_end      ( ddr_cached_frame_end   ),
        .frame_busy         ( ddr_frame_busy      ),
        .frame_error        ( ddr_frame_error     ),
        .dropped_frames     ( ddr_dropped_frames  ),
        .committed_frames   ( ddr_committed_frames),
        .ready_count        ( ddr_ready_count     )
    );

    wire relay_token_event = ddr_cached_word_valid || ddr_cached_frame_start ||
                             ddr_cached_line_end || ddr_cached_frame_end;
    reg relay_token_overflow;
    reg [17:0] relay_token_data;
    always @* begin
        if (ddr_cached_word_valid)
            relay_token_data = {2'b00, ddr_cached_word_data};
        else if (ddr_cached_frame_start)
            relay_token_data = {2'b01, 16'd0};
        else if (ddr_cached_line_end)
            relay_token_data = {2'b10, A_LINE_WC};
        else
            relay_token_data = {2'b11, 16'd0};
    end
    always @(posedge ddr_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            relay_token_overflow <= 1'b0;
        else if (relay_token_event && relay_token_full)
            relay_token_overflow <= 1'b1;
    end
    csi2_ptr_fifo #(
        .DW(18),
        .AW(12),
        .USE_ERAM(1)
    ) u_relay_tx_token_fifo (
        .I_wclk(ddr_clk),
        .I_wrstn(sys_rst_n),
        .I_we(relay_token_event && !relay_token_full),
        .I_wdata(relay_token_data),
        .O_wfull(relay_token_full),
        .I_rclk(tx_clk),
        .I_rrstn(rst_n_tx),
        .I_re(tx_token_re),
        .O_rdata(tx_token_data),
        .O_rempty(tx_token_empty)
    );

    mc_to_user_interface u_mc_to_user_interface (
        .I_clk                   ( ddr_clk              ),
        .I_rst_n                 ( sys_rst_n            ),
        .I_ddr_user_wr_en        ( ddr_user_wr_en       ),
        .I_ddr_user_rd_en        ( ddr_user_rd_en       ),
        .I_ddr_user_addr         ( ddr_user_wr_en ? ddr_user_wr_addr : ddr_user_rd_addr ),
        .I_ddr_user_wr_data      ( ddr_user_wr_data     ),
        .O_ddr_user_ready        ( ddr_user_ready       ),
        .O_ddr_user_rd_valid     ( ddr_user_rd_valid    ),
        .O_ddr_user_rd_data      ( ddr_user_rd_data     ),
        .O_mc_app_en             ( mc_app_en            ),
        .O_mc_app_addr           ( mc_app_addr          ),
        .O_mc_app_cmd            ( mc_app_cmd           ),
        .I_mc_app_rdy            ( mc_app_rdy           ),
        .O_mc_app_wdf_wren       ( mc_app_wdf_wren      ),
        .O_mc_app_wdf_data       ( mc_app_wdf_data      ),
        .O_mc_app_wdf_end        ( mc_app_wdf_end       ),
        .O_mc_app_wdf_mask       ( mc_app_wdf_mask      ),
        .I_mc_app_wdf_rdy        ( mc_app_wdf_rdy       ),
        .I_mc_app_rd_data        ( mc_app_rd_data       ),
        .I_mc_app_rd_data_end    ( mc_app_rd_data_end   ),
        .I_mc_app_rd_data_valid  ( mc_app_rd_data_valid )
    );

    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper (
        .I_sys_clk              ( clk_100m             ),
        .I_sys_rst_n            ( sys_rst_n            ),
        .O_ddr_clk              ( ddr_clk              ),
        .O_init_calib_complete  ( ddr_init_done        ),
        .I_mc_app_addr          ( mc_app_addr          ),
        .I_mc_app_cmd           ( mc_app_cmd           ),
        .I_mc_app_en            ( mc_app_en            ),
        .I_mc_app_wdf_data      ( mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( mc_app_wdf_rdy       ),
        .ddr_addr               ( ddr_addr              ),
        .ddr_ba                 ( ddr_ba                ),
        .ddr_cke                ( ddr_cke               ),
        .ddr_odt                ( ddr_odt               ),
        .ddr_cs_n               ( ddr_cs_n              ),
        .ddr_ras_n              ( ddr_ras_n             ),
        .ddr_cas_n              ( ddr_cas_n             ),
        .ddr_we_n               ( ddr_we_n              ),
        .ddr_ck_p               ( ddr_ck_p              ),
        .ddr_ck_n               ( ddr_ck_n              ),
        .ddr_dm                 ( ddr_dm                ),
        .ddr_dq                 ( ddr_dq                ),
        .ddr_dqs_p              ( ddr_dqs_p             ),
        .ddr_dqs_n              ( ddr_dqs_n             )
    );

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
    wire        tx_pkt_dbg_done;
    wire [7:0]  tx_pkt_dbg_dt;
    wire [15:0] tx_pkt_dbg_wc;
    wire        tx_packet_data_ready;
    wire        tx_packet_pkt_ready;

    // DDR cache returns {type[1:0], data[15:0]} tokens to the packetizer.
    assign tk_empty = tx_token_empty;
    assign tk_rdata = tx_token_data;
    assign tk_re = !tk_empty &&
                   ((tk_rdata[17:16] == 2'b00) ? tx_packet_data_ready :
                    (tk_rdata[17:16] == 2'b10) ? tx_packet_pkt_ready : 1'b1);
    assign tx_token_re = tk_re;

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

            if (tx_token_re) begin
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

    wire tx_ovf;
    wire tx_wcerr;

    // 行缓冲 + CSI-2 打包（B 板后续要在此处插入识别/OSD，需保留像素流）
    csi2_tx_packet #(
        .LANE_NUM ( LANES  ),
        .VC       ( 2'b00  ),
        .DT       ( 8'h2B  ),
        .LANE_SWAP( LANE_SWAP ),
        .BUF_AW   ( 11     ),
        .USE_ERAM_BUFFER(1)
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
        .O_wc_err     ( tx_wcerr    ),
        .O_data_ready ( tx_packet_data_ready ),
        .O_pkt_ready  ( tx_packet_pkt_ready ),
        .O_dbg_pkt_done( tx_pkt_dbg_done ),
        .O_dbg_pkt_dt  ( tx_pkt_dbg_dt   ),
        .O_dbg_pkt_wc  ( tx_pkt_dbg_wc   )
    );

    reg [15:0] tx_packets_run, tx_lines_run, tx_wc_run;
    reg [31:0] tx_bytes_run;
    reg [15:0] tx_packets_frame, tx_lines_frame, tx_wc_frame;
    reg [31:0] tx_bytes_frame;
    reg        tx_error_frame, tx_stats_toggle;
    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx) begin
            tx_packets_run <= 16'd0;
            tx_lines_run <= 16'd0;
            tx_wc_run <= 16'd0;
            tx_bytes_run <= 32'd0;
            tx_packets_frame <= 16'd0;
            tx_lines_frame <= 16'd0;
            tx_wc_frame <= 16'd0;
            tx_bytes_frame <= 32'd0;
            tx_error_frame <= 1'b0;
            tx_stats_toggle <= 1'b0;
        end else if (tx_pkt_dbg_done) begin
            case (tx_pkt_dbg_dt[5:0])
            6'h00: begin
                tx_packets_run <= 16'd1;
                tx_lines_run <= 16'd0;
                tx_wc_run <= 16'd0;
                tx_bytes_run <= 32'd4;
            end
            6'h2B: begin
                if (tx_packets_run != 16'hFFFF)
                    tx_packets_run <= tx_packets_run + 16'd1;
                if (tx_lines_run != 16'hFFFF)
                    tx_lines_run <= tx_lines_run + 16'd1;
                tx_wc_run <= tx_pkt_dbg_wc;
                tx_bytes_run <= tx_bytes_run + 32'd6 + {16'd0, tx_pkt_dbg_wc};
            end
            6'h01: begin
                tx_packets_frame <= tx_packets_run + 16'd1;
                tx_lines_frame <= tx_lines_run;
                tx_wc_frame <= tx_wc_run;
                tx_bytes_frame <= tx_bytes_run + 32'd4;
                tx_error_frame <= tx_ovf | tx_wcerr;
                tx_stats_toggle <= ~tx_stats_toggle;
                tx_packets_run <= 16'd0;
                tx_lines_run <= 16'd0;
                tx_wc_run <= 16'd0;
                tx_bytes_run <= 32'd0;
            end
            default: begin
                if (tx_packets_run != 16'hFFFF)
                    tx_packets_run <= tx_packets_run + 16'd1;
            end
            endcase
        end
    end

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
    assign O_led_err  = DBG_MODE ? err_lat          :
                        (tx_ovf | tx_wcerr | crop_format_error |
                         relay_token_overflow | ddr_frame_error |
                         ~ddr_init_done | ~tx_dpll_locked);

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
        // Match the previously working B-to-A link: 100MHz * (11+1) / 16
        // = 75MHz byte clock, or 600Mbps per lane.
        .DPLL_MULTI_RATIO      ( 11 ),
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

    // 解析器识别到的帧起/帧尾计数。短包不会产生 p_pkt_end，需统计边界脉冲。
    reg [15:0] fs_cnt, fe_cnt;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            fs_cnt <= 16'd0;
            fe_cnt <= 16'd0;
        end else begin
            if (p_frm_start && fs_cnt != 16'hFFFF)
                fs_cnt <= fs_cnt + 16'd1;
            if (p_frm_end && fe_cnt != 16'hFFFF)
                fe_cnt <= fe_cnt + 16'd1;
        end
    end

    reg [15:0] raw_fe_cnt;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) raw_fe_cnt <= 16'd0;
        else if (fe_det && raw_fe_cnt != 16'hFFFF)
            raw_fe_cnt <= raw_fe_cnt + 16'd1;
    end

    // Per-frame CSI RX and scaler summaries. These are published only at FE.
    reg [15:0] rx_rows_run, rx_bad_rows_run, rx_wc_run;
    reg [15:0] rx_rows_frame, rx_bad_rows_frame, rx_wc_frame;
    reg        rx_parse_err_frame, rx_stats_toggle;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            rx_rows_run <= 16'd0;
            rx_bad_rows_run <= 16'd0;
            rx_wc_run <= 16'd0;
            rx_rows_frame <= 16'd0;
            rx_bad_rows_frame <= 16'd0;
            rx_wc_frame <= 16'd0;
            rx_parse_err_frame <= 1'b0;
            rx_stats_toggle <= 1'b0;
        end else begin
            if (p_pkt_end && p_pkt_dt[5:0] == 6'h2B) begin
                if (rx_rows_run != 16'hFFFF) rx_rows_run <= rx_rows_run + 16'd1;
                rx_wc_run <= p_pkt_wc;
                if (p_pkt_wc != 16'd3240 && rx_bad_rows_run != 16'hFFFF)
                    rx_bad_rows_run <= rx_bad_rows_run + 16'd1;
            end
            if (p_frm_end) begin
                rx_rows_frame <= rx_rows_run;
                rx_bad_rows_frame <= rx_bad_rows_run;
                rx_wc_frame <= rx_wc_run;
                rx_parse_err_frame <= p_wc_err;
                rx_stats_toggle <= ~rx_stats_toggle;
                rx_rows_run <= 16'd0;
                rx_bad_rows_run <= 16'd0;
                rx_wc_run <= 16'd0;
            end
        end
    end

    reg [15:0] scale_rows_run, scale_bad_wc_run, scale_wc_run;
    reg [15:0] scale_rows_frame, scale_wc_frame;
    reg        scale_error_frame, scale_stats_toggle;
    always @(posedge rx_clk or negedge rst_n_rx) begin
        if (!rst_n_rx) begin
            scale_rows_run <= 16'd0;
            scale_bad_wc_run <= 16'd0;
            scale_wc_run <= 16'd0;
            scale_rows_frame <= 16'd0;
            scale_wc_frame <= 16'd0;
            scale_error_frame <= 1'b0;
            scale_stats_toggle <= 1'b0;
        end else begin
            if (crop_line_end) begin
                if (scale_rows_run != 16'hFFFF)
                    scale_rows_run <= scale_rows_run + 16'd1;
                scale_wc_run <= crop_word_count;
                if (crop_word_count != A_LINE_WC && scale_bad_wc_run != 16'hFFFF)
                    scale_bad_wc_run <= scale_bad_wc_run + 16'd1;
            end
            if (crop_frame_end) begin
                scale_rows_frame <= scale_rows_run;
                scale_wc_frame <= scale_wc_run;
                scale_error_frame <= crop_format_error ||
                                     (scale_bad_wc_run != 16'd0) ||
                                     (scale_rows_run != CACHE_HEIGHT);
                scale_stats_toggle <= ~scale_stats_toggle;
                scale_rows_run <= 16'd0;
                scale_bad_wc_run <= 16'd0;
                scale_wc_run <= 16'd0;
            end
        end
    end

    // TX 实际发出的拍数：为 0 就说明一个字节都没发出去
    reg [15:0] txbeat_cnt;
    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx) txbeat_cnt <= 16'd0;
        else if (tx_hs_en && txbeat_cnt != 16'hFFFF) txbeat_cnt <= txbeat_cnt + 16'd1;
    end

    // Saturating counters expose progress from the DDR cache through TX.
    reg [15:0] diag_rd_req_count, diag_rd_rsp_count, diag_cache_word_count;
    reg [15:0] diag_cache_line_count, diag_cache_frame_count;
    reg [15:0] diag_relay_write_count;
    reg        diag_relay_full;
    always @(posedge ddr_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            diag_rd_req_count <= 16'd0;
            diag_rd_rsp_count <= 16'd0;
            diag_cache_word_count <= 16'd0;
            diag_cache_line_count <= 16'd0;
            diag_cache_frame_count <= 16'd0;
            diag_relay_write_count <= 16'd0;
            diag_relay_full <= 1'b0;
        end else begin
            if (ddr_user_rd_en && diag_rd_req_count != 16'hFFFF)
                diag_rd_req_count <= diag_rd_req_count + 16'd1;
            if (ddr_user_rd_valid && diag_rd_rsp_count != 16'hFFFF)
                diag_rd_rsp_count <= diag_rd_rsp_count + 16'd1;
            if (ddr_cached_word_valid && diag_cache_word_count != 16'hFFFF)
                diag_cache_word_count <= diag_cache_word_count + 16'd1;
            if (ddr_cached_line_end && diag_cache_line_count != 16'hFFFF)
                diag_cache_line_count <= diag_cache_line_count + 16'd1;
            if (ddr_cached_frame_end && diag_cache_frame_count != 16'hFFFF)
                diag_cache_frame_count <= diag_cache_frame_count + 16'd1;
            if (relay_token_event && !relay_token_full &&
                diag_relay_write_count != 16'hFFFF)
                diag_relay_write_count <= diag_relay_write_count + 16'd1;
            if (relay_token_full) diag_relay_full <= 1'b1;
        end
    end

    reg [15:0] diag_relay_read_count, diag_tx_word_count, diag_tx_line_count;
    reg        diag_tx_error;
    always @(posedge tx_clk or negedge rst_n_tx) begin
        if (!rst_n_tx) begin
            diag_relay_read_count <= 16'd0;
            diag_tx_word_count <= 16'd0;
            diag_tx_line_count <= 16'd0;
            diag_tx_error <= 1'b0;
        end else begin
            if (tx_token_re && diag_relay_read_count != 16'hFFFF)
                diag_relay_read_count <= diag_relay_read_count + 16'd1;
            if (tx_wr_en && diag_tx_word_count != 16'hFFFF)
                diag_tx_word_count <= diag_tx_word_count + 16'd1;
            if (tx_pkt_dbg_done && tx_pkt_dbg_dt[5:0] == 6'h2B &&
                diag_tx_line_count != 16'hFFFF)
                diag_tx_line_count <= diag_tx_line_count + 16'd1;
            if (tx_ovf || tx_wcerr) diag_tx_error <= 1'b1;
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
    reg [15:0] dbg_rxfs;
    reg [15:0] dbg_rawfe;
    reg [15:0] dbg_txcnt;
    reg [15:0] dbg_rx_rows, dbg_rx_wc, dbg_rx_bad_rows;
    reg        dbg_rx_parse_err;
    reg [15:0] dbg_scale_rows, dbg_scale_wc;
    reg        dbg_scale_err;
    reg [15:0] dbg_tx_packets, dbg_tx_lines, dbg_tx_wc;
    reg [31:0] dbg_tx_bytes;
    reg        dbg_tx_err;

    (* ASYNC_REG = "TRUE" *) reg diag_init_meta, diag_init_sync;
    (* ASYNC_REG = "TRUE" *) reg diag_busy_meta, diag_busy_sync;
    (* ASYNC_REG = "TRUE" *) reg diag_error_meta, diag_error_sync;
    (* ASYNC_REG = "TRUE" *) reg diag_full_meta, diag_full_sync;
    (* ASYNC_REG = "TRUE" *) reg diag_dpll_meta, diag_dpll_sync;
    (* ASYNC_REG = "TRUE" *) reg diag_ready_meta, diag_ready_sync;
    (* ASYNC_REG = "TRUE" *) reg diag_txerr_meta, diag_txerr_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] diag_ready_count_meta, diag_ready_count_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_commit_meta, diag_commit_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_drop_meta, diag_drop_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_rd_req_meta, diag_rd_req_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_rd_rsp_meta, diag_rd_rsp_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_cache_word_meta, diag_cache_word_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_cache_line_meta, diag_cache_line_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_cache_frame_meta, diag_cache_frame_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_relay_write_meta, diag_relay_write_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_relay_read_meta, diag_relay_read_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_tx_word_meta, diag_tx_word_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] diag_tx_line_meta, diag_tx_line_sync;

    always @(posedge clk_24m or negedge rst_n_24) begin
        if (!rst_n_24) begin
            diag_init_meta <= 1'b0;
            diag_init_sync <= 1'b0;
            diag_busy_meta <= 1'b0;
            diag_busy_sync <= 1'b0;
            diag_error_meta <= 1'b0;
            diag_error_sync <= 1'b0;
            diag_full_meta <= 1'b0;
            diag_full_sync <= 1'b0;
            diag_dpll_meta <= 1'b0;
            diag_dpll_sync <= 1'b0;
            diag_ready_meta <= 1'b0;
            diag_ready_sync <= 1'b0;
            diag_txerr_meta <= 1'b0;
            diag_txerr_sync <= 1'b0;
            diag_ready_count_meta <= 2'b00;
            diag_ready_count_sync <= 2'b00;
            diag_commit_meta <= 16'd0;
            diag_commit_sync <= 16'd0;
            diag_drop_meta <= 16'd0;
            diag_drop_sync <= 16'd0;
            diag_rd_req_meta <= 16'd0;
            diag_rd_req_sync <= 16'd0;
            diag_rd_rsp_meta <= 16'd0;
            diag_rd_rsp_sync <= 16'd0;
            diag_cache_word_meta <= 16'd0;
            diag_cache_word_sync <= 16'd0;
            diag_cache_line_meta <= 16'd0;
            diag_cache_line_sync <= 16'd0;
            diag_cache_frame_meta <= 16'd0;
            diag_cache_frame_sync <= 16'd0;
            diag_relay_write_meta <= 16'd0;
            diag_relay_write_sync <= 16'd0;
            diag_relay_read_meta <= 16'd0;
            diag_relay_read_sync <= 16'd0;
            diag_tx_word_meta <= 16'd0;
            diag_tx_word_sync <= 16'd0;
            diag_tx_line_meta <= 16'd0;
            diag_tx_line_sync <= 16'd0;
        end else begin
            diag_init_meta <= ddr_init_done;
            diag_init_sync <= diag_init_meta;
            diag_busy_meta <= ddr_frame_busy;
            diag_busy_sync <= diag_busy_meta;
            diag_error_meta <= ddr_frame_error;
            diag_error_sync <= diag_error_meta;
            diag_full_meta <= diag_relay_full;
            diag_full_sync <= diag_full_meta;
            diag_dpll_meta <= tx_dpll_locked;
            diag_dpll_sync <= diag_dpll_meta;
            diag_ready_meta <= tx_ready;
            diag_ready_sync <= diag_ready_meta;
            diag_txerr_meta <= diag_tx_error;
            diag_txerr_sync <= diag_txerr_meta;
            diag_ready_count_meta <= ddr_ready_count;
            diag_ready_count_sync <= diag_ready_count_meta;
            diag_commit_meta <= ddr_committed_frames;
            diag_commit_sync <= diag_commit_meta;
            diag_drop_meta <= ddr_dropped_frames;
            diag_drop_sync <= diag_drop_meta;
            diag_rd_req_meta <= diag_rd_req_count;
            diag_rd_req_sync <= diag_rd_req_meta;
            diag_rd_rsp_meta <= diag_rd_rsp_count;
            diag_rd_rsp_sync <= diag_rd_rsp_meta;
            diag_cache_word_meta <= diag_cache_word_count;
            diag_cache_word_sync <= diag_cache_word_meta;
            diag_cache_line_meta <= diag_cache_line_count;
            diag_cache_line_sync <= diag_cache_line_meta;
            diag_cache_frame_meta <= diag_cache_frame_count;
            diag_cache_frame_sync <= diag_cache_frame_meta;
            diag_relay_write_meta <= diag_relay_write_count;
            diag_relay_write_sync <= diag_relay_write_meta;
            diag_relay_read_meta <= diag_relay_read_count;
            diag_relay_read_sync <= diag_relay_read_meta;
            diag_tx_word_meta <= diag_tx_word_count;
            diag_tx_word_sync <= diag_tx_word_meta;
            diag_tx_line_meta <= diag_tx_line_count;
            diag_tx_line_sync <= diag_tx_line_meta;
        end
    end

    wire diag_init = diag_init_sync;
    wire diag_busy = diag_busy_sync;
    wire diag_error = diag_error_sync;
    wire diag_full = diag_full_sync;
    wire diag_dpll = diag_dpll_sync;
    wire diag_ready = diag_ready_sync;
    wire diag_txerr = diag_txerr_sync;
    wire [1:0] diag_ready_count = diag_ready_count_sync;

    reg rx_toggle_meta, rx_toggle_sync, rx_toggle_seen;
    reg scale_toggle_meta, scale_toggle_sync, scale_toggle_seen;
    reg tx_toggle_meta, tx_toggle_sync, tx_toggle_seen;
    always @(posedge clk_24m or negedge rst_n_24) begin
        if (!rst_n_24) begin
            rx_toggle_meta <= 1'b0;
            rx_toggle_sync <= 1'b0;
            rx_toggle_seen <= 1'b0;
            scale_toggle_meta <= 1'b0;
            scale_toggle_sync <= 1'b0;
            scale_toggle_seen <= 1'b0;
            tx_toggle_meta <= 1'b0;
            tx_toggle_sync <= 1'b0;
            tx_toggle_seen <= 1'b0;
            dbg_rx_rows <= 16'd0;
            dbg_rx_wc <= 16'd0;
            dbg_rx_bad_rows <= 16'd0;
            dbg_rx_parse_err <= 1'b0;
            dbg_scale_rows <= 16'd0;
            dbg_scale_wc <= 16'd0;
            dbg_scale_err <= 1'b0;
            dbg_tx_packets <= 16'd0;
            dbg_tx_lines <= 16'd0;
            dbg_tx_wc <= 16'd0;
            dbg_tx_bytes <= 32'd0;
            dbg_tx_err <= 1'b0;
        end else begin
            rx_toggle_meta <= rx_stats_toggle;
            rx_toggle_sync <= rx_toggle_meta;
            if (rx_toggle_sync != rx_toggle_seen) begin
                rx_toggle_seen <= rx_toggle_sync;
                dbg_rx_rows <= rx_rows_frame;
                dbg_rx_wc <= rx_wc_frame;
                dbg_rx_bad_rows <= rx_bad_rows_frame;
                dbg_rx_parse_err <= rx_parse_err_frame;
            end

            scale_toggle_meta <= scale_stats_toggle;
            scale_toggle_sync <= scale_toggle_meta;
            if (scale_toggle_sync != scale_toggle_seen) begin
                scale_toggle_seen <= scale_toggle_sync;
                dbg_scale_rows <= scale_rows_frame;
                dbg_scale_wc <= scale_wc_frame;
                dbg_scale_err <= scale_error_frame;
            end

            tx_toggle_meta <= tx_stats_toggle;
            tx_toggle_sync <= tx_toggle_meta;
            if (tx_toggle_sync != tx_toggle_seen) begin
                tx_toggle_seen <= tx_toggle_sync;
                dbg_tx_packets <= tx_packets_frame;
                dbg_tx_lines <= tx_lines_frame;
                dbg_tx_wc <= tx_wc_frame;
                dbg_tx_bytes <= tx_bytes_frame;
                dbg_tx_err <= tx_error_frame;
            end
        end
    end

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
        dbg_rxfe  <= fe_cnt;
        dbg_rxfs  <= fs_cnt;
        dbg_rawfe <= raw_fe_cnt;
        dbg_txcnt <= txclk_cnt;
        dbg_burst <= burst_last;
        dbg_fs    <= lpf_lat;   // A 字段改为每帧行数
        dbg_fe    <= txbeat_cnt;
        dbg_h0    <= h0_r;
        dbg_h1    <= h1_r;
    end

    reg result_toggle_meta, result_toggle_sync, result_toggle_seen;
    reg result_start;
    always @(posedge clk_24m or negedge rst_n_24) begin
        if (!rst_n_24) begin
            result_toggle_meta <= 1'b0;
            result_toggle_sync <= 1'b0;
            result_toggle_seen <= 1'b0;
            result_start <= 1'b0;
        end else begin
            result_toggle_meta <= cnn_result_toggle;
            result_toggle_sync <= result_toggle_meta;
            result_start <= 1'b0;
            if (result_toggle_sync != result_toggle_seen) begin
                result_toggle_seen <= result_toggle_sync;
                result_start <= 1'b1;
            end
        end
    end

    wire result_uart_busy;
    wire result_uart_tx;
    wire dbg_uart_tx;
    assign O_uart_tx = dbg_uart_tx;
    cnn_result_uart #(
        .CLK_HZ(24000000), .BAUD(115200)
    ) u_cnn_result_uart (
        .clk(clk_24m), .rst_n(rst_n_24), .start(result_start),
        .x(cnn_result_x), .y(cnn_result_y), .w(cnn_result_w), .h(cnn_result_h),
        .class_id(cnn_result_class), .busy(result_uart_busy), .uart_tx(result_uart_tx)
    );

    dbg_uart #(
        .CLK_HZ(24000000), .BAUD(115200), .GAP_MS(100)
    ) u_dbg_uart (
        .I_clk(clk_24m), .I_rst_n(rst_n_24),
        .I_st_cfg(cam_cfg_done), .I_st_wc(dbg_wc), .I_st_dt(dbg_dt),
        .I_st_burst(dbg_burst), .I_st_h0(dbg_h0), .I_st_h1(dbg_h1),
        .I_st_frame(dbg_frame), .I_st_rxval(dbg_rxval), .I_st_werr(dbg_werr),
        .I_st_gapmax(dbg_fs), .I_st_pktcnt(dbg_fe),
        .I_st_rx_fs(dbg_rxfs), .I_st_rx_fe(dbg_rxfe), .I_st_raw_fe(dbg_rawfe),
        .I_st_rx_rows(dbg_rx_rows), .I_st_rx_wc(dbg_rx_wc),
        .I_st_rx_bad_rows(dbg_rx_bad_rows), .I_st_rx_parse_err(dbg_rx_parse_err),
        .I_st_scale_rows(dbg_scale_rows), .I_st_scale_wc(dbg_scale_wc),
        .I_st_scale_err(dbg_scale_err),
        .I_st_tx_packets(dbg_tx_packets), .I_st_tx_lines(dbg_tx_lines),
        .I_st_tx_bytes(dbg_tx_bytes), .I_st_tx_wc(dbg_tx_wc),
        .I_st_tx_err(dbg_tx_err),
        .I_st_ddr_init(diag_init), .I_st_ddr_commit(diag_commit_sync),
        .I_st_ddr_drop(diag_drop_sync), .I_st_ddr_ready_count(diag_ready_count),
        .I_st_ddr_rd_req(diag_rd_req_sync), .I_st_ddr_rd_rsp(diag_rd_rsp_sync),
        .I_st_cache_words(diag_cache_word_sync), .I_st_cache_lines(diag_cache_line_sync),
        .I_st_cache_frames(diag_cache_frame_sync),
        .I_st_relay_writes(diag_relay_write_sync), .I_st_relay_reads(diag_relay_read_sync),
        .I_st_relay_full(diag_full), .I_st_tx_words(diag_tx_word_sync),
        .I_st_tx_long_lines(diag_tx_line_sync), .I_st_tx_dpll_locked(diag_dpll),
        .I_st_tx_ready(diag_ready), .I_st_ddr_busy(diag_busy),
        .I_st_ddr_error(diag_error), .I_st_tx_error_live(diag_txerr),
        .O_uart_tx(dbg_uart_tx)
    );

endmodule
