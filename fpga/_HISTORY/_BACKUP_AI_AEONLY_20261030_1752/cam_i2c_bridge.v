//==========================================================================
// cam_i2c_bridge.v
//   摄像头 I2C 桥：把 A 板发来的寄存器写，原样转发给真正的 SC500CS。
//
//   ┌──────────┐  I2C(0x6C)   ┌──────────────┐  I2C(0x6C)   ┌────────┐
//   │  A 板    │─────────────>│  B 板 FPGA   │─────────────>│ 摄像头 │
//   │ (控制方) │  DSI 座 P11/R12│  本模块      │  CSI 座 T9/U9│        │
//   └──────────┘              └──────────────┘              └────────┘
//
//   数据格式（与 A 板 uicfgcs500 完全一致，A 板无需改动）：
//     START | 0x6C(W) | REG_H | REG_L | DATA | STOP
//
//   结构：
//     i2c_slave  → 收 (reg[15:0], data[7:0])
//     FIFO(256)  → 缓冲，隔离两侧速率
//     转发 FSM   → 调 uii2c 主机逐条写出
//==========================================================================
module cam_i2c_bridge #(
    parameter [7:0]  DEV_ID  = 8'h6C,
    parameter        CLK_DIV = 16'd999,   // I2C 时钟分频（24MHz / (2*(999+1)) ≈ 12kHz）
    parameter        FIFO_AW = 8,         // 256 条，足够放完整的 111 条初始化表
    parameter        FILTER_VTS = 1,      // 1 = 丢弃对 0x320e/0x320f 的写（见下方说明）
    // 上电后延迟多久才开始转发（毫秒）。
    //   原因：A 板一上电 ~5ms 就开始发初始化表，而摄像头要几十 ms 才响应 I2C。
    //   不延迟的话前面几十条会被摄像头 NAK 掉，模式/HTS/VTS 这些关键寄存器
    //   全在表的前面，丢了摄像头就停在上电默认模式（1920x1080@14.8fps）。
    //   这里的做法是：先把 A 发来的写全部收进 FIFO，等摄像头准备好了再按序转发。
    parameter        START_DELAY_MS = 300
)(
    input  wire I_clk,
    input  wire I_rstn,

    // ---- 从机侧：接 A 板（B 板 DSI 座，P11/R12）----
    input  wire I_dsi_scl,
    inout  wire IO_dsi_sda,

    // ---- 主机侧：接摄像头（B 板 CSI 座，T9/U9）----
    output wire O_cam_scl,
    inout  wire IO_cam_sda,

    // ---- 统计（接调试串口/LED）----
    output reg  [15:0] O_ok_cnt,       // 转发成功条数
    output reg  [15:0] O_err_cnt,      // 摄像头没 ACK 的条数
    output reg  [15:0] O_vts_drop_cnt, // 被过滤掉的 VTS 写条数（调试）
    // 寄存器嗅探：记录 A 板最后写进这几个关键寄存器的值（仅调试）
    output reg  [7:0]  O_sn_301f,
    output reg  [7:0]  O_sn_320c,
    output reg  [7:0]  O_sn_320d,
    output reg  [15:0] O_first_reg,    // 上电后收到的第一条寄存器地址（调试）
    output reg  [7:0]  O_first_data,   // 上电后收到的第一条数据（调试）
    output wire [FIFO_AW:0] O_fifo_cnt,
    output wire        O_busy
);

    //------------------------------------------------------------------
    // 1. I2C 从机：收 A 板的写
    //------------------------------------------------------------------
    wire [15:0] rx_reg;
    wire [7:0]  rx_data;
    wire        rx_valid;
    wire        rx_seen;

    i2c_slave #( .DEV_ID ( DEV_ID ) ) u_slave (
        .clk     ( I_clk     ),
        .rstn    ( I_rstn    ),
        .scl_i   ( I_dsi_scl ),
        .sda_io  ( IO_dsi_sda),
        .o_reg   ( rx_reg    ),
        .o_data  ( rx_data   ),
        .o_valid ( rx_valid  ),
        .o_seen  ( rx_seen   ),
        .dbg_start(          ),
        .dbg_stop (          )
    );

    //------------------------------------------------------------------
    // 2. 转发状态机（先声明，FIFO 逻辑要引用）
    //------------------------------------------------------------------
    localparam S_IDLE = 2'd0;   // 等 FIFO 有数据
    localparam S_REQ  = 2'd1;   // 发起 I2C 请求
    localparam S_WAIT = 2'd2;   // 等 I2C 完成
    localparam S_POP  = 2'd3;   // 弹出 FIFO

    reg [1:0]  fsm_st;
    reg [1:0]  retry_cnt;         // 本条写操作的重试次数
    reg        first_seen;        // 是否已捕获到第一条写

    //------------------------------------------------------------------
    // 上电延迟：摄像头准备好之前只收不发
    //------------------------------------------------------------------
    localparam integer DELAY_CYC = 24000000 / 1000 * START_DELAY_MS;
    reg [31:0] startup_cnt;
    wire       relay_en = (startup_cnt >= DELAY_CYC);

    always @(posedge I_clk or negedge I_rstn) begin
        if (!I_rstn)              startup_cnt <= 32'd0;
        else if (!relay_en)       startup_cnt <= startup_cnt + 32'd1;
    end

    //------------------------------------------------------------------
    // 3. 命令 FIFO
    //------------------------------------------------------------------
    reg  [23:0] mem [0:(1<<FIFO_AW)-1];
    reg  [FIFO_AW:0] wptr, rptr;
    wire [FIFO_AW:0] cnt = wptr - rptr;
    assign O_fifo_cnt = cnt;

    wire fifo_empty = (wptr == rptr);
    wire fifo_full  = (cnt == (1<<FIFO_AW));

    //------------------------------------------------------------------
    // VTS 过滤
    //   A 板的 AE 表每次都会写 VTS（0x320e/0x320f），它的默认值是 2000，
    //   那是 2592x1944 模式的参数；而摄像头实际跑 1920x1080 模式、VTS 必须
    //   是 1127。让 A 改 VTS 会把帧周期拉长（20fps -> 11fps）。
    //   VTS 属于"时序模式"而不是"曝光控制"，所以这里拦掉，
    //   曝光(0x3e00~0x3e02)和增益(0x3e08/0x3e09)照常转发。
    //------------------------------------------------------------------
    wire is_vts   = (rx_reg == 16'h320e) || (rx_reg == 16'h320f);
    wire vts_drop = FILTER_VTS && is_vts;

    wire fifo_we    = rx_valid && !fifo_full && !vts_drop;
    wire fifo_re    = (fsm_st == S_POP);
    reg [31:0] wr_data;
    reg        iic_req;
    wire       iic_busy;
    wire       iic_bus_err;

    assign O_busy = (fsm_st != S_IDLE);

    always @(posedge I_clk or negedge I_rstn) begin
        if (!I_rstn) begin
            fsm_st   <= S_IDLE;
            retry_cnt<= 2'd0;
            wr_data  <= 32'd0;
            iic_req  <= 1'b0;
            O_ok_cnt <= 16'd0;
            O_err_cnt<= 16'd0;
            O_vts_drop_cnt <= 16'd0;
            first_seen     <= 1'b0;
            O_first_reg    <= 16'd0;
            O_first_data   <= 8'd0;
        end else begin
            // VTS 过滤计数（必须用 rx_valid 门控：rx_reg 会保持上一个值）
            if (rx_valid && vts_drop) O_vts_drop_cnt <= O_vts_drop_cnt + 16'd1;

            // 捕获上电后的第一条写（调试：判断 A 板到底先发什么）
            if (rx_valid && !first_seen) begin
                first_seen   <= 1'b1;
                O_first_reg  <= rx_reg;
                O_first_data <= rx_data;
            end

            // 寄存器嗅探：A 板写的关键时刻寄存器留一份（仅调试）
            if (rx_valid && !vts_drop) begin
                case (rx_reg)
                16'h301f: O_sn_301f <= rx_data;
                16'h320c: O_sn_320c <= rx_data;
                16'h320d: O_sn_320d <= rx_data;
                default: ;
                endcase
            end

            case (fsm_st)
            S_IDLE: begin
                iic_req <= 1'b0;
                // relay_en 之前只收不发，把 A 板发来的写先存在 FIFO 里
                if (relay_en && !fifo_empty && !iic_busy) begin
                    // 组装 4 字节：{DATA, REG_L, REG_H, DEV_ID}
                    wr_data <= { mem[rptr[FIFO_AW-1:0]][7:0],
                                 mem[rptr[FIFO_AW-1:0]][15:8],
                                 mem[rptr[FIFO_AW-1:0]][23:16],
                                 DEV_ID };
                    retry_cnt <= 2'd0;      // 新的一条，重试计数清零
                    fsm_st  <= S_REQ;
                end
            end
            S_REQ: begin
                iic_req <= 1'b1;
                if (iic_busy) begin
                    iic_req <= 1'b0;
                    fsm_st  <= S_WAIT;
                end
            end
            S_WAIT: begin
                if (!iic_busy) begin
                    if (iic_bus_err && (retry_cnt != 2'd3)) begin
                        // 摄像头还没准备好或总线抖动：重发同一条，不弹出 FIFO
                        retry_cnt <= retry_cnt + 2'd1;
                        fsm_st    <= S_IDLE;
                    end else begin
                        if (iic_bus_err) O_err_cnt <= O_err_cnt + 16'd1;
                        else             O_ok_cnt  <= O_ok_cnt  + 16'd1;
                        fsm_st <= S_POP;
                    end
                end
            end
            S_POP: begin
                fsm_st <= S_IDLE;
            end
            default: fsm_st <= S_IDLE;
            endcase
        end
    end

    // FIFO 读写
    always @(posedge I_clk or negedge I_rstn) begin
        if (!I_rstn) begin
            wptr <= {(FIFO_AW+1){1'b0}};
            rptr <= {(FIFO_AW+1){1'b0}};
        end else begin
            if (fifo_we) begin
                mem[wptr[FIFO_AW-1:0]] <= { rx_reg, rx_data };
                wptr <= wptr + 1'b1;
            end
            if (fifo_re) rptr <= rptr + 1'b1;
        end
    end

    //------------------------------------------------------------------
    // 4. I2C 主机（写摄像头）
    //------------------------------------------------------------------
    reg [7:0] rst_cnt;
    always @(posedge I_clk or negedge I_rstn)
        if (!I_rstn) rst_cnt <= 8'd0;
        else if (!rst_cnt[7]) rst_cnt <= rst_cnt + 1'b1;

    uii2c #(
        .WMEN_LEN ( 4     ),
        .RMEN_LEN ( 1     ),
        .CLK_DIV  ( CLK_DIV )
    ) u_master (
        .I_clk        ( I_clk      ),
        .I_rstn       ( rst_cnt[7] ),
        .O_iic_scl    ( O_cam_scl  ),
        .IO_iic_sda   ( IO_cam_sda ),
        .I_wr_data    ( wr_data    ),
        .I_wr_cnt     ( 8'd4      ),
        .O_rd_data    (            ),
        .I_rd_cnt     ( 8'd0      ),
        .I_iic_mode   ( 1'b0      ),
        .I_iic_req    ( iic_req    ),
        .O_iic_busy   ( iic_busy   ),
        .O_iic_bus_error( iic_bus_err ),
        .IO_iic_sda_dg(            ),
        .O_dbg_ack    (            )
    );

endmodule
