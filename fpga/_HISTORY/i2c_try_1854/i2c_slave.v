//==========================================================================
// i2c_slave.v   （滤波过采样版）
//
//   I2C 从机：在 B 板 DSI 侧总线上"冒充"摄像头（地址 0x6C）。
//
//   为什么用滤波过采样而不是裸边沿检测：
//     SCL/SDA 是异步输入，两级同步器之后仍可能因两路延迟不同而在
//     "SCL 与 SDA 同时变化"的时刻造出假 START/STOP。本设计要求
//     SCL 连续稳定若干拍才认为电平有效，滤掉这类毛刺。
//
//   事务格式（与 A 板 uii2c 一致）：
//     START | DEV_ID(W) | REG_H | REG_L | DATA | STOP
//
//   时序：
//     · 第 1..8 个 SCL 上升沿采样 8 个数据位
//     · 第 8 个上升沿数据收齐 → 立刻决定 ACK
//     · ACK 在第 9 个 SCL 低电平期间拉低 SDA，下一个低电平期间释放
//       —— 绝不在 SCL 高电平时改变 SDA
//==========================================================================
module i2c_slave #(
    parameter [7:0] DEV_ID = 8'h6C
)(
    input  wire        clk,
    input  wire        rstn,

    input  wire        scl_i,
    inout  wire        sda_io,

    output reg  [15:0] o_reg,
    output reg  [7:0]  o_data,
    output reg         o_valid,
    output reg         o_seen,

    output wire        dbg_start,
    output wire        dbg_stop
);

    //------------------------------------------------------------------
    // 1. 同步 + 稳定滤波
    //------------------------------------------------------------------
    (* ASYNC_REG="TRUE" *) reg [2:0] scl_sy, sda_sy;
    wire scl_raw = scl_sy[2];
    wire sda_raw = sda_sy[2];

    reg [3:0] scl_hist;
    reg [3:0] sda_hist;
    reg       scl_f, sda_f;

    reg [3:0] scl_hi_cnt;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            scl_sy   <= 3'b111; sda_sy   <= 3'b111;
            scl_hist <= 4'b1111; sda_hist <= 4'b1111;
            scl_f    <= 1'b1;   sda_f    <= 1'b1;
            scl_hi_cnt <= 4'd0;
        end else begin
            scl_sy   <= { scl_sy[1:0],   scl_i  };
            sda_sy   <= { sda_sy[1:0],   sda_io };
            scl_hist <= { scl_hist[2:0], scl_raw };
            sda_hist <= { sda_hist[2:0], sda_raw };

            // 连续 4 拍一致才更新电平
            if (scl_hist == 4'b1111) scl_f <= 1'b1;
            if (scl_hist == 4'b0000) scl_f <= 1'b0;
            if (sda_hist == 4'b1111) sda_f <= 1'b1;
            if (sda_hist == 4'b0000) sda_f <= 1'b0;

            if (scl_f) begin
                if (scl_hi_cnt != 4'd15) scl_hi_cnt <= scl_hi_cnt + 4'd1;
            end else begin
                scl_hi_cnt <= 4'd0;
            end
        end
    end

    // SCL 已稳定为高：够久才认 START/STOP
    wire scl_hi_stable = (scl_hi_cnt >= 4'd12);
    wire scl_lo_stable = (scl_hi_cnt == 4'd0);

    // 滤波后的边沿
    reg scl_f_d, sda_f_d;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            scl_f_d <= 1'b1; sda_f_d <= 1'b1;
        end else begin
            scl_f_d <= scl_f;
            sda_f_d <= sda_f;
        end
    end

    wire scl_rise = scl_f && !scl_f_d;
    wire scl_fall = !scl_f && scl_f_d;
    wire sda_rise = sda_f && !sda_f_d;
    wire sda_fall = !sda_f && sda_f_d;

    wire start_det = scl_hi_stable && sda_fall;
    wire stop_det  = scl_hi_stable && sda_rise;

    assign dbg_start = start_det;
    assign dbg_stop  = stop_det;

    //------------------------------------------------------------------
    // 2. SDA 输出
    //------------------------------------------------------------------
    reg sda_oe;
    assign sda_io = sda_oe ? 1'b0 : 1'bz;

    //------------------------------------------------------------------
    // 3. 接收状态机
    //------------------------------------------------------------------
    localparam S_IDLE = 2'd0;
    localparam S_BYTE = 2'd1;
    localparam S_STOP = 2'd2;

    reg [1:0] st;
    reg [3:0] bit;                // 0..8，必须 4 位（3 位会把 8 截断成 0）
    reg [7:0] sh;
    reg [1:0] byte_idx;
    reg       ack_next;
    reg       fin_byte;
    reg [7:0] regh_r;

    wire [7:0] byte_in = { sh[6:0], sda_f };

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st       <= S_IDLE;
            bit      <= 4'd0;
            sh       <= 8'd0;
            byte_idx <= 2'd0;
            sda_oe   <= 1'b0;
            ack_next <= 1'b0;
            fin_byte <= 1'b0;
            regh_r   <= 8'd0;
            o_reg    <= 16'd0;
            o_data   <= 8'd0;
            o_valid  <= 1'b0;
            o_seen   <= 1'b0;
        end else begin
            o_valid <= 1'b0;

            if (start_det) begin
                st       <= S_BYTE;
                bit      <= 4'd0;
                sh       <= 8'd0;
                byte_idx <= 2'd0;
                ack_next <= 1'b0;
                fin_byte <= 1'b0;
                sda_oe   <= 1'b0;
            end

            if (stop_det) begin
                st     <= S_IDLE;
                bit    <= 4'd0;
                sda_oe <= 1'b0;
            end

            //----------- 下降沿：ACK 期间拉低，其余释放 -----------
            // 注意：必须放在状态判断之外！最后一个字节 ACK 之后状态已切到
            // S_STOP，若放在 S_BYTE 里就永远执行不到，SDA 会被永久拉低，
            // 导致后续 START 无法产生。
            if (scl_fall) begin
                if ((st == S_BYTE) && (bit == 4'd8)) sda_oe <= ack_next;
                else                                 sda_oe <= 1'b0;
            end

            if (st == S_BYTE) begin
                //----------- 上升沿：采样 -----------
                if (scl_rise) begin
                    if (bit < 4'd7) begin
                        sh  <= { sh[6:0], sda_f };
                        bit <= bit + 4'd1;
                    end else if (bit == 4'd7) begin
                        sh  <= byte_in;
                        bit <= 4'd8;
                        case (byte_idx)
                        2'd0: begin
                            ack_next <= (byte_in[7:1] == DEV_ID[7:1]) && (byte_in[0] == 1'b0);
                            byte_idx <= 2'd1;
                        end
                        2'd1: begin
                            regh_r   <= byte_in;
                            ack_next <= 1'b1;
                            byte_idx <= 2'd2;
                        end
                        2'd2: begin
                            o_reg    <= { regh_r, byte_in };
                            ack_next <= 1'b1;
                            byte_idx <= 2'd3;
                        end
                        2'd3: begin
                            o_data   <= byte_in;
                            o_valid  <= 1'b1;
                            o_seen   <= 1'b1;
                            ack_next <= 1'b1;
                            byte_idx <= 2'd0;
                            fin_byte <= 1'b1;
                        end
                        default: ;
                        endcase
                    end else begin
                        // 第 9 个上升沿：ACK 位采样完毕
                        bit <= 4'd0;
                        sh  <= 8'd0;
                        if (fin_byte) begin
                            fin_byte <= 1'b0;
                            st       <= S_STOP;
                        end
                    end
                end

            end
        end
    end

endmodule
