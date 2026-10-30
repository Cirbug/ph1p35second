//==========================================================================
// ae_from_dsi.v
//   从 A 板发来的寄存器写里，提取曝光/增益数值。
//
//   背景：A 板通过 40P 第 3/6 脚的 I2C 把 SC500CS 的 AE 写发过来，B 板
//         用 i2c_slave 收下来。但 B 板自己也有一套初始化（保证 1920x1080
//         的稳定 20fps），不能把 A 的写原样转发（A 的表是 2592 的，会破坏
//         B 已经配好的模式）。
//
//         所以这里只"取数值"：把 A 写的曝光/增益拼成 I_AE / I_AG，喂给
//         B 自己的 uics500regAE 去写摄像头。B 的 AE 表只有 5 条，不碰 VTS，
//         不会破坏模式。
//
//   A 板的 AE 写格式（来自 uics500regAE）：
//     0x3E00 = {4'h0, EXP[15:12]}
//     0x3E01 = EXP[11:4]
//     0x3E02 = {EXP[3:0], 4'h0}
//     0x3E08 = GAIN[15:8]
//     0x3E09 = GAIN[7:0]
//
//   B 板 uics500regAE 需要的 I_AE[15:0]：
//     0x3E00 <= {4'h0, I_AE[15:12]}
//     0x3E01 <= I_AE[11:4]
//   所以 I_AE = {EXP[15:12], EXP[11:4]} = {data(0x3E00)[3:0], data(0x3E01)}
//==========================================================================
module ae_from_dsi (
    input  wire        I_clk,
    input  wire        I_rstn,

    input  wire [15:0] I_reg,        // 收到的寄存器地址
    input  wire [7:0]  I_data,       // 收到的数据
    input  wire        I_valid,      // 单拍有效

    input  wire        I_cfg_done,   // 摄像头初始化完成（完成后才允许 AE）

    output reg  [15:0] O_ae,         // 曝光，给 uics500regAE 的 I_AE
    output reg  [15:0] O_ag,         // 增益，给 uics500regAE 的 I_AG
    output reg         O_req         // 请求写一次 AE（单拍）
);

    reg [3:0] exp_hi;                // 暂存 0x3E00 的高半字节

    always @(posedge I_clk or negedge I_rstn) begin
        if (!I_rstn) begin
            exp_hi <= 4'd0;
            O_ae   <= 16'd2246;      // 与初始化表的曝光初值一致
            O_ag   <= 16'd80;
            O_req  <= 1'b0;
        end else begin
            O_req <= 1'b0;

            if (I_valid) begin
                case (I_reg)
                //---- 曝光 ----
                16'h3E00: exp_hi <= I_data[3:0];
                16'h3E01: O_ae   <= { exp_hi, I_data };

                //---- 增益：收到低字节才触发一次写 ----
                16'h3E08: O_ag[15:8] <= I_data;
                16'h3E09: begin
                    O_ag[7:0] <= I_data;
                    // 只在初始化完成后才允许请求，否则会把初始化表打回第 0 条
                    if (I_cfg_done) O_req <= 1'b1;
                end

                default: ;
                endcase
            end
        end
    end

endmodule
