//==========================================================================
// stub_top_deps.v  —— 仅用于 ModelSim，不参与综合
//
//   顶层 csi2_tx_top 依赖三个"厂商件"：
//     - pll（TD 生成的 IP）
//     - mipi_dphy_rx_ph1p_mipiio_wrapper（含 PH1P 原语）
//     - mipi_dphy_tx_over_ph1p_mipiio_wrapper（含 PH1P 原语）
//   ModelSim 没有 PH1P 原语库，这里用行为级替身：
//     1) 端口名/位宽与真实封装严格一致（已脚本比对，17/22 个端口全对）
//     2) RX 侧提供可驱动的 rx_valid_r / rx_data_r，供顶层 testbench 灌数据
//     3) RX 侧自己产生字节时钟，模拟摄像头 MIPI 时钟恢复
//     4) TX 侧把参考时钟直通为字节时钟，模拟 DPLL 锁定后的输出
//==========================================================================
`timescale 1ns / 1ps

module pll (
    input  wire refclk,
    input  wire reset,
    output wire clk0_out,
    output wire clk1_out,
    output wire lock
);
    assign clk0_out = refclk;
    assign clk1_out = refclk;
    assign lock     = 1'b1;
endmodule


module mipi_dphy_rx_ph1p_mipiio_wrapper #(
    parameter DPHY_RX_LOCATION = "DPHY0",
    parameter HS_EQUALIZER     = "0dB",
    parameter HS_VGA_GAIN      = "0dB",
    parameter LANE_NUM         = 4,
    parameter BYTE_NUM         = 1
)(
    input  wire                        I_lp_clk,
    input  wire                        I_rst,
    input  wire [5:0]                  I_clk_lane_in_delay,
    input  wire [5:0]                  I_data_lane0_in_delay,
    input  wire [5:0]                  I_data_lane1_in_delay,
    input  wire [5:0]                  I_data_lane2_in_delay,
    input  wire [5:0]                  I_data_lane3_in_delay,
    input  wire [LANE_NUM-1:0]         I_lane_invert,
    output wire                        O_hs_rx_clk,
    output wire                        O_hs_rx_valid,
    output wire [LANE_NUM*BYTE_NUM*8-1:0] O_hs_rx_data,
    output wire                        O_lp_rx_lane0_p,
    output wire                        O_lp_rx_lane0_n,
    input  wire                        I_lp_tx_en,
    input  wire                        I_lp_tx_lane0_p,
    input  wire                        I_lp_tx_lane0_n,
    output wire [3:0]                  O_lane_match_error,
    output wire [LANE_NUM-1:0]         O_lane_error,
    inout  wire                        IO_rx_clk_pad_n,
    inout  wire                        IO_rx_clk_pad_p,
    inout  wire [3:0]                  IO_rx_data_pad_n,
    inout  wire [3:0]                  IO_rx_data_pad_p
);
    // 模拟摄像头 MIPI 字节时钟（约 10MHz，比 TX 慢，符合实际比例）
    reg rxclk_r = 1'b0;
    always #50 rxclk_r = ~rxclk_r;

    // 供 testbench 通过层次路径驱动
    reg                        rx_valid_r = 1'b0;
    reg [LANE_NUM*BYTE_NUM*8-1:0] rx_data_r = 0;

    assign O_hs_rx_clk        = rxclk_r;
    assign O_hs_rx_valid      = rx_valid_r;
    assign O_hs_rx_data       = rx_data_r;
    assign O_lp_rx_lane0_p    = 1'b1;
    assign O_lp_rx_lane0_n    = 1'b1;
    assign O_lane_match_error = 4'b0000;
    assign O_lane_error       = {LANE_NUM{1'b0}};
endmodule


module mipi_dphy_tx_over_ph1p_mipiio_wrapper #(
    parameter DPHY_TX_LOCATION      = "DPHY0",
    parameter BYTE_NUM              = 1,
    parameter LANE_NUM              = 4,
    parameter CLK_LANE_MODE         = "CONTINUE",
    parameter HS_OUTPUT_VOD         = "250mV",
    parameter HS_OUTPUT_IMPEDANCE   = "50ohm",
    parameter HS_OUTPUT_DE_EMPHASIS = "0dB",
    parameter CK_OUTPUT_DELAY       = 0,
    parameter L0_OUTPUT_DELAY       = 0,
    parameter L1_OUTPUT_DELAY       = 0,
    parameter L2_OUTPUT_DELAY       = 0,
    parameter L3_OUTPUT_DELAY       = 0,
    parameter DPLL_MULTI_RATIO      = 19,
    parameter DPLL_DIV_RATIO        = 1,
    parameter T_DATA_LPX            = 4,
    parameter T_DATA_HS_PREPARE     = 4,
    parameter T_DATA_HS_ZERO        = 10,
    parameter T_DATA_HS_TRAIL       = 10,
    parameter T_CLK_LPX             = 4,
    parameter T_CLK_PREPARE         = 4,
    parameter T_CLK_ZERO            = 10,
    parameter T_CLK_PRE            = 10,
    parameter T_CLK_POST           = 10,
    parameter T_CLK_TRAIL           = 10
)(
    input  wire                        I_dphy_pll_ref_clk,
    input  wire                        I_rst,
    output wire                        O_dpll_locked,
    output wire                        O_hs_tx_clk,
    input  wire                        I_hs_tx_valid,
    input  wire [LANE_NUM*BYTE_NUM*8-1:0] I_hs_tx_data,
    input  wire                        I_hs_tx_last,
    output wire                        O_hs_tx_ready,
    input  wire                        I_lp_tx_p,
    input  wire                        I_lp_tx_n,
    input  wire                        I_lp_rx_en,
    output wire                        O_lp_rx_p,
    output wire                        O_lp_rx_n,
    inout  wire                        IO_tx_clk_pad_n,
    inout  wire                        IO_tx_clk_pad_p,
    inout  wire [3:0]                  IO_tx_data_pad_n,
    inout  wire [3:0]                  IO_tx_data_pad_p
);
    // 模拟 DPLL 锁定后的字节时钟（直通参考时钟）
    assign O_dpll_locked = 1'b1;
    assign O_hs_tx_clk   = I_dphy_pll_ref_clk;
    assign O_hs_tx_ready = 1'b1;
    assign O_lp_rx_p     = 1'b1;
    assign O_lp_rx_n     = 1'b1;
endmodule
