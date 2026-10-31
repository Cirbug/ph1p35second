// =============================================================================
//  eth_top  —  PH1P35 B 板（MLKPAI-F02 + FC02）双以太网顶层
//
//  硬件（见《MLKPAI-F02开发平台硬件使用手册》第 15/16 页）：
//    ETHA -> RJ45 P1，PHY = YT8531DH-CA，RGMII
//    ETHB -> RJ45 P2，PHY = YT8531DH-CA，RGMII
//    两颗 PHY 各自有 25MHz 有源晶振，复位共用 ETH_RST_B
//
//  当前版本：先做"引脚合法性 + PHY 复位 + 时钟"验证骨架，
//            两颗 PHY 的 TX 输出固定 0，RX 暂未使用。
//            验证通过后再接入 RGMII 收发与 UDP 中继。
// =============================================================================
`timescale 1ns / 1ps

module eth_top (
    input  wire        I_sys_clk,        // C4  25 MHz 板载晶振
    input  wire        I_rst_n,          // R10 复位按键

    // ---------------- ETHA : RJ45 P1 ----------------
    input  wire [3:0]  I_etha_rxd,       // H1 K1 K2 L3
    input  wire        I_etha_rx_ctl,    // J1
    input  wire        I_etha_rxc,       // N6
    output wire [3:0]  O_etha_txd,       // P3 R1 R2 U1
    output wire        O_etha_tx_ctl,    // L2
    output wire        O_etha_txc,       // R3
    output wire        O_etha_mdc,       // M5
    input  wire        I_etha_mdio,      // V1

    // ---------------- ETHB : RJ45 P2 ----------------
    input  wire [3:0]  I_ethb_rxd,       // L4 M2 N1 T2
    input  wire        I_ethb_rx_ctl,    // M1
    input  wire        I_ethb_rxc,       // N7
    output wire [3:0]  O_ethb_txd,       // P4 P5 T4 T3
    output wire        O_ethb_tx_ctl,    // P1
    output wire        O_ethb_txc,       // U2
    output wire        O_ethb_mdc,       // P6
    input  wire        I_ethb_mdio,      // R5

    // ---------------- 其它 ----------------
    output wire        O_eth_rst_b,      // H2  两颗 PHY 共用复位（低有效）
    output wire        O_led_beat,       // K16 心跳
    output wire        O_led_err         // K17 预留
);

    // ------------------------------------------------------------------
    // 时钟：25 MHz 晶振 -> 125 MHz / 125 MHz(90°) / 25 MHz
    //   net_pll 已按 25 MHz 输入改过（FIN=25, REFCLK_DIV=1，其余同 A 板）
    // ------------------------------------------------------------------
    wire clk125, clk125_90, clk25, locked;

    net_pll u_pll (
        .refclk  (I_sys_clk),
        .reset   (~I_rst_n),
        .clk0_out(clk125),
        .clk1_out(clk125_90),
        .clk2_out(clk25),
        .lock    (locked)
    );

    // ------------------------------------------------------------------
    // PHY 复位时序（两颗 PHY 共用 ETH_RST_B）
    // ------------------------------------------------------------------
    wire phy_rst_n, phy_done;

    uiphyrst #(.CLK_FREQ(25000000)) u_phy_reset (
        .I_CLK        (clk25),
        .I_rstn       (locked),
        .I_phyrst     (locked),
        .O_phyrst     (phy_rst_n),
        .O_phyrst_done(phy_done)
    );

    assign O_eth_rst_b = phy_rst_n;

    // 复位同步
    (* ASYNC_REG = "TRUE" *) reg done1, done2;
    always @(posedge clk125 or negedge locked) begin
        if (!locked) begin
            done1 <= 1'b0;
            done2 <= 1'b0;
        end else begin
            done1 <= phy_done;
            done2 <= done1;
        end
    end
    wire rst = !done2;

    // ------------------------------------------------------------------
    // 两颗 PHY 的 RGMII 接口（本版 TX 固定空闲，RX 暂不接）
    // ------------------------------------------------------------------
    wire        etha_rxclk, ethb_rxclk;
    wire [7:0]  etha_rxd8, ethb_rxd8;
    wire        etha_rxdv, ethb_rxdv;
    wire        etha_rxer, ethb_rxer;

    rgmii_interface u_etha_rgmii (
        .speed_10_100    (1'b0),
        .gmii_tx_clk_d   (clk125),
        .gmii_tx_reset_d (rst),
        .rgmii_txd       (O_etha_txd),
        .rgmii_tx_ctl    (O_etha_tx_ctl),
        .rgmii_txc       (O_etha_txc),
        .rgmii_rxd       (I_etha_rxd),
        .rgmii_rx_ctl    (I_etha_rx_ctl),
        .rgmii_rxc       (I_etha_rxc),
        .gmii_tx_reset   (rst),
        .gmii_tx_clk     (clk125),
        .gmii_txd        (8'h00),
        .gmii_tx_en      (1'b0),
        .gmii_tx_er      (1'b0),
        .gmii_rx_reset   (rst),
        .gmii_rx_clk     (etha_rxclk),
        .gmii_rxd        (etha_rxd8),
        .gmii_rx_dv      (etha_rxdv),
        .gmii_rx_er      (etha_rxer),
        .gmii_crs        (),
        .gmii_col        (),
        .link_status     (),
        .clock_speed     (),
        .duplex_status   ()
    );

    rgmii_interface u_ethb_rgmii (
        .speed_10_100    (1'b0),
        .gmii_tx_clk_d   (clk125),
        .gmii_tx_reset_d (rst),
        .rgmii_txd       (O_ethb_txd),
        .rgmii_tx_ctl    (O_ethb_tx_ctl),
        .rgmii_txc       (O_ethb_txc),
        .rgmii_rxd       (I_ethb_rxd),
        .rgmii_rx_ctl    (I_ethb_rx_ctl),
        .rgmii_rxc       (I_ethb_rxc),
        .gmii_tx_reset   (rst),
        .gmii_tx_clk     (clk125),
        .gmii_txd        (8'h00),
        .gmii_tx_en      (1'b0),
        .gmii_tx_er      (1'b0),
        .gmii_rx_reset   (rst),
        .gmii_rx_clk     (ethb_rxclk),
        .gmii_rxd        (ethb_rxd8),
        .gmii_rx_dv      (ethb_rxdv),
        .gmii_rx_er      (ethb_rxer),
        .gmii_crs        (),
        .gmii_col        (),
        .link_status     (),
        .clock_speed     (),
        .duplex_status   ()
    );

    // MDIO 本版不驱动：MDC 拉低，MDIO 只做输入
    assign O_etha_mdc = 1'b0;
    assign O_ethb_mdc = 1'b0;

    // ------------------------------------------------------------------
    // 灯：125MHz / 2^25 ≈ 3.7Hz 心跳
    // ------------------------------------------------------------------
    reg [24:0] beat_cnt;
    always @(posedge clk125 or posedge rst) begin
        if (rst) beat_cnt <= 25'd0;
        else     beat_cnt <= beat_cnt + 25'd1;
    end
    assign O_led_beat = beat_cnt[24];
    assign O_led_err  = ~locked;

endmodule
