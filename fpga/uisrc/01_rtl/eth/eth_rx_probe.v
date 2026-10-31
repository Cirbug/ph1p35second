// =============================================================================
//  eth_rx_thumb - receive A's 480x270 RGB565 frame on ETHA and send a
//                 96x54 grayscale thumbnail out of the UART
//
//  Decimation: every 10th payload byte (i.e. every 5th pixel) is kept, giving
//  96 samples per row; every 5th row is kept, giving 54 rows.
//  The kept byte is the high byte of the RGB565 word (R5G3).
//
//  UART (J3, 115200): when a frame completes, print
//       "<5184 bytes as 10368 hex chars>"
// =============================================================================
`timescale 1ns / 1ps

module eth_rx_probe (
    input  wire        I_sys_clk,
    input  wire        I_rst_n,

    input  wire [3:0]  I_etha_rxd,
    input  wire        I_etha_rx_ctl,
    input  wire        I_etha_rxc,
    output wire [3:0]  O_etha_txd,
    output wire        O_etha_tx_ctl,
    output wire        O_etha_txc,
    output wire        O_etha_mdc,
    inout  wire        IO_etha_mdio,

    input  wire [3:0]  I_ethb_rxd,
    input  wire        I_ethb_rx_ctl,
    input  wire        I_ethb_rxc,
    output wire [3:0]  O_ethb_txd,
    output wire        O_ethb_tx_ctl,
    output wire        O_ethb_txc,
    output wire        O_ethb_mdc,
    inout  wire        IO_ethb_mdio,

    output wire        O_eth_rst_b,
    output wire        O_led_beat,
    output wire        O_led_err,
    output wire        O_uart_tx
);

    wire clk125, clk125_90, clk25, locked;
    net_pll u_pll (
        .refclk(I_sys_clk), .reset(~I_rst_n),
        .clk0_out(clk125), .clk1_out(clk125_90), .clk2_out(clk25), .lock(locked)
    );

    wire phy_rst_n, phy_done;
    uiphyrst #(.CLK_FREQ(25000000)) u_phy_reset (
        .I_CLK(clk25), .I_rstn(locked), .I_phyrst(locked),
        .O_phyrst(phy_rst_n), .O_phyrst_done(phy_done)
    );
    assign O_eth_rst_b = phy_rst_n;
    assign O_etha_mdc   = 1'b0;
    assign IO_etha_mdio = 1'bz;
    assign O_ethb_mdc   = 1'b0;
    assign IO_ethb_mdio = 1'bz;

    wire        a_rxclk;
    wire [7:0]  a_rxd8;
    wire        a_rxdv, a_rxer;

    rgmii_interface u_etha (
        .speed_10_100(1'b0), .gmii_tx_clk_d(clk125_90), .gmii_tx_reset_d(~locked),
        .rgmii_txd(O_etha_txd), .rgmii_tx_ctl(O_etha_tx_ctl), .rgmii_txc(O_etha_txc),
        .rgmii_rxd(I_etha_rxd), .rgmii_rx_ctl(I_etha_rx_ctl), .rgmii_rxc(I_etha_rxc),
        .gmii_tx_reset(~locked), .gmii_tx_clk(clk125), .gmii_txd(8'h00),
        .gmii_tx_en(1'b0), .gmii_tx_er(1'b0),
        .gmii_rx_reset(~locked),
        .gmii_rx_clk(a_rxclk), .gmii_rxd(a_rxd8),
        .gmii_rx_dv(a_rxdv), .gmii_rx_er(a_rxer),
        .gmii_crs(), .gmii_col(), .link_status(), .clock_speed(), .duplex_status()
    );

    rgmii_interface u_ethb (
        .speed_10_100(1'b0), .gmii_tx_clk_d(clk125_90), .gmii_tx_reset_d(~locked),
        .rgmii_txd(O_ethb_txd), .rgmii_tx_ctl(O_ethb_tx_ctl), .rgmii_txc(O_ethb_txc),
        .rgmii_rxd(I_ethb_rxd), .rgmii_rx_ctl(I_ethb_rx_ctl), .rgmii_rxc(I_ethb_rxc),
        .gmii_tx_reset(~locked), .gmii_tx_clk(clk125), .gmii_txd(8'h00),
        .gmii_tx_en(1'b0), .gmii_tx_er(1'b0),
        .gmii_rx_reset(~locked),
        .gmii_rx_clk(), .gmii_rxd(), .gmii_rx_dv(), .gmii_rx_er(),
        .gmii_crs(), .gmii_col(), .link_status(), .clock_speed(), .duplex_status()
    );

    // ---------------- receive + decimate ----------------
    reg [7:0]  thumb [0:5183];
    reg        sop;
    reg        a_dvd;
    reg [19:0] bi;           // 帧内字节序号（从 MAC 头起）
    reg [9:0]  bx;           // 行内字节 0..959
    reg [9:0]  row;          // 行 0..269
    reg [3:0]  m10;          // 0..9，每 10 字节采样一次
    reg [6:0]  xc;           // 缩略图列 0..95
    reg [2:0]  r5;           // 0..4，每 5 行取一行
    reg [5:0]  yc;           // 缩略图行 0..53
    reg        we;
    reg [12:0] waddr;
    reg [7:0]  wdata;
    reg        dump_req;
    reg [7:0]  frm_cnt;

    wire body = (bi >= 20'd54);          // 跳过 MAC/IP/UDP + CNN1 头
    wire [12:0] widx = yc * 7'd96 + xc;

    always @(posedge a_rxclk or negedge locked) begin
        if (!locked) begin
            sop <= 0; a_dvd <= 0; bi <= 0; bx <= 0; row <= 0; m10 <= 0;
            xc <= 0; r5 <= 0; yc <= 0; we <= 0; waddr <= 0; wdata <= 0;
            dump_req <= 0; frm_cnt <= 0;
        end else begin
            a_dvd <= a_rxdv;
            we <= 1'b0;
            if (a_rxdv) begin
                if (!sop) begin
                    if (a_rxd8 == 8'hD5 || a_rxd8 == 8'hF5) begin
                        sop <= 1'b1;
                        bi <= 20'd0; bx <= 0; row <= 0; m10 <= 0;
                        xc <= 0; r5 <= 0; yc <= 0;
                    end
                end else if (bi < 20'd259200) begin
                    bi <= bi + 20'd1;

                    if (body) begin
                        // 采样：每 10 字节取一个（且每 5 行取一行）
                        if (m10 == 4'd0 && r5 == 3'd0 && yc < 6'd54) begin
                            we <= 1'b1;
                            waddr <= widx;
                            wdata <= a_rxd8;
                        end
                        // 行内推进
                        if (bx == 10'd959) begin
                            bx <= 10'd0;
                            row <= row + 10'd1;
                            r5 <= (r5 == 3'd4) ? 3'd0 : (r5 + 3'd1);
                            if (r5 == 3'd4 && yc < 6'd53) yc <= yc + 6'd1;
                        end else bx <= bx + 10'd1;

                        // 10 字节计数
                        if (m10 == 4'd9) begin
                            m10 <= 4'd0;
                            if (xc == 7'd95) xc <= 7'd0; else xc <= xc + 7'd1;
                        end else m10 <= m10 + 4'd1;
                    end

                    if (bi == 20'd259198) begin
                        sop <= 1'b0;
                        dump_req <= ~dump_req;
                        frm_cnt <= frm_cnt + 8'd1;
                    end
                end
            end else begin
                sop <= 1'b0;
            end
        end
    end

    always @(posedge a_rxclk) begin
        if (we) thumb[waddr] <= wdata;
    end

    // ---------------- dump over UART ----------------
    (* ASYNC_REG = "TRUE" *) reg dr1, dr2, dr_d;
    reg [12:0] didx;
    reg        dact;
    reg        nib;
    reg [7:0]  tx_byte;
    reg        tx_send;
    wire       tx_busy;
    reg [7:0]  db;

    always @(posedge clk125) begin
        dr1 <= dump_req; dr2 <= dr1; dr_d <= dr2;
    end

    function [7:0] hexc;
        input [3:0] n;
        begin hexc = (n <= 4'd9) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n}); end
    endfunction

    always @(posedge clk125) begin
        db <= thumb[didx];
        if (!locked) begin
            didx <= 0; dact <= 0; nib <= 0; tx_byte <= 0; tx_send <= 0;
        end else begin
            tx_send <= 1'b0;
            if (!dact) begin
                if (dr2 != dr_d) begin
                    dact <= 1'b1; didx <= 0; nib <= 1'b0;
                end
            end else if (!tx_busy && !tx_send) begin
                tx_byte <= nib ? hexc(db[3:0]) : hexc(db[7:4]);
                tx_send <= 1'b1;
                if (nib) begin
                    nib <= 1'b0;
                    if (didx == 13'd5183) dact <= 1'b0;
                    else didx <= didx + 13'd1;
                end else nib <= 1'b1;
            end
        end
    end

    uart_tx_simple #(.CLK_HZ(125_000_000), .BAUD(115200)) u_uart (
        .clk(clk125), .rst(~locked), .data(tx_byte), .send(tx_send),
        .tx(O_uart_tx), .busy(tx_busy)
    );

    reg [24:0] beat_cnt;
    always @(posedge clk125) begin
        if (!locked) beat_cnt <= 0; else beat_cnt <= beat_cnt + 25'd1;
    end
    assign O_led_beat = beat_cnt[24];
    assign O_led_err  = ~locked | (frm_cnt != 0);

endmodule
