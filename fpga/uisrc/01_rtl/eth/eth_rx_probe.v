// =============================================================================
//  eth_ack - B board receives CNN1 packets on ETHA and answers ACK1 on ETHA
//
//  The A board only advances when it receives a matching ACK1 for each chunk,
//  so a working ACK proves bidirectional ethernet AND makes A stream at full
//  speed instead of one packet per timeout.
//
//  Received layout (from the first byte after SFD):
//      0..13 eth, 14..33 ip, 34..41 udp, 42..45 "CNN1",
//      48..49 frame_id, 50..51 packet_index
//
//  ACK1 frame: 60 bytes + 4 byte FCS
//      dst 02:00:00:00:10:01 (A board), src 02:00:00:00:00:02
//      IPv4 192.168.50.2 -> 192.168.50.1, UDP 6002 -> 6001
//      payload "ACK1" + frame_id + packet_index + status + 00
//
//  UART (J3, 115200) once per second:  R=<recv8> T=<ack8>
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
    wire [7:0]  a_txd8;
    wire        a_txen;

    rgmii_interface u_etha (
        .speed_10_100(1'b0), .gmii_tx_clk_d(clk125_90), .gmii_tx_reset_d(~locked),
        .rgmii_txd(O_etha_txd), .rgmii_tx_ctl(O_etha_tx_ctl), .rgmii_txc(O_etha_txc),
        .rgmii_rxd(I_etha_rxd), .rgmii_rx_ctl(I_etha_rx_ctl), .rgmii_rxc(I_etha_rxc),
        .gmii_tx_reset(~locked), .gmii_tx_clk(clk125), .gmii_txd(a_txd8),
        .gmii_tx_en(a_txen), .gmii_tx_er(1'b0),
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

    // ---------------- receive ----------------
    reg        sop, a_dvd;
    reg [15:0] bi;
    reg [31:0] magic;
    reg [15:0] fid, pidx;
    reg        have_pkt;
    reg [31:0] cnt_rx;
    reg        ack_req;
    reg [15:0] ack_fid, ack_idx;
    reg        ack_last;

    always @(posedge a_rxclk or negedge locked) begin
        if (!locked) begin
            sop <= 0; a_dvd <= 0; bi <= 0; magic <= 0; fid <= 0; pidx <= 0;
            have_pkt <= 0; cnt_rx <= 0; ack_req <= 0; ack_fid <= 0; ack_idx <= 0; ack_last <= 0;
        end else begin
            a_dvd <= a_rxdv;
            if (a_rxdv) begin
                if (!sop) begin
                    if (a_rxd8 == 8'hD5 || a_rxd8 == 8'hF5) begin
                        sop <= 1'b1; bi <= 0; magic <= 0; have_pkt <= 0;
                    end
                end else begin
                    magic <= {magic[23:0], a_rxd8};
                    if (bi == 16'd48) fid[15:8]  <= a_rxd8;
                    if (bi == 16'd49) fid[7:0]   <= a_rxd8;
                    if (bi == 16'd50) pidx[15:8] <= a_rxd8;
                    if (bi == 16'd51) pidx[7:0]  <= a_rxd8;
                    if (bi == 16'd45 && {magic[23:0], a_rxd8} == 32'h434E4E31)
                        have_pkt <= 1'b1;
                    bi <= bi + 16'd1;
                end
            end else begin
                if (sop && have_pkt) begin
                    ack_req  <= ~ack_req;
                    ack_fid  <= fid;
                    ack_idx  <= pidx;
                    ack_last <= (pidx == 16'd178);
                    cnt_rx   <= cnt_rx + 32'd1;
                end
                sop <= 1'b0;
            end
        end
    end

    // ---------------- ACK transmitter ----------------
    (* ASYNC_REG = "TRUE" *) reg s1, s2, sd;
    always @(posedge clk125) begin
        s1 <= ack_req; s2 <= s1; sd <= s2;
    end

    function [7:0] crc32_byte;
        input [31:0] c;
        input [7:0]  d;
        integer k;
        reg [31:0] x;
        begin
            x = c ^ {24'd0, d};
            for (k = 0; k < 8; k = k + 1)
                x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
            crc32_byte = x;
        end
    endfunction

    reg [7:0]  txpos;
    reg        en;
    reg [31:0] crc;
    reg [31:0] cnt_ack;

    wire [7:0] bidx = (txpos >= 8'd8) ? (txpos - 8'd8) : 8'd0;

    reg [7:0] cur;
    always @(*) begin
        case (bidx)
            // dst MAC 02:00:00:00:10:01
            8'd0:  cur = 8'h02;
            8'd1:  cur = 8'h00;
            8'd2:  cur = 8'h00;
            8'd3:  cur = 8'h00;
            8'd4:  cur = 8'h10;
            8'd5:  cur = 8'h01;
            // src MAC 02:00:00:00:00:02
            8'd6:  cur = 8'h02;
            8'd7:  cur = 8'h00;
            8'd8:  cur = 8'h00;
            8'd9:  cur = 8'h00;
            8'd10: cur = 8'h00;
            8'd11: cur = 8'h02;
            // ethertype
            8'd12: cur = 8'h08;
            8'd13: cur = 8'h00;
            // IPv4
            8'd14: cur = 8'h45;
            8'd15: cur = 8'h00;
            8'd16: cur = 8'h00;
            8'd17: cur = 8'h26;
            8'd18: cur = 8'h00;
            8'd19: cur = 8'h02;
            8'd20: cur = 8'h00;
            8'd21: cur = 8'h00;
            8'd22: cur = 8'h40;
            8'd23: cur = 8'h11;
            8'd24: cur = 8'h95;
            8'd25: cur = 8'h71;
            8'd26: cur = 8'hC0;
            8'd27: cur = 8'hA8;
            8'd28: cur = 8'h32;
            8'd29: cur = 8'h02;
            8'd30: cur = 8'hC0;
            8'd31: cur = 8'hA8;
            8'd32: cur = 8'h32;
            8'd33: cur = 8'h01;
            // UDP 6002 -> 6001, len 18
            8'd34: cur = 8'h17;
            8'd35: cur = 8'h72;
            8'd36: cur = 8'h17;
            8'd37: cur = 8'h71;
            8'd38: cur = 8'h00;
            8'd39: cur = 8'h12;
            8'd40: cur = 8'h00;
            8'd41: cur = 8'h00;
            // "ACK1" + fid + idx + status + 00
            8'd42: cur = 8'h41;
            8'd43: cur = 8'h43;
            8'd44: cur = 8'h4B;
            8'd45: cur = 8'h31;
            8'd46: cur = ack_fid[15:8];
            8'd47: cur = ack_fid[7:0];
            8'd48: cur = ack_idx[15:8];
            8'd49: cur = ack_idx[7:0];
            8'd50: cur = ack_last ? 8'h01 : 8'h00;
            8'd51: cur = 8'h00;
            default: cur = 8'h00;
        endcase
    end

    assign a_txd8 = (txpos < 8'd7)  ? 8'h55 :
                    (txpos == 8'd7) ? 8'hD5 :
                    (txpos < 8'd68) ? cur :
                    (~crc >> ((txpos - 8'd68) * 8));
    assign a_txen = en;

    always @(posedge clk125) begin
        if (!locked) begin
            txpos <= 0; en <= 0; crc <= 32'hFFFFFFFF; cnt_ack <= 0;
        end else if (!en) begin
            if (s2 != sd) begin
                en <= 1'b1; txpos <= 8'd0; crc <= 32'hFFFFFFFF;
            end
        end else begin
            if (txpos >= 8'd8 && txpos < 8'd68)
                crc <= crc32_byte(crc, cur);
            if (txpos == 8'd71) begin
                en <= 1'b0; cnt_ack <= cnt_ack + 32'd1;
            end else txpos <= txpos + 8'd1;
        end
    end

    // ---------------- UART ----------------
    reg [26:0] tick;
    reg        line_run;
    reg [5:0]  idx;
    reg [31:0] rf_r, rf_t;
    reg [7:0]  tx_byte;
    reg        tx_send;
    wire       tx_busy;

    function [7:0] hexc;
        input [3:0] n;
        begin hexc = (n <= 4'd9) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n}); end
    endfunction

    wire [4:0] i1 = (idx >= 6'd2 && idx <= 6'd9) ? (idx[4:0] - 5'd2) : 5'd0;
    wire [4:0] i2 = (idx >= 6'd13 && idx <= 6'd20) ? (idx[4:0] - 5'd13) : 5'd0;
    wire [31:0] sh1 = rf_r >> ((5'd7 - i1) * 4);
    wire [31:0] sh2 = rf_t >> ((5'd7 - i2) * 4);

    wire [7:0] cur_chr =
        (idx == 6'd0)  ? 8'h52 :
        (idx == 6'd1)  ? 8'h3D :
        (idx <= 6'd9)  ? hexc(sh1[3:0]) :
        (idx == 6'd10) ? 8'h20 :
        (idx == 6'd11) ? 8'h54 :
        (idx == 6'd12) ? 8'h3D :
        (idx <= 6'd20) ? hexc(sh2[3:0]) :
        (idx == 6'd21) ? 8'h0D : 8'h0A;

    (* ASYNC_REG = "TRUE" *) reg [31:0] r1, r2;
    always @(posedge clk125) begin
        r1 <= cnt_rx; r2 <= r1;
    end

    always @(posedge clk125) begin
        if (!locked) begin
            tick <= 0; line_run <= 0; idx <= 0; tx_byte <= 0; tx_send <= 0;
            rf_r <= 0; rf_t <= 0;
        end else begin
            tx_send <= 1'b0;
            if (tick == 27'd124_999_999) begin
                tick <= 0;
                if (!line_run) begin rf_r <= r2; rf_t <= cnt_ack; line_run <= 1'b1; idx <= 0; end
            end else tick <= tick + 27'd1;

            if (line_run && !tx_busy && !tx_send) begin
                tx_byte <= cur_chr;
                tx_send <= 1'b1;
                if (idx == 6'd21) begin idx <= 0; line_run <= 1'b0; end
                else idx <= idx + 6'd1;
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
    assign O_led_err  = ~locked | (cnt_ack != 0);

endmodule
