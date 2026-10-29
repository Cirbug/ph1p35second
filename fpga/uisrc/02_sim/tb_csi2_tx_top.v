//==========================================================================
// tb_csi2_tx_top.v
//   Top-level integration test with drivable PHY stubs.
//   DPHY0(stub) -> csi2_rx_parse -> token pack -> async FIFO -> token unpack
//     -> csi2_tx_packet -> DPHY1(stub)
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_tx_top;

    localparam L  = 2;
    localparam WC = 32;

    reg sysclk = 1'b0;
    reg rst_n  = 1'b0;
    always #20 sysclk = ~sysclk;

    wire        O_cam_scl;
    wire        IO_cam_sda;
    wire        O_cam_24m;
    wire        O_cam_rst;
    wire        O_led_cfg;
    wire        O_led_beat;
    wire        O_led_err;

    csi2_tx_top #( .LANE_SWAP ( 0 ) ) dut (
        .I_sys_clk          ( sysclk     ),
        .I_rst_n            ( rst_n      ),
        .I_button           ( 2'b11      ),
        .O_cam_scl          ( O_cam_scl  ),
        .IO_cam_sda         ( IO_cam_sda ),
        .O_cam_24m          ( O_cam_24m  ),
        .O_cam_rst          ( O_cam_rst  ),
        .IO_tx_clk_pad_p    (            ),
        .IO_tx_clk_pad_n    (            ),
        .IO_tx_data_pad_p   (            ),
        .IO_tx_data_pad_n   (            ),
        .IO_rx_clk_pad_p    (            ),
        .IO_rx_clk_pad_n    (            ),
        .IO_rx_data_pad_p   (            ),
        .IO_rx_data_pad_n   (            ),
        .O_led_cfg          ( O_led_cfg  ),
        .O_led_beat         ( O_led_beat ),
        .O_led_err          ( O_led_err  )
    );

    reg  [23:0] ref_hdr;
    wire [7:0]  ref_ecc;
    csi2_ecc u_ecc ( .I_hdr ( ref_hdr ), .O_ecc ( ref_ecc ) );

    reg [7:0] got [0:191];
    integer   gi, npkt;
    integer   pend_i [0:7];

    always @(posedge dut.tx_clk) begin
        if (rst_n && dut.tx_hs_en) begin
            got[gi]     = dut.tx_hs_data[15:8];
            got[gi + 1] = dut.tx_hs_data[7:0];
            gi = gi + 2;
            if (dut.tx_hs_last) begin
                pend_i[npkt] = gi;
                npkt = npkt + 1;
            end
        end
    end

    task rx_beat;
        input [15:0] d;
        begin
            @(negedge dut.u_dphy_rx.rxclk_r);
            dut.u_dphy_rx.rx_valid_r = 1'b1;
            dut.u_dphy_rx.rx_data_r  = d;
        end
    endtask

    task rx_gap;
        input integer n;
        begin
            @(negedge dut.u_dphy_rx.rxclk_r);
            dut.u_dphy_rx.rx_valid_r = 1'b0;
            repeat (n) @(negedge dut.u_dphy_rx.rxclk_r);
        end
    endtask

    function [15:0] crc_byte;
        input [15:0] s;
        input [7:0]  b;
        integer      i;
        reg   [15:0] c;
        reg          fb;
        begin
            c = s;
            for (i = 7; i >= 0; i = i - 1) begin
                fb = c[15] ^ b[i];
                c  = { c[14:0], 1'b0 };
                if (fb) c = c ^ 16'h1021;
            end
            crc_byte = c;
        end
    endfunction

    reg [7:0]  exp_pay [0:WC-1];
    integer    i, p, errs;
    reg [15:0] crc_ref;

    initial begin
        errs = 0; gi = 0; npkt = 0;
        for (i = 0; i < 8; i = i + 1) pend_i[i] = 0;

        rst_n = 1'b0;
        repeat (6) @(posedge sysclk);
        rst_n = 1'b1;
        repeat (6) @(posedge sysclk);

        drv_packets;
        repeat (300) @(posedge dut.tx_clk);
        check;
    end

    task drv_packets;
        begin
            ref_hdr = 24'h00_0000; #1;
            rx_beat({ 8'h00, 8'h00 });
            rx_beat({ 8'h00, ref_ecc });
            rx_gap(8);

            rx_beat({ 8'h2B, WC[7:0] });
            ref_hdr = { WC[15:8], WC[7:0], 8'h2B }; #1;
            rx_beat({ WC[15:8], ref_ecc });

            for (i = 0; i < WC/2; i = i + 1) begin
                exp_pay[2*i]     = 8'hA0 + i[7:0];
                exp_pay[2*i + 1] = 8'h50 + i[7:0];
                rx_beat({ 8'hA0 + i[7:0], 8'h50 + i[7:0] });
            end
            rx_beat({ 8'hC0, 8'hDE });
            rx_gap(8);

            ref_hdr = 24'h00_0001; #1;
            rx_beat({ 8'h01, 8'h00 });
            rx_beat({ 8'h00, ref_ecc });
            rx_gap(10);
        end
    endtask

    task check;
        begin
            $display("--------------------------------------------------");
            $display("TX packets captured: %0d  len %0d / %0d / %0d", npkt,
                     pend_i[0], pend_i[1]-pend_i[0],
                     (npkt>2) ? pend_i[2]-pend_i[1] : 0);

            if (npkt !== 4) begin
                $display("FAIL: expect 4 packets (FS/LONG/FE/FS), got %0d", npkt);
                errs = errs + 1;
            end

            ref_hdr = 24'h00_0000; #1;
            if (got[0]!==8'h00 || got[3]!==ref_ecc) begin
                $display("FAIL: FS wrong"); errs = errs + 1;
            end else $display("PASS: FS  = 00 00 00 %02h", ref_ecc);

            ref_hdr = { got[6], got[5], got[4] }; #1;
            if (got[4] !== 8'h2B) begin
                $display("FAIL: long DT=%02h", got[4]); errs = errs + 1;
            end
            if ({got[6],got[5]} !== WC[15:0]) begin
                $display("FAIL: long WC=%0d", {got[6],got[5]}); errs = errs + 1;
            end
            if (got[7] !== ref_ecc) begin
                $display("FAIL: long ECC=%02h", got[7]); errs = errs + 1;
            end

            p = 8;
            for (i = 0; i < WC; i = i + 1)
                if (got[p+i] !== exp_pay[i]) begin
                    if (errs < 6)
                        $display("FAIL: payload[%0d]=%02h exp %02h", i, got[p+i], exp_pay[i]);
                    errs = errs + 1;
                end

            crc_ref = 16'hFFFF;
            for (i = 0; i < WC; i = i + 1) crc_ref = crc_byte(crc_ref, exp_pay[i]);
            if ({got[8+WC], got[9+WC]} !== crc_ref) begin
                $display("FAIL: CRC=%04h exp %04h", {got[8+WC],got[9+WC]}, crc_ref);
                errs = errs + 1;
            end

            ref_hdr = 24'h00_0001; #1;
            if (got[8+WC+2] !== 8'h01 || got[8+WC+5] !== ref_ecc) begin
                $display("FAIL: FE wrong"); errs = errs + 1;
            end else $display("PASS: FE  = 01 00 00 %02h", ref_ecc);

            if (errs == 0)
                $display("PASS: long packet DT/WC/ECC/payload(%0d bytes)/CRC all match", WC);

            if (O_led_err) begin
                $display("FAIL: O_led_err asserted"); errs = errs + 1;
            end

            $display("--------------------------------------------------");
            if (errs == 0) $display("RESULT: PASS");
            else           $display("RESULT: FAIL (%0d items)", errs);
            $display("--------------------------------------------------");
            $finish;
        end
    endtask

endmodule
