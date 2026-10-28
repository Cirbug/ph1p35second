//==========================================================================
// tb_csi2_full_line.v
//   真实尺寸回归：走完整顶层链路，送一行与摄像头实际一致的宽行。
//
//   摄像头（SC500CS FHD 模式）输出 1928 像素/行：
//     RAW10 每行字节数 = 1928*10/8 = 2410 字节 = 1205 word
//
//   本测试同时验证：
//     - 行缓冲深度够用（BUF_AW=11 -> 2048 word/bank）
//     - 速率比接近实际（RX 10MHz : TX 25MHz ≈ 1:2.5，实际约 1:2.6）
//     - 长行下 WC 透传、载荷、CRC 仍然全部正确
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_full_line;

    localparam WC  = 2410;               // 真实行字节数（1928 像素）
    localparam NW  = WC / 2;             // 1205 个 word

    reg sysclk = 1'b0;
    reg rst_n  = 1'b0;
    always #20 sysclk = ~sysclk;         // 25MHz

    wire O_cam_scl, O_cam_24m, O_cam_rst, O_led_cfg, O_led_beat, O_led_err;
    wire IO_cam_sda;

    csi2_tx_top dut (
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

    //---------------- 采集 TX 输出（容量按整行 + 包头 + CRC + 余量）------------
    localparam MAXB = WC + 64;
    reg [7:0] got [0:MAXB-1];
    integer   gi, npkt;
    integer   pend_i [0:3];

    always @(posedge dut.tx_clk) begin
        if (rst_n && dut.tx_hs_en) begin
            if (gi < MAXB)     got[gi]     = dut.tx_hs_data[15:8];
            if (gi + 1 < MAXB) got[gi + 1] = dut.tx_hs_data[7:0];
            gi = gi + 2;
            if (dut.tx_hs_last) begin
                pend_i[npkt] = gi;
                npkt = npkt + 1;
            end
        end
    end

    //---------------- RX 驱动 ----------------
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

    integer    i, p, errs;
    reg [15:0] crc_ref;
    reg [15:0] w;

    initial begin
        errs = 0; gi = 0; npkt = 0;
        for (i = 0; i < 4; i = i + 1) pend_i[i] = 0;

        rst_n = 1'b0;
        repeat (6) @(posedge sysclk);
        rst_n = 1'b1;
        repeat (6) @(posedge sysclk);

        //-------- FS --------
        ref_hdr = 24'h00_0000; #1;
        rx_beat({ 8'h00, 8'h00 });
        rx_beat({ 8'h00, ref_ecc });
        rx_gap(8);

        //-------- 长包：真实行宽 --------
        rx_beat({ 8'h2B, WC[15:8] });
        ref_hdr = { 8'h2B, WC[15:0] }; #1;
        rx_beat({ WC[7:0], ref_ecc });

        for (i = 0; i < NW; i = i + 1) begin
            w = { 8'h10 + i[7:0], 8'h40 + i[7:0] };
            rx_beat(w);
        end
        rx_beat({ 8'hC0, 8'hDE });           // 摄像头 CRC，应丢弃
        rx_gap(8);

        //-------- FE --------
        ref_hdr = 24'h01_0000; #1;
        rx_beat({ 8'h01, 8'h00 });
        rx_beat({ 8'h00, ref_ecc });
        rx_gap(10);

        repeat (4000) @(posedge dut.tx_clk);

        //==================== 校验 ====================
        $display("--------------------------------------------------");
        $display("WC=%0d  NW=%0d  captured packets=%0d  length=%0d/%0d/%0d",
                 WC, NW, npkt, pend_i[0],
                 pend_i[1]-pend_i[0],
                 (npkt>2) ? pend_i[2]-pend_i[1] : 0);

        if (npkt !== 3) begin
            $display("FAIL: expect 3 packets, got %0d", npkt);
            errs = errs + 1;
        end

        if ((pend_i[1]-pend_i[0]) !== (WC + 6)) begin
            $display("FAIL: long packet length %0d, expect %0d",
                     pend_i[1]-pend_i[0], WC+6);
            errs = errs + 1;
        end

        // 包头
        ref_hdr = { got[4], got[5], got[6] }; #1;
        if (got[4] !== 8'h2B)              begin $display("FAIL: DT=%02h", got[4]);      errs=errs+1; end
        if ({got[5],got[6]} !== WC[15:0])  begin $display("FAIL: WC hdr=%0d", {got[5],got[6]}); errs=errs+1; end
        if (got[7] !== ref_ecc)            begin $display("FAIL: ECC=%02h", got[7]);     errs=errs+1; end

        // 载荷
        p = 8;
        for (i = 0; i < NW; i = i + 1) begin
            if (got[p + 2*i]     !== (8'h10 + i[7:0])) begin
                if (errs < 8) $display("FAIL: payload[%0d].hi=%02h exp %02h",
                                       2*i, got[p+2*i], 8'h10+i[7:0]);
                errs = errs + 1;
            end
            if (got[p + 2*i + 1] !== (8'h40 + i[7:0])) begin
                if (errs < 8) $display("FAIL: payload[%0d].lo=%02h exp %02h",
                                       2*i+1, got[p+2*i+1], 8'h40+i[7:0]);
                errs = errs + 1;
            end
        end

        // CRC
        crc_ref = 16'hFFFF;
        for (i = 0; i < NW; i = i + 1) begin
            crc_ref = crc_byte(crc_ref, 8'h10 + i[7:0]);
            crc_ref = crc_byte(crc_ref, 8'h40 + i[7:0]);
        end
        if ({got[8+WC], got[9+WC]} !== crc_ref) begin
            $display("FAIL: CRC=%04h exp %04h", {got[8+WC],got[9+WC]}, crc_ref);
            errs = errs + 1;
        end

        // FE
        ref_hdr = 24'h01_0000; #1;
        if (got[8+WC+2] !== 8'h01 || got[8+WC+5] !== ref_ecc) begin
            $display("FAIL: FE wrong"); errs = errs + 1;
        end

        if (O_led_err) begin
            $display("FAIL: O_led_err asserted (overflow or wc_err)");
            errs = errs + 1;
        end

        if (errs == 0)
            $display("PASS: full line WC=%0d (%0d word) relayed intact, no overflow",
                     WC, NW);

        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
