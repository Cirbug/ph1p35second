//==========================================================================
// tb_csi2_tx_packet.v
//   把 csi2_tx_packet 的输出字节流"反解"回来，逐字段校验：
//     FS 短包 -> 长包（包头+载荷+CRC）-> FE 短包
//   同时校验 lane 轮询顺序（byte0 走 lane0，即 O_hs_data[15:8]）
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_tx_packet;

    localparam L  = 2;
    localparam WC = 16;          // 一行 16 字节 = 8 个 word
    localparam NW = WC / L;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    //---------------- DUT ----------------
    reg  [L*8-1:0] wr_data;
    reg            wr_en;
    reg            pkt_end;
    reg  [15:0]    pkt_wc;
    reg            frm_start;
    reg            frm_end;
    reg            hs_ready;

    wire           hs_en;
    wire [L*8-1:0] hs_data;
    wire           hs_last;
    wire           overflow;
    wire           wc_err;

    csi2_tx_packet #(
        .LANE_NUM ( L ),
        .VC       ( 2'b00 ),
        .DT       ( 8'h2B ),
        .BUF_AW   ( 10 )
    ) dut (
        .I_clk        ( clk       ),
        .I_rst        ( rst       ),
        .I_wr_en      ( wr_en     ),
        .I_wr_data    ( wr_data   ),
        .I_pkt_end    ( pkt_end   ),
        .I_pkt_wc     ( pkt_wc    ),
        .I_frame_start( frm_start ),
        .I_frame_end  ( frm_end   ),
        .O_hs_en      ( hs_en     ),
        .O_hs_data    ( hs_data   ),
        .O_hs_last    ( hs_last   ),
        .I_hs_ready   ( hs_ready  ),
        .O_overflow   ( overflow  ),
        .O_wc_err     ( wc_err    )
    );

    //---------------- 抓取输出字节流 ----------------
    reg [7:0] got [0:63];
    integer   gi;
    integer   npkt;
    integer   pkt_end_i [0:7];

    always @(posedge clk) begin
        if (!rst && hs_en) begin
            got[gi]     = hs_data[15:8];      // lane0 = 第一个字节
            got[gi + 1] = hs_data[7:0];       // lane1 = 第二个字节
            gi = gi + 2;
            if (hs_last) begin
                pkt_end_i[npkt] = gi;
                npkt = npkt + 1;
            end
        end
    end

    //---------------- 参考模型 ----------------
    reg  [23:0] ref_hdr;
    wire [7:0]  ref_ecc;

    csi2_ecc u_ref_ecc (
        .I_hdr ( ref_hdr ),
        .O_ecc ( ref_ecc )
    );

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

    //---------------- 期望载荷 ----------------
    reg [7:0] exp_pay [0:WC-1];

    integer i, p;
    integer errs;
    reg [15:0] crc_ref;
    reg [15:0] wc_be;
    reg [7:0]  di_long;

    initial begin
        errs = 0; gi = 0; npkt = 0;
        wr_en = 0; wr_data = 0; pkt_end = 0; pkt_wc = 0;
        frm_start = 0; frm_end = 0; hs_ready = 1;
        for (i = 0; i < 8; i = i + 1) pkt_end_i[i] = 0;

        rst = 1'b1;
        repeat (4) @(posedge clk);
        #1 rst = 1'b0;
        @(posedge clk);

        //-------- 帧起 --------
        #1 frm_start = 1'b1;
        @(posedge clk); #1 frm_start = 1'b0;

        //-------- 写一行载荷 --------
        for (i = 0; i < NW; i = i + 1) begin
            wr_en   = 1'b1;
            wr_data = { 8'hA0 + i[7:0], 8'h50 + i[7:0] };
            exp_pay[2*i]     = 8'hA0 + i[7:0];
            exp_pay[2*i + 1] = 8'h50 + i[7:0];
            @(posedge clk); #1;
        end
        wr_en = 1'b0;

        //-------- 行结束 --------
        pkt_wc = WC;
        pkt_end = 1'b1;
        @(posedge clk); #1 pkt_end = 1'b0;

        //-------- 帧尾 --------
        frm_end = 1'b1;
        @(posedge clk); #1 frm_end = 1'b0;

        //-------- 等发送完 --------
        repeat (200) @(posedge clk);
        #1;

        //======================== 校验 ========================
        $display("--------------------------------------------------");
        if (npkt !== 3) begin
            $display("FAIL: 期望 3 个包，实际 %0d", npkt);
            errs = errs + 1;
        end
        $display("捕获 %0d 个包，字节数分别为 %0d / %0d / %0d",
                 npkt, pkt_end_i[0], pkt_end_i[1] - pkt_end_i[0],
                 (npkt > 2) ? pkt_end_i[2] - pkt_end_i[1] : 0);

        //---- 包 1：帧起 ----
        if (pkt_end_i[0] !== 4) begin
            $display("FAIL: FS 长度 %0d，应为 4", pkt_end_i[0]);
            errs = errs + 1;
        end
        ref_hdr = { 8'h00, 8'h00, 8'h00 };
        #1;
        if (got[0] !== 8'h00 || got[1] !== 8'h00 || got[2] !== 8'h00
            || got[3] !== ref_ecc) begin
            $display("FAIL: FS 包头 %02h %02h %02h %02h，期望 00 00 00 %02h",
                     got[0], got[1], got[2], got[3], ref_ecc);
            errs = errs + 1;
        end else begin
            $display("PASS: FS  包头 00 00 00 %02h", ref_ecc);
        end

        //---- 包 2：长包 ----
        wc_be  = { got[6], got[5] };   // CSI-2: 低字节先发
        di_long = got[4];
        ref_hdr = { got[6], got[5], di_long };  // ECC 输入顺序 {WC_MSB, WC_LSB, DI}
        #1;
        if (di_long !== { 2'b00, 6'h2B }) begin
            $display("FAIL: 长包 DI=%02h，期望 2B", di_long);
            errs = errs + 1;
        end
        if (wc_be !== WC[15:0]) begin
            $display("FAIL: 长包 WC=%0d，期望 %0d", wc_be, WC);
            errs = errs + 1;
        end
        if (got[7] !== ref_ecc) begin
            $display("FAIL: 长包 ECC=%02h，期望 %02h", got[7], ref_ecc);
            errs = errs + 1;
        end else begin
            $display("PASS: 长包 DI=%02h WC=%0d ECC=%02h", di_long, wc_be, ref_ecc);
        end

        //---- 载荷 ----
        p = 8;
        for (i = 0; i < WC; i = i + 1) begin
            if (got[p + i] !== exp_pay[i]) begin
                if (errs < 6)
                    $display("FAIL: 载荷[%0d]=%02h，期望 %02h", i, got[p + i], exp_pay[i]);
                errs = errs + 1;
            end
        end

        //---- CRC ----
        crc_ref = 16'hFFFF;
        for (i = 0; i < WC; i = i + 1)
            crc_ref = crc_byte(crc_ref, exp_pay[i]);
        if ({ got[8 + WC], got[9 + WC] } !== crc_ref) begin
            $display("FAIL: CRC=%02h%02h，期望 %04h",
                     got[8 + WC], got[9 + WC], crc_ref);
            errs = errs + 1;
        end else begin
            $display("PASS: 载荷 %0d 字节全对，CRC=%04h", WC, crc_ref);
        end

        //---- 包 3：帧尾 ----
        ref_hdr = { 8'h00, 8'h00, 8'h01 };   // ECC 输入顺序 {WC_MSB, WC_LSB, DI}
        #1;
        if (got[26] !== 8'h01 || got[27] !== 8'h00 || got[28] !== 8'h00
            || got[29] !== ref_ecc) begin
            $display("FAIL: FE 包头 %02h %02h %02h %02h，期望 01 00 00 %02h",
                     got[26], got[27], got[28], got[29], ref_ecc);
            errs = errs + 1;
        end else begin
            $display("PASS: FE  包头 01 00 00 %02h", ref_ecc);
        end

        if (overflow) begin $display("FAIL: 出现 overflow"); errs = errs + 1; end
        if (wc_err)   begin $display("FAIL: 出现 wc_err");   errs = errs + 1; end

        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
