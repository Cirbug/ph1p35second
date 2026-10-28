//==========================================================================
// tb_csi2_bank_switch.v
//   针对性回归：验证"上一行还在发送时，下一行已写满"的场景不会污染数据。
//
//   触发条件：行长度悬殊。
//     line1 = 1205 word（发送需约 1215 拍）
//     line2 =   10 word（写入只需 10 拍，必然落在 line1 发送期间）
//
//   旧实现把"刚填满的 bank"和"正在发送的 bank"合用同一个寄存器，
//   于是 line2 完成时读指针会突然跳到 line2 的 bank，line1 后半段被静默污染。
//   本测试即为此而设：两个包的内容都必须与各自源数据完全一致。
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_bank_switch;

    localparam L    = 2;
    localparam NW1  = 1205;          // line1 word 数
    localparam WC1  = NW1 * L;       // 2410 字节
    localparam NW2  = 10;
    localparam WC2  = NW2 * L;       // 20 字节

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;            // 100MHz

    reg  [L*8-1:0] wr_data;
    reg            wr_en, pkt_end, frm_start, frm_end;
    reg  [15:0]    pkt_wc;
    wire           hs_en, hs_last, ovf, wcerr;
    wire [L*8-1:0] hs_data;

    csi2_tx_packet #(
        .LANE_NUM ( L ),
        .VC       ( 2'b00 ),
        .DT       ( 8'h2B ),
        .BUF_AW   ( 11 )
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
        .I_hs_ready   ( 1'b1      ),
        .O_overflow   ( ovf       ),
        .O_wc_err     ( wcerr     )
    );

    //---------------- 采集 ----------------
    localparam MAXB = WC1 + WC2 + 64;
    reg [7:0] got [0:MAXB-1];
    integer   gi, npkt;
    integer   pend_i [0:3];

    always @(posedge clk) begin
        if (!rst && hs_en) begin
            if (gi < MAXB)     got[gi]     = hs_data[15:8];
            if (gi+1 < MAXB)   got[gi + 1] = hs_data[7:0];
            gi = gi + 2;
            if (hs_last) begin
                pend_i[npkt] = gi;
                npkt = npkt + 1;
            end
        end
    end

    //---------------- 源数据 ----------------
    function [7:0] pay1_hi; input integer k; pay1_hi = 8'h20 + k[7:0]; endfunction
    function [7:0] pay1_lo; input integer k; pay1_lo = 8'h60 + k[7:0]; endfunction
    function [7:0] pay2_hi; input integer k; pay2_hi = 8'hE0 + k[7:0]; endfunction
    function [7:0] pay2_lo; input integer k; pay2_lo = 8'hF0 + k[7:0]; endfunction

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

    integer    i, errs;
    reg [15:0] crc_ref;
    reg [15:0] exp_w;

    initial begin
        errs = 0; gi = 0; npkt = 0;
        wr_en = 0; wr_data = 0; pkt_end = 0; pkt_wc = 0;
        frm_start = 0; frm_end = 0;
        for (i = 0; i < 4; i = i + 1) pend_i[i] = 0;

        repeat (4) @(posedge clk);
        #1 rst = 1'b0;
        @(posedge clk);

        //-------- 帧起 --------
        #1 frm_start = 1'b1;
        @(posedge clk); #1 frm_start = 1'b0;

        //-------- line1：1205 word --------
        for (i = 0; i < NW1; i = i + 1) begin
            wr_en   = 1'b1;
            wr_data = { pay1_hi(i), pay1_lo(i) };
            @(posedge clk); #1;
        end
        wr_en = 1'b0;
        pkt_wc = WC1; pkt_end = 1'b1;
        @(posedge clk); #1 pkt_end = 1'b0;

        //-------- 立刻塞 line2：只有 10 word --------
        //  此时期望 tx_packet 刚开始发 line1（约 1~2 拍），
        //  line2 在 10 拍内写完，形成"发送中切 bank"的条件。
        for (i = 0; i < NW2; i = i + 1) begin
            wr_en   = 1'b1;
            wr_data = { pay2_hi(i), pay2_lo(i) };
            @(posedge clk); #1;
        end
        wr_en = 1'b0;
        pkt_wc = WC2; pkt_end = 1'b1;
        @(posedge clk); #1 pkt_end = 1'b0;

        frm_end = 1'b1;
        @(posedge clk); #1 frm_end = 1'b0;

        repeat (3000) @(posedge clk);
        #1;
        if (npkt !== 4) begin
            $display("FAIL: expect 3 packets, got %0d", npkt);
            errs = errs + 1;
        end

        //-------- 包 1：line1 --------
        for (i = 0; i < NW1; i = i + 1) begin
            exp_w = { pay1_hi(i), pay1_lo(i) };
            if (got[8 + 2*i] !== exp_w[15:8] || got[9 + 2*i] !== exp_w[7:0]) begin
                if (errs < 5)
                    $display("FAIL: line1 word[%0d]=%02h%02h exp %02h%02h",
                             i, got[8+2*i], got[9+2*i], exp_w[15:8], exp_w[7:0]);
                errs = errs + 1;
            end
        end
        crc_ref = 16'hFFFF;
        for (i = 0; i < NW1; i = i + 1) begin
            crc_ref = crc_byte(crc_ref, pay1_hi(i));
            crc_ref = crc_byte(crc_ref, pay1_lo(i));
        end
        if ({got[8+WC1], got[9+WC1]} !== crc_ref) begin
            $display("FAIL: line1 CRC=%04h exp %04h", {got[8+WC1],got[9+WC1]}, crc_ref);
            errs = errs + 1;
        end

        //-------- 包 2：line2 --------
        for (i = 0; i < NW2; i = i + 1) begin
            exp_w = { pay2_hi(i), pay2_lo(i) };
            if (got[8 + WC1 + 2 + 4 + 2*i] !== exp_w[15:8] ||
                got[9 + WC1 + 2 + 4 + 2*i] !== exp_w[7:0]) begin
                $display("FAIL: line2 word[%0d]=%02h%02h exp %02h%02h", i,
                         got[8+WC1+2+4+2*i], got[9+WC1+2+4+2*i],
                         exp_w[15:8], exp_w[7:0]);
                errs = errs + 1;
            end
        end

        if (errs == 0)
            $display("PASS: line1(%0d word) 与 line2(%0d word) 均未被污染", NW1, NW2);

        if (wcerr) begin $display("FAIL: wc_err"); errs = errs + 1; end

        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
