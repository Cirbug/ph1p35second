//==========================================================================
// tb_csi2_pipeline.v
//   端到端回环：csi2_rx_parse -> csi2_tx_packet
//   喂进 FS + 长包(WC=32) + FE，再把发出来的包解回来，
//   要求字节级完全等价（DT/WC/载荷/CRC 都对得上）。
//   这一层通了，说明 B 板的"协议中继"逻辑成立。
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_pipeline;

    localparam L  = 2;
    localparam WC = 32;              // 一行 32 字节 = 16 个 word

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    //---------------- RX 侧驱动 ----------------
    reg  [L*8-1:0] rx_data;
    reg            rx_valid;

    //---------------- RX -> TX ----------------
    wire           p_wr_en;
    wire [L*8-1:0] p_wr_data;
    wire           p_pkt_end;
    wire [15:0]    p_pkt_wc;
    wire [7:0]     p_pkt_dt;
    wire           p_fs, p_fe, p_wc_err;

    csi2_rx_parse #( .LANE_NUM ( L ) ) u_rx (
        .I_clk        ( clk       ),
        .I_rst        ( rst       ),
        .I_hs_valid   ( rx_valid  ),
        .I_hs_data    ( rx_data   ),
        .O_wr_en      ( p_wr_en   ),
        .O_wr_data    ( p_wr_data ),
        .O_pkt_end    ( p_pkt_end ),
        .O_pkt_wc     ( p_pkt_wc  ),
        .O_pkt_dt     ( p_pkt_dt  ),
        .O_frame_start( p_fs      ),
        .O_frame_end  ( p_fe      ),
        .O_wc_err     ( p_wc_err  )
    );

    //---------------- TX 侧 ----------------
    wire           t_en;
    wire [L*8-1:0] t_data;
    wire           t_last;
    wire           t_ovf, t_wcerr;

    csi2_tx_packet #(
        .LANE_NUM ( L ),
        .VC       ( 2'b00 ),
        .DT       ( 8'h2B ),
        .BUF_AW   ( 10 )
    ) u_tx (
        .I_clk        ( clk       ),
        .I_rst        ( rst       ),
        .I_wr_en      ( p_wr_en   ),
        .I_wr_data    ( p_wr_data ),
        .I_pkt_end    ( p_pkt_end ),
        .I_pkt_wc     ( p_pkt_wc  ),
        .I_frame_start( p_fs      ),
        .I_frame_end  ( p_fe      ),
        .O_hs_en      ( t_en      ),
        .O_hs_data    ( t_data    ),
        .O_hs_last    ( t_last    ),
        .I_hs_ready   ( 1'b1      ),
        .O_overflow   ( t_ovf     ),
        .O_wc_err     ( t_wcerr   )
    );

    //---------------- 采集发送出来的字节流 ----------------
    reg [7:0] got [0:127];
    integer   gi, npkt;
    integer   pend_i [0:7];

    always @(posedge clk) begin
        if (!rst && t_en) begin
            got[gi]     = t_data[15:8];
            got[gi + 1] = t_data[7:0];
            gi = gi + 2;
            if (t_last) begin
                pend_i[npkt] = gi;
                npkt = npkt + 1;
            end
        end
    end

    //---------------- 参考 ECC ----------------
    reg  [23:0] ref_hdr;
    wire [7:0]  ref_ecc;
    csi2_ecc u_ecc ( .I_hdr ( ref_hdr ), .O_ecc ( ref_ecc ) );

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

    //---------------- 驱动任务 ----------------
    task drv_beat;
        input [15:0] d;
        begin
            rx_valid = 1'b1;
            rx_data  = d;
            @(posedge clk); #1;
        end
    endtask

    task drv_gap;
        input integer n;
        begin
            rx_valid = 1'b0;
            rx_data  = 16'h0000;
            repeat (n) @(posedge clk); #1;
        end
    endtask

    //---------------- 主流程 ----------------
    reg [7:0]  exp_pay [0:WC-1];
    integer    i, p;
    integer    errs;
    reg [15:0] crc_ref;

    initial begin
        errs = 0; gi = 0; npkt = 0;
        rx_valid = 1'b0; rx_data = 16'h0000;
        for (i = 0; i < 8; i = i + 1) pend_i[i] = 0;

        rst = 1'b1;
        repeat (4) @(posedge clk);
        #1 rst = 1'b0;
        @(posedge clk);

        //-------- FS --------
        ref_hdr = 24'h00_0000; #1;
        drv_beat({ 8'h00, 8'h00 });
        drv_beat({ 8'h00, ref_ecc });
        drv_gap(6);

        //-------- 长包 WC=32 --------
        drv_beat({ 8'h2B, WC[15:8] });
        ref_hdr = { 8'h2B, WC[15:0] }; #1;
        drv_beat({ WC[7:0], ref_ecc });

        for (i = 0; i < WC/2; i = i + 1) begin
            exp_pay[2*i]     = 8'hA0 + i[7:0];
            exp_pay[2*i + 1] = 8'h50 + i[7:0];
            drv_beat({ 8'hA0 + i[7:0], 8'h50 + i[7:0] });
        end
        drv_beat({ 8'hC0, 8'hDE });        // 摄像头给的 CRC，应被丢弃
        drv_gap(6);

        //-------- FE --------
        ref_hdr = 24'h01_0000; #1;
        drv_beat({ 8'h01, 8'h00 });
        drv_beat({ 8'h00, ref_ecc });
        drv_gap(20);

        repeat (100) @(posedge clk);
        #1;

        //==================== 校验 ====================
        $display("--------------------------------------------------");
        $display("发送侧捕获 %0d 个包，长度 %0d / %0d / %0d", npkt,
                 pend_i[0], pend_i[1]-pend_i[0],
                 (npkt>2) ? pend_i[2]-pend_i[1] : 0);

        if (npkt !== 3) begin
            $display("FAIL: 应发出 3 个包，实际 %0d", npkt);
            errs = errs + 1;
        end

        // FS
        ref_hdr = 24'h00_0000; #1;
        if (got[0]!==8'h00 || got[1]!==8'h00 || got[2]!==8'h00 || got[3]!==ref_ecc) begin
            $display("FAIL: FS 出错"); errs = errs + 1;
        end else $display("PASS: FS  = 00 00 00 %02h", ref_ecc);

        // 长包
        ref_hdr = { got[4], got[5], got[6] }; #1;
        if (got[4] !== 8'h2B) begin
            $display("FAIL: 长包 DT=%02h", got[4]); errs = errs + 1;
        end
        if ({got[5],got[6]} !== WC[15:0]) begin
            $display("FAIL: 长包 WC=%0d", {got[5],got[6]}); errs = errs + 1;
        end
        if (got[7] !== ref_ecc) begin
            $display("FAIL: 长包 ECC 错"); errs = errs + 1;
        end

        // 载荷
        p = 8;
        for (i = 0; i < WC; i = i + 1) begin
            if (got[p+i] !== exp_pay[i]) begin
                if (errs < 6) $display("FAIL: 载荷[%0d]=%02h 期望 %02h", i, got[p+i], exp_pay[i]);
                errs = errs + 1;
            end
        end

        // CRC
        crc_ref = 16'hFFFF;
        for (i = 0; i < WC; i = i + 1) crc_ref = crc_byte(crc_ref, exp_pay[i]);
        if ({got[8+WC], got[9+WC]} !== crc_ref) begin
            $display("FAIL: CRC=%04h 期望 %04h", {got[8+WC],got[9+WC]}, crc_ref);
            errs = errs + 1;
        end

        // FE
        ref_hdr = 24'h01_0000; #1;
        if (got[8+WC+2] !== 8'h01 || got[8+WC+5] !== ref_ecc) begin
            $display("FAIL: FE 出错"); errs = errs + 1;
        end else $display("PASS: FE  = 01 00 00 %02h", ref_ecc);

        if (errs == 0)
            $display("PASS: 长包 DT/WC/ECC/载荷(%0d字节)/CRC 全等价", WC);

        if (t_ovf || p_wc_err || t_wcerr) begin
            $display("FAIL: ovf=%0b wc_err=%0b t_wcerr=%0b", t_ovf, p_wc_err, t_wcerr);
            errs = errs + 1;
        end

        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS   (回环等价)");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
