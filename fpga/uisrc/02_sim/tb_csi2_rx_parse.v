//==========================================================================
// tb_csi2_rx_parse.v
//   构造一段 CSI-2 包流：FS 短包 -> 长包(WC=16) -> FE 短包
//   验证 csi2_rx_parse 能正确：
//     - 产生 frame_start / frame_end 脉冲
//     - 把长包载荷逐 word 搬出来，并给出 pkt_end / pkt_wc / pkt_dt
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_rx_parse;

    localparam L  = 2;
    localparam WC = 16;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    reg  [L*8-1:0] hs_data;
    reg            hs_valid;

    wire           wr_en;
    wire [L*8-1:0] wr_data;
    wire           pkt_end;
    wire [15:0]    pkt_wc;
    wire [7:0]     pkt_dt;
    wire           frm_start;
    wire           frm_end;
    wire           wc_err;

    csi2_rx_parse #( .LANE_NUM ( L ) ) dut (
        .I_clk        ( clk       ),
        .I_rst        ( rst       ),
        .I_hs_valid   ( hs_valid  ),
        .I_hs_data    ( hs_data   ),
        .O_wr_en      ( wr_en     ),
        .O_wr_data    ( wr_data   ),
        .O_pkt_end    ( pkt_end   ),
        .O_pkt_wc     ( pkt_wc    ),
        .O_pkt_dt     ( pkt_dt    ),
        .O_frame_start( frm_start ),
        .O_frame_end  ( frm_end   ),
        .O_wc_err     ( wc_err    )
    );

    //---------------- 参考 ECC ----------------
    reg  [23:0] ref_hdr;
    wire [7:0]  ref_ecc;
    csi2_ecc u_ecc ( .I_hdr ( ref_hdr ), .O_ecc ( ref_ecc ) );

    //---------------- 采集输出 ----------------
    reg [15:0] cap [0:63];
    integer    wen_cnt;
    integer    pend_cnt;
    integer    fs_cnt, fe_cnt;
    reg [15:0] wc_cap;
    reg [7:0]  dt_cap;

    always @(posedge clk) begin
        if (!rst) begin
            if (wr_en) begin
                cap[wen_cnt] = wr_data;
                wen_cnt = wen_cnt + 1;
            end
            if (pkt_end) begin
                pend_cnt = pend_cnt + 1;
                wc_cap   = pkt_wc;
                dt_cap   = pkt_dt;
            end
            if (frm_start) fs_cnt = fs_cnt + 1;
            if (frm_end)   fe_cnt = fe_cnt + 1;
        end
    end

    //---------------- 驱动任务 ----------------
    task drv_beat;
        input [15:0] d;
        begin
            hs_valid = 1'b1;
            hs_data  = d;
            @(posedge clk); #1;
        end
    endtask

    task drv_gap;
        input integer n;
        begin
            hs_valid = 1'b0;
            hs_data  = 16'h0000;
            repeat (n) @(posedge clk); #1;
        end
    endtask

    //---------------- 主流程 ----------------
    integer i;
    integer errs;

    initial begin
        errs = 0; wen_cnt = 0; pend_cnt = 0; fs_cnt = 0; fe_cnt = 0;
        wc_cap = 16'd0; dt_cap = 8'd0;
        hs_valid = 1'b0; hs_data = 16'h0000;

        rst = 1'b1;
        repeat (4) @(posedge clk);
        #1 rst = 1'b0;
        @(posedge clk);

        //--------------------------------------------------
        // 包 1：FS 短包  DI=0x00, WC=0
        //--------------------------------------------------
        ref_hdr = 24'h00_0000; #1;
        drv_beat({ 8'h00, 8'h00 });
        drv_beat({ 8'h00, ref_ecc });
        drv_gap(6);

        //--------------------------------------------------
        // 包 2：长包  DI=0x2B, WC=16
        //--------------------------------------------------
        drv_beat({ 8'h2B, WC[15:8] });
        ref_hdr = 24'h2B_0010; #1;
        drv_beat({ WC[7:0], ref_ecc });

        for (i = 0; i < WC/2; i = i + 1)
            drv_beat({ 8'hA0 + i[7:0], 8'h50 + i[7:0] });

        drv_beat({ 8'hDE, 8'hAD });          // CRC 字节，应被忽略
        drv_gap(6);

        //--------------------------------------------------
        // 包 3：FE 短包  DI=0x01, WC=0
        //--------------------------------------------------
        ref_hdr = 24'h01_0000; #1;
        drv_beat({ 8'h01, 8'h00 });
        drv_beat({ 8'h00, ref_ecc });
        drv_gap(10);

        //==================== 校验 ====================
        $display("--------------------------------------------------");
        $display("frame_start=%0d  frame_end=%0d  pkt_end=%0d  wr_en=%0d",
                 fs_cnt, fe_cnt, pend_cnt, wen_cnt);

        if (fs_cnt !== 1)  begin $display("FAIL: frame_start 应为 1 次"); errs = errs + 1; end
        if (fe_cnt !== 1)  begin $display("FAIL: frame_end 应为 1 次");   errs = errs + 1; end
        if (pend_cnt !== 1)begin $display("FAIL: pkt_end 应为 1 次");     errs = errs + 1; end
        if (wen_cnt !== WC/2) begin
            $display("FAIL: wr_en 次数 %0d，应为 %0d", wen_cnt, WC/2);
            errs = errs + 1;
        end

        if (wc_cap !== WC[15:0]) begin
            $display("FAIL: pkt_wc=%0d，应为 %0d", wc_cap, WC);
            errs = errs + 1;
        end else begin
            $display("PASS: pkt_wc=%0d", wc_cap);
        end

        if (dt_cap !== 8'h2B) begin
            $display("FAIL: pkt_dt=%02h，应为 2B", dt_cap);
            errs = errs + 1;
        end else begin
            $display("PASS: pkt_dt=%02h", dt_cap);
        end

        for (i = 0; i < WC/2; i = i + 1) begin
            if (cap[i] !== { 8'hA0 + i[7:0], 8'h50 + i[7:0] }) begin
                if (errs < 6)
                    $display("FAIL: 载荷 word[%0d]=%04h，期望 %02h%02h",
                             i, cap[i], 8'hA0 + i[7:0], 8'h50 + i[7:0]);
                errs = errs + 1;
            end
        end
        if (errs == 0) $display("PASS: %0d 个载荷 word 全部正确", WC/2);

        if (wc_err) begin $display("FAIL: 出现 wc_err"); errs = errs + 1; end

        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
