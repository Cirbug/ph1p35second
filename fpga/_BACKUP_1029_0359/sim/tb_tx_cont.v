//==========================================================================
// tb_tx_cont.v
//   检查 csi2_tx_packet 的 O_hs_en 在一个包内是否连续（不能有气泡）。
//   A 板的 csi_unpacket 只要 valid 掉一拍就整包作废，所以这一点必须成立。
//==========================================================================
`timescale 1ns / 1ps

module tb_tx_cont;

    localparam L  = 2;
    localparam WC = 3240;   // 2592 像素 RAW10 的实际行宽

    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    reg  [L*8-1:0] wr_data;
    reg            wr_en, pkt_end, frm_start, frm_end;
    reg  [15:0]    pkt_wc;
    wire           hs_en, hs_last, ovf, wcerr;
    wire [L*8-1:0] hs_data;

    csi2_tx_packet #(
        .LANE_NUM ( L ), .VC ( 2'b00 ), .DT ( 8'h2B ),
        .BUF_AW   ( 11 ), .LANE_SWAP ( 0 )
    ) dut (
        .I_clk(clk), .I_rst(rst),
        .I_wr_en(wr_en), .I_wr_data(wr_data), .I_pkt_end(pkt_end), .I_pkt_wc(pkt_wc),
        .I_frame_start(frm_start), .I_frame_end(frm_end),
        .O_hs_en(hs_en), .O_hs_data(hs_data), .O_hs_last(hs_last),
        .I_hs_ready(1'b1), .O_overflow(ovf), .O_wc_err(wcerr)
    );

    integer i, errs, beat, gap, npkt, len;
    integer lens [0:7];
    reg     in_burst;
    integer cur_len;

    // 监测突发：统计每段 hs_en 连续的拍数，并检查包内是否被切断
    always @(posedge clk) begin
        if (!rst) begin
            if (hs_en) begin
                if (!in_burst) begin
                    in_burst = 1; cur_len = 0;
                    if (npkt < 8) lens[npkt] = 0;
                end
                cur_len = cur_len + 1;
                if (npkt < 8) lens[npkt] = cur_len;
                beat = beat + 1;
                if (beat == 1)
                    $display("  pkt%0d beat1 = %04h (expect 2B%02h)", npkt, hs_data, WC[7:0]);
                if (beat == 2 && npkt == 1)
                    $display("  pkt%0d beat2 = %04h (expect WC_MSB=%02h ECC=3B)", npkt, hs_data, WC[15:8]);
            end else begin
                if (in_burst) begin
                    in_burst = 0;
                    npkt = npkt + 1;
                    beat = 0;
                end
            end
        end
    end

    initial begin
        errs = 0; beat = 0; npkt = 0; in_burst = 0; cur_len = 0;
        wr_en = 0; wr_data = 0; pkt_end = 0; pkt_wc = 0; frm_start = 0; frm_end = 0;
        repeat (4) @(posedge clk);
        #1 rst = 0;
        @(posedge clk);

        #1 frm_start = 1; @(posedge clk); #1 frm_start = 0;
        wr_en = 1;
        for (i = 0; i < WC/2; i = i + 1) begin
            wr_data = 16'hA000 + i; @(posedge clk); #1;
        end
        wr_en = 0;
        pkt_wc = WC; pkt_end = 1; @(posedge clk); #1 pkt_end = 0;
        frm_end = 1; @(posedge clk); #1 frm_end = 0;

        repeat (4000) @(posedge clk);
        #1;

        $display("--------------------------------------------------");
        $display("突发段数 = %0d", npkt);
        for (i = 0; i < npkt && i < 8; i = i + 1)
            $display("  段[%0d] 长度 = %0d 拍", i, lens[i]);
        // FS(2) 长包(2+WC/2+1) FE(2)
        $display("期望: FS=2, 长包=%0d, FE=2", 2 + WC/2 + 1);
        if (npkt == 3) $display("RESULT: PASS  (三个包各自连续，无气泡)");
        else           $display("RESULT: FAIL  (突发被切断，段数=%0d)", npkt);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
