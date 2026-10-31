//==========================================================================
// tb_csi2_lane_swap.v
//   验证 LANE_SWAP=1 时 lane 顺序被正确反转（应对排线反向/RX 端接反）。
//
//   送一个 WC=4 的长包，载荷 AA BB CC DD：
//     正常   : {2B,00} {04,ECC} {AA,BB} {CC,DD} {CRC_H,CRC_L}
//     交换后 : {00,2B} {ECC,04} {BB,AA} {DD,CC} {CRC_L,CRC_H}
//   字节的"串行顺序"不变，只是落到物理 lane 的位置反了。
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_lane_swap;

    localparam L  = 2;
    localparam WC = 4;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    reg  [L*8-1:0] wr_data;
    reg            wr_en, pkt_end, frm_start, frm_end;
    reg  [15:0]    pkt_wc;
    wire           hs_en, hs_last, ovf, wcerr;
    wire [L*8-1:0] hs_data;

    csi2_tx_packet #(
        .LANE_NUM  ( L ),
        .VC        ( 2'b00 ),
        .DT        ( 8'h2B ),
        .BUF_AW    ( 10 ),
        .LANE_SWAP ( 1 )
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

    reg  [23:0] ref_hdr;
    wire [7:0]  ref_ecc;
    csi2_ecc u_ecc ( .I_hdr ( ref_hdr ), .O_ecc ( ref_ecc ) );

    //---------------- 采集每一拍 ----------------
    reg [15:0] beat [0:15];
    integer    nb;

    always @(posedge clk) begin
        if (!rst && hs_en) begin
            if (nb < 16) beat[nb] = hs_data;
            nb = nb + 1;
        end
    end

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
    reg [7:0]  ecc_l;
    reg [7:0]  ecc_fs;
    reg [7:0]  ecc_fe;

    initial begin
        errs = 0; nb = 0;
        wr_en = 0; wr_data = 0; pkt_end = 0; pkt_wc = 0;
        frm_start = 0; frm_end = 0;

        repeat (4) @(posedge clk);
        #1 rst = 1'b0;
        @(posedge clk);

        #1 frm_start = 1'b1;
        @(posedge clk); #1 frm_start = 1'b0;

        // 预先算好 ECC
        ref_hdr = { 8'h00, 8'h04, 8'h2B }; #1;   // {WC_MSB, WC_LSB, DI}
        ecc_l = ref_ecc;
        ref_hdr = { 8'h00, 8'h00, 8'h00 }; #1;
        ecc_fs = ref_ecc;
        ref_hdr = { 8'h00, 8'h00, 8'h01 }; #1;
        ecc_fe = ref_ecc;

        wr_en = 1'b1;
        wr_data = 16'hAABB; @(posedge clk); #1;
        wr_data = 16'hCCDD; @(posedge clk); #1;
        wr_en = 1'b0;

        pkt_wc = WC; pkt_end = 1'b1;
        @(posedge clk); #1 pkt_end = 1'b0;

        frm_end = 1'b1;
        @(posedge clk); #1 frm_end = 1'b0;

        repeat (80) @(posedge clk);
        #1;

        crc_ref = 16'hFFFF;
        crc_ref = crc_byte(crc_ref, 8'hAA);
        crc_ref = crc_byte(crc_ref, 8'hBB);
        crc_ref = crc_byte(crc_ref, 8'hCC);
        crc_ref = crc_byte(crc_ref, 8'hDD);

        $display("--------------------------------------------------");
        $display("beats captured = %0d", nb);

        // 期望（LANE_SWAP=1）：包序 FS(2拍) + 长包(5拍) + FE(2拍) = 9 拍
        check_beat(0, { 8'h00,  8'h00              }, "FS   beat0");
        check_beat(1, { 8'h00,  ecc_fs             }, "FS   beat1");
        check_beat(2, { 8'h04,  8'h2B              }, "long beat0");   // {WC_LSB, DI} 交换后
        check_beat(3, { ecc_l,  8'h00              }, "long beat1");   // {ECC, WC_MSB} 交换后
        check_beat(4, { 8'hBB,  8'hAA              }, "pay  beat0");
        check_beat(5, { 8'hDD,  8'hCC              }, "pay  beat1");
        check_beat(6, { crc_ref[7:0], crc_ref[15:8]}, "CRC  beat");
        check_beat(7, { 8'h00,  8'h01              }, "FE   beat0");
        check_beat(8, { ecc_fe, 8'h00              }, "FE   beat1");

        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS  (lane 顺序已反转)");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

    task check_beat;
        input integer      idx;
        input [15:0]       exp;
        input [8*16-1:0]   name;
        begin
            if (beat[idx] !== exp) begin
                $display("FAIL: %0s got %04h exp %04h", name, beat[idx], exp);
                errs = errs + 1;
            end else begin
                $display("PASS: %0s = %04h", name, beat[idx]);
            end
        end
    endtask

endmodule
