//==========================================================================
// tb_csi2_ecc_crc.v
//   验证 uisrc/01_rtl/csi2/csi2_ecc.v 与 crc16_csi.v
//   1) ECC：24 个单比特错误必须产生 24 个互不相同、且非零的 syndrome
//   2) CRC：标准校验向量 "123456789" 必须得到 0x29B1
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_ecc_crc;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    //---------------- ECC ----------------
    reg  [23:0] hdr;
    wire [7:0]  ecc;

    csi2_ecc u_ecc (
        .I_hdr ( hdr ),
        .O_ecc ( ecc )
    );

    //---------------- CRC ----------------
    reg         crc_init;
    reg         crc_valid;
    reg  [7:0]  crc_data;
    wire [15:0] crc;

    crc16_csi u_crc (
        .I_clk   ( clk       ),
        .I_rst   ( rst       ),
        .I_init  ( crc_init  ),
        .I_valid ( crc_valid ),
        .I_data  ( crc_data  ),
        .O_crc   ( crc       )
    );

    //---------------- 测试 ----------------
    integer       i;
    integer       errs;
    reg [7:0]     base_ecc;
    reg [7:0]     synd;
    reg [7:0]     seen [0:255];
    reg [8*9-1:0] msg = "123456789";

    initial begin
        errs      = 0;
        hdr       = 24'h2B_05_00;
        crc_init  = 1'b0;
        crc_valid = 1'b0;
        crc_data  = 8'h00;

        //--------------------------------------------------
        // 1) ECC 单比特错误 syndrome 唯一性
        //--------------------------------------------------
        hdr = 24'h2B_05_00;
        #1;
        base_ecc = ecc;
        $display("[ECC] hdr=0x%06h  ecc=0x%02h", hdr, base_ecc);

        if (base_ecc[6] !== 1'b0) begin
            $display("[ECC] FAIL: P6 必须为 0");
            errs = errs + 1;
        end

        for (i = 0; i < 256; i = i + 1) seen[i] = 8'h00;
        seen[0] = 8'hFF;                      // syndrome=0 视为已占用

        for (i = 0; i < 24; i = i + 1) begin
            hdr = 24'h2B_05_00 ^ (24'h000001 << i);
            #1;
            synd = ecc ^ base_ecc;
            if (synd == 8'h00) begin
                $display("[ECC] FAIL: bit %0d 翻转后 syndrome 仍为 0", i);
                errs = errs + 1;
            end else if (seen[synd] != 8'h00) begin
                $display("[ECC] FAIL: bit %0d syndrome=0x%02h 与 bit %0d 重复",
                         i, synd, seen[synd] - 1);
                errs = errs + 1;
            end else begin
                seen[synd] = i[7:0] + 8'h01;
            end
        end
        $display("[ECC] 24 个单比特错误 syndrome 唯一性检查完成");

        //--------------------------------------------------
        // 2) CRC 标准向量
        //--------------------------------------------------
        rst       = 1'b0;
        crc_init  = 1'b1;
        @(posedge clk); #1;
        crc_init  = 1'b0;

        for (i = 0; i < 9; i = i + 1) begin
            crc_valid = 1'b1;
            crc_data  = msg[8*(8-i) +: 8];
            @(posedge clk); #1;
        end
        crc_valid = 1'b0;
        #1;

        if (crc !== 16'h29B1) begin
            $display("[CRC] FAIL: \"123456789\" -> 0x%04h，期望 0x29B1", crc);
            errs = errs + 1;
        end else begin
            $display("[CRC] PASS: \"123456789\" -> 0x%04h", crc);
        end

        //--------------------------------------------------
        $display("------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("------------------------------------------------");
        $finish;
    end

endmodule
