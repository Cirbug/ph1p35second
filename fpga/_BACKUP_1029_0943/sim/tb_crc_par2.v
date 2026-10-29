//==========================================================================
// tb_crc_par2.v
//   交叉验证：crc16_par2（2 字节/拍并行）必须与 crc16_csi（1 字节/拍串行）
//             对同样的字节流逐位一致。
//   用确定性伪随机序列跑 512 个字（1024 字节）。
//==========================================================================
`timescale 1ns / 1ps

module tb_crc_par2;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    //---------------- 参考实现：1 字节/拍 ----------------
    reg         s_init;
    reg         s_valid;
    reg  [7:0]  s_data;
    wire [15:0] s_crc;

    crc16_csi u_ser (
        .I_clk   ( clk     ),
        .I_rst   ( rst     ),
        .I_init  ( s_init  ),
        .I_valid ( s_valid ),
        .I_data  ( s_data  ),
        .O_crc   ( s_crc   )
    );

    //---------------- 被测：2 字节/拍并行 ----------------
    reg  [15:0] p_valid;
    reg  [15:0] p_data;
    wire [15:0] p_next;

    crc16_par2 u_par (
        .I_crc  ( p_valid ),
        .I_data ( p_data  ),
        .O_crc  ( p_next  )
    );

    reg [15:0] acc;                 // 并行路径的累积寄存器
    reg [15:0] lfsr;                // 伪随机源
    reg [7:0]  b0, b1;
    integer    i;
    integer    errs;

    initial begin
        errs    = 0;
        lfsr    = 16'hACE1;
        s_init  = 1'b0;
        s_valid = 1'b0;
        s_data  = 8'h00;
        p_valid = 16'hFFFF;
        p_data  = 16'h0000;

        rst = 1'b1;
        @(posedge clk); #1;
        rst = 1'b0;

        // 两条路径都从 0xFFFF 起
        s_init = 1'b1;
        p_valid = 16'hFFFF;
        @(posedge clk); #1;
        s_init = 1'b0;

        acc = 16'hFFFF;

        for (i = 0; i < 512; i = i + 1) begin
            // 产生一个字（带一点随机性）
            lfsr = {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            b0   = lfsr[15:8] ^ i[7:0];
            b1   = lfsr[7:0]  ^ {i[3:0], 4'h5};

            // 第 1 拍：串行吃 b0
            s_valid = 1'b1;
            s_data  = b0;
            p_data  = 16'h0000;
            @(posedge clk); #1;

            // 第 2 拍：串行吃 b1；同时并行路径吃整个字
            s_data  = b1;
            p_data  = {b0, b1};
            @(posedge clk); #1;

            // 并行路径把结果打回
            acc     = p_next;
            p_valid = p_next;

            if (s_crc !== acc) begin
                if (errs < 5)
                    $display("[CRC-PAR2] FAIL word %0d: ser=0x%04h par=0x%04h",
                             i, s_crc, acc);
                errs = errs + 1;
            end
        end

        s_valid = 1'b0;

        $display("------------------------------------------------");
        if (errs == 0)
            $display("[CRC-PAR2] PASS: 512 words (1024 bytes) match serial CRC");
        else
            $display("[CRC-PAR2] FAIL: %0d mismatches", errs);
        $display("------------------------------------------------");
        $finish;
    end

endmodule
