//==========================================================================
// tb_csi2_ptr_fifo.v
//   异步 FIFO 回环：写 3000 个字（超过深度 1024，必然触发 full 反压），
//   读侧用不同频率的时钟，校验数据顺序与数量完全正确。
//==========================================================================
`timescale 1ns / 1ps

module tb_csi2_ptr_fifo;

    localparam DW = 18;
    localparam AW = 10;
    localparam N  = 3000;

    // 两个不同频率的时钟，制造真实跨时钟域
    reg wclk = 1'b0;
    reg rclk = 1'b0;
    always #5.0  wclk = ~wclk;      // 100 MHz
    always #5.15 rclk = ~rclk;      // ~97 MHz

    reg           wrstn = 1'b0;
    reg           rrstn = 1'b0;
    reg           we    = 1'b0;
    reg  [DW-1:0] wdata = 0;
    wire          full;

    reg           re    = 1'b0;
    wire [DW-1:0] rdata;
    wire          empty;

    csi2_ptr_fifo #( .DW ( DW ), .AW ( AW ) ) dut (
        .I_wclk   ( wclk   ),
        .I_wrstn  ( wrstn  ),
        .I_we     ( we     ),
        .I_wdata  ( wdata  ),
        .O_wfull  ( full   ),
        .I_rclk   ( rclk   ),
        .I_rrstn  ( rrstn  ),
        .I_re     ( re     ),
        .O_rdata  ( rdata  ),
        .O_rempty ( empty  )
    );

    integer nw, nr;
    integer errs;
    reg [DW-1:0] cap;
    integer full_seen;

    //---------------- 写侧 ----------------
    initial begin
        nw = 0;
        repeat (3) @(posedge wclk);
        wrstn = 1'b1;

        while (nw < N) begin
            @(negedge wclk);
            if (!full) begin
                we    = 1'b1;
                wdata = nw[DW-1:0];
                nw    = nw + 1;
            end else begin
                we    = 1'b0;
                full_seen = full_seen + 1;
            end
        end
        @(negedge wclk);
        we = 1'b0;
    end

    //---------------- 读侧 ----------------
    initial begin
        nr = 0;
        errs = 0;
        full_seen = 0;
        repeat (5) @(posedge rclk);
        rrstn = 1'b1;

        while (nr < N) begin
            @(negedge rclk);
            if (!empty) begin
                cap = rdata;
                if (cap !== nr[DW-1:0]) begin
                    if (errs < 8)
                        $display("FAIL: 第 %0d 个字读回 %0d", nr, cap);
                    errs = errs + 1;
                end
                re = 1'b1;
                nr = nr + 1;
            end else begin
                re = 1'b0;
            end
        end
        @(negedge rclk);
        re = 1'b0;

        repeat (20) @(posedge rclk);

        $display("--------------------------------------------------");
        $display("写入 %0d 字，读出 %0d 字，full 反压次数 %0d", nw, nr, full_seen);
        if (errs == 0)
            $display("PASS: 跨时钟域 3000 字顺序与数值全部正确");
        else
            $display("FAIL: %0d 个错误", errs);
        if (empty !== 1'b1) begin
            $display("FAIL: 读完后 FIFO 未空"); errs = errs + 1;
        end
        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
