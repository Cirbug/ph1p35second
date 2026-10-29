//==========================================================================
// tb_cam_cfg_i2c.v
//   验证 uicfgcs500 的 I2C 配置链路 —— 这是整个设计里唯一没被仿真覆盖的
//   子系统，一旦有问题表现为 J18 永不亮、且没有任何错误提示。
//
//   做法：用一个行为级 I2C 从机模型接在 SDA 上：
//     - 检测 START / STOP
//     - 每个字节 8 bit MSB first，第 9 拍拉低 SDA 应答 ACK
//     - 记录收到的字节，检查器件地址是否为 0x6C
//
//   期望：uicfgcs500 把 116 条寄存器全部写完后 O_cfg_done 拉高。
//         每条 4 字节（器件地址 + 寄存器地址高/低 + 数据）→ 共 464 字节。
//==========================================================================
`timescale 1ns / 1ps

module tb_cam_cfg_i2c;

    localparam CLK_DIV = 24000000 / 100000 - 1;   // 与顶层一致：24MHz -> 100kHz

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #10 clk = ~clk;          // 24MHz，周期 41.667ns（近似）

    wire scl;
    wire sda;

    // I2C 总线：上拉 + 从机可拉低
    pullup (sda);

    reg  sda_low = 1'b0;
    assign sda = sda_low ? 1'b0 : 1'bz;

    wire cfg_done;
    wire ae_done;

    uicfgcs500 #(
        .CLK_DIV ( CLK_DIV )
    ) dut (
        .I_clk        ( clk      ),
        .I_rst_n      ( rst_n    ),
        .I_ae_req     ( 1'b0     ),
        .I_ae         ( 16'd3000 ),
        .I_ag         ( 16'd60   ),
        .O_cam_scl    ( scl      ),
        .IO_cam_sda   ( sda      ),
        .O_cfg_done   ( cfg_done ),
        .O_ae_cfg_done( ae_done  )
    );

    //==================================================================
    // 行为级 I2C 从机
    //==================================================================
    reg [7:0]  sh;
    integer    bitcnt;
    integer    nbytes;
    reg [7:0]  bytes [0:1023];
    reg        in_xfer;
    reg        scl_d;

    integer    i;

    // SCL 延迟一拍，用于边沿方向判断
    always @(scl) scl_d <= scl;

    // START：SCL 高时 SDA 下降（排除从机自己拉低的时刻）
    always @(negedge sda) begin
        if (scl === 1'b1 && !sda_low) begin
            in_xfer <= 1'b1;
            bitcnt  <= 0;
        end
    end

    // STOP：SCL 高时 SDA 上升
    always @(posedge sda) begin
        if (scl === 1'b1 && !sda_low) in_xfer <= 1'b0;
    end

    // 上升沿采样数据位
    always @(posedge scl) begin
        if (in_xfer) begin
            if (bitcnt < 8) begin
                sh     <= { sh[6:0], sda };
                bitcnt <= bitcnt + 1;
            end else if (bitcnt == 8) begin
                bitcnt <= 9;              // 第 9 拍是 ACK
            end
        end
    end

    // 下降沿处理 ACK 与字节收尾
    always @(negedge scl) begin
        if (in_xfer) begin
            if (bitcnt == 8) begin
                sda_low <= 1'b1;          // 拉低 SDA 应答
            end else if (bitcnt == 9) begin
                if (nbytes < 1024) bytes[nbytes] = sh;
                nbytes  <= nbytes + 1;
                sda_low <= 1'b0;
                bitcnt  <= 0;
            end
        end else begin
            sda_low <= 1'b0;
        end
    end

    //==================================================================
    // 主流程
    //==================================================================
    integer errs;
    integer waited;
    integer reg_writes;

    initial begin
        errs = 0; nbytes = 0; bitcnt = 0; in_xfer = 0; scl_d = 1'b1;
        sh = 0; sda_low = 0; waited = 0;

        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        #1 rst_n = 1'b1;

        // 等待配置完成，最多约 80ms 仿真时间（24MHz 下约 200 万拍）
        while (!cfg_done && waited < 2000000) begin
            @(posedge clk);
            waited = waited + 1;
        end
        #2000;

        reg_writes = nbytes / 4;

        $display("--------------------------------------------------");
        $display("sim cycles = %0d   bytes on bus = %0d   reg writes = %0d",
                 waited, nbytes, reg_writes);
        $display("cfg_done = %0b   ae_done = %0b", cfg_done, ae_done);

        if (!cfg_done) begin
            $display("FAIL: cfg_done 未拉高（配置未完成）");
            errs = errs + 1;
        end

        if (nbytes < 8) begin
            $display("FAIL: 总线上只有 %0d 字节，I2C 没跑起来", nbytes);
            errs = errs + 1;
        end else begin
            // 每个 4 字节组的第 1 个字节应是器件地址 0x6C（写）
            for (i = 0; i < reg_writes; i = i + 1) begin
                if (bytes[4*i] !== 8'h6C) begin
                    if (errs < 5)
                        $display("FAIL: 第 %0d 组器件地址 = %02h，期望 6C",
                                 i, bytes[4*i]);
                    errs = errs + 1;
                end
                // 第 1 组应是软件复位 0x0103 <- 0x01（SC500 上电标准动作）
                if (i == 0) begin
                    if (bytes[1] !== 8'h01 || bytes[2] !== 8'h03 ||
                        bytes[3] !== 8'h01) begin
                        $display("FAIL: 首条寄存器 = %02h%02h <- %02h，期望 0103 <- 01",
                                 bytes[1], bytes[2], bytes[3]);
                        errs = errs + 1;
                    end else begin
                        $display("PASS: 首条 = 器件 6C, 寄存器 0103 <- 01（软件复位）");
                    end
                end
            end
            $display("前 3 组：%02h %02h %02h %02h | %02h %02h %02h %02h | %02h %02h %02h %02h",
                     bytes[0],bytes[1],bytes[2],bytes[3],
                     bytes[4],bytes[5],bytes[6],bytes[7],
                     bytes[8],bytes[9],bytes[10],bytes[11]);
        end

        $display("--------------------------------------------------");
        if (errs == 0) $display("RESULT: PASS  (I2C 配置链路正常)");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
