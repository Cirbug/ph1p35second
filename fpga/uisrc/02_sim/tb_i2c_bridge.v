//==========================================================================
// tb_i2c_bridge.v
//   摄像头 I2C 桥的端到端测试。
//
//   ┌────────────┐  I2C   ┌───────────────┐  I2C   ┌──────────────┐
//   │ A板主机器  │───────>│  cam_i2c_bridge│───────>│ 摄像头模型   │
//   │ (本TB任务) │        │   (DUT)        │        │ (i2c_slave)  │
//   └────────────┘        └───────────────┘        └──────────────┘
//
//   时序纪律（非常重要，不遵守就会误判 START/STOP）：
//     · 数据位改变 SDA 时，SCL 必须已经在低电平并保持
//     · 只有 START / STOP 才允许在 SCL 高电平期间改变 SDA
//     · SDA 与 SCL 绝不能在同一时刻变化（同步器会造出假边沿）
//==========================================================================
`timescale 1ns/1ps

module tb_i2c_bridge;

    reg clk = 1'b0;
    reg rstn = 1'b0;
    always #20 clk = ~clk;              // 约 25MHz

    //==================================================================
    // DUT 接线
    //==================================================================
    reg  dsi_scl;
    wire dsi_sda;
    reg  dsi_sda_oe, dsi_sda_val;

    wire cam_scl;
    wire cam_sda;

    pullup (dsi_sda);
    pullup (cam_sda);

    assign dsi_sda = dsi_sda_oe ? dsi_sda_val : 1'bz;

    wire [15:0] ok_cnt, err_cnt, vts_drop_cnt;
    wire [8:0]  fifo_cnt;
    wire        busy;

    cam_i2c_bridge #(
        .DEV_ID     ( 8'h6C ),
        .CLK_DIV    ( 16'd40 ),
        .FIFO_AW    ( 8 ),
        .FILTER_VTS ( 1 ),
        .START_DELAY_MS ( 0 )   // 仿真不等上电延时
    ) dut (
        .I_clk      ( clk          ),
        .I_rstn     ( rstn         ),
        .I_dsi_scl  ( dsi_scl      ),
        .IO_dsi_sda ( dsi_sda      ),
        .O_cam_scl  ( cam_scl      ),
        .IO_cam_sda ( cam_sda      ),
        .O_ok_cnt   ( ok_cnt       ),
        .O_err_cnt  ( err_cnt      ),
        .O_vts_drop_cnt ( vts_drop_cnt ),
        .O_sn_301f  (          ),
        .O_first_reg (          ),
        .O_first_data(          ),
        .O_sn_320c  (          ),
        .O_sn_320d  (          ),
        .O_fifo_cnt ( fifo_cnt     ),
        .O_busy     ( busy         )
    );

    //==================================================================
    // 摄像头模型
    //==================================================================
    wire [15:0] cam_reg;
    wire [7:0]  cam_data;
    wire        cam_valid;
    wire        cam_seen;

    i2c_slave #( .DEV_ID ( 8'h6C ) ) u_cam (
        .clk     ( clk      ),
        .rstn    ( rstn     ),
        .scl_i   ( cam_scl  ),
        .sda_io  ( cam_sda  ),
        .o_reg   ( cam_reg  ),
        .o_data  ( cam_data ),
        .o_valid ( cam_valid),
        .o_seen  ( cam_seen )
    );

    //==================================================================
    // A 板 I2C 主机模型
    //==================================================================
    localparam integer HALF = 16;

    task scl_lo; begin dsi_scl = 1'b0; repeat (HALF) @(negedge clk); end endtask
    task scl_hi; begin dsi_scl = 1'b1; repeat (HALF) @(negedge clk); end endtask

    task sda_set(input v);
        begin
            dsi_sda_oe  = 1'b1;
            dsi_sda_val = v;
            repeat (HALF) @(negedge clk);
        end
    endtask

    task sda_rel;
        begin
            dsi_sda_oe = 1'b0;
            repeat (HALF) @(negedge clk);
        end
    endtask

    reg ack_seen;

    task i2c_start;
        begin
            scl_lo;
            sda_rel;
            scl_hi;
            repeat (HALF) @(negedge clk);
            dsi_sda_oe = 1'b1; dsi_sda_val = 1'b0;   // START：SCL 高，SDA 下降
            repeat (HALF) @(negedge clk);
            scl_lo;
        end
    endtask

    task i2c_stop;
        begin
            scl_lo;
            sda_set(1'b0);
            scl_hi;                                   // STOP：SCL 高，SDA 上升
            dsi_sda_oe = 1'b0;
            repeat (HALF*2) @(negedge clk);
        end
    endtask

    task i2c_send_byte(input [7:0] b);
        integer k;
        begin
            for (k = 7; k >= 0; k = k - 1) begin
                scl_lo;
                sda_set(b[k]);
                scl_hi;
                scl_lo;
            end
            scl_lo;
            sda_rel;          // ACK 位前释放 SDA
            scl_hi;
            ack_seen = (dsi_sda === 1'b0);
            scl_lo;
        end
    endtask

    task i2c_write(input [15:0] rg, input [7:0] dt);
        begin
            i2c_start;
            i2c_send_byte(8'h6C);
            if (!ack_seen) $display("  WARN: 地址未 ACK");
            i2c_send_byte(rg[15:8]);
            i2c_send_byte(rg[7:0]);
            i2c_send_byte(dt);
            i2c_stop;
        end
    endtask

    //==================================================================
    // 检查
    //==================================================================
    reg [15:0] exp_reg  [0:15];
    reg [7:0]  exp_data [0:15];
    reg [15:0] got_reg  [0:15];
    reg [7:0]  got_data [0:15];
    integer    n_got, n_exp, i, errs;

    always @(posedge clk) begin
        if (cam_valid && n_got < 16) begin
            got_reg [n_got] = cam_reg;
            got_data[n_got] = cam_data;
            $display("  [摄像头模型] REG=%04h DATA=%02h   (第 %0d 条)", cam_reg, cam_data, n_got+1);
            n_got = n_got + 1;
        end
    end

    //==================================================================
    // 主流程
    //==================================================================
    initial begin
        errs = 0; n_got = 0; n_exp = 0;
        dsi_scl = 1'b1; dsi_sda_oe = 1'b0; dsi_sda_val = 1'b1;

        rstn = 1'b0;
        repeat (20) @(posedge clk);
        rstn = 1'b1;
        repeat (300) @(posedge clk);

        $display("==================================================");
        $display("测试：A 板发 4 条寄存器写（含 2 条 VTS），验证转发 + VTS 过滤");
        $display("==================================================");

        // 第 1、2 条是 VTS，应该被过滤掉（FILTER_VTS=1）
        i2c_write(16'h320e, 8'h05);
        i2c_write(16'h320f, 8'h35);

        // 第 3、4 条是曝光，应该正常转发
        exp_reg[0] = 16'h3e01; exp_data[0] = 8'hA6; n_exp = 1;
        i2c_write(16'h3e01, 8'hA6);

        exp_reg[1] = 16'h3e02; exp_data[1] = 8'h00; n_exp = 2;
        i2c_write(16'h3e02, 8'h00);

        repeat (20000) @(posedge clk);

        $display("--------------------------------------------------");
        $display("A 板发出 %0d 条，摄像头模型收到 %0d 条", n_exp, n_got);
        if (n_got != n_exp) begin
            $display("FAIL: 条数不符");
            errs = errs + 1;
        end

        for (i = 0; i < n_exp; i = i + 1) begin
            if (i < n_got) begin
                if (got_reg[i] !== exp_reg[i] || got_data[i] !== exp_data[i]) begin
                    $display("FAIL: 第 %0d 条 = REG %04h DATA %02h，期望 %04h / %02h",
                             i+1, got_reg[i], got_data[i], exp_reg[i], exp_data[i]);
                    errs = errs + 1;
                end
            end
        end

        if (errs == 0) $display("PASS: 全部 %0d 条寄存器写正确转发", n_exp);

        // VTS 过滤检查：发了 2 条 VTS，应该被丢 2 条
        if (vts_drop_cnt != 16'd2) begin
            $display("FAIL: VTS 过滤计数 = %0d，期望 2", vts_drop_cnt);
            errs = errs + 1;
        end else begin
            $display("PASS: VTS 过滤正确（丢弃 %0d 条）", vts_drop_cnt);
        end

        $display("  转发成功 ok_cnt = %0d   失败 err_cnt = %0d   过滤 %0d",
                 ok_cnt, err_cnt, vts_drop_cnt);
        $display("==================================================");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d items)", errs);
        $finish;
    end    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
