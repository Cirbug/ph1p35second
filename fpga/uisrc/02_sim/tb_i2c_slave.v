//==========================================================================
// tb_i2c_slave.v —— 只测 I2C 从机，不涉及桥接
//
//   主机器发：START | 0x6C | REG_H | REG_L | DATA | STOP
//   检查从机 o_reg / o_data
//==========================================================================
`timescale 1ns/1ps

module tb_i2c_slave;

    reg clk = 1'b0, rstn = 1'b0;
    always #20 clk = ~clk;

    reg  scl;
    wire sda;
    reg  sda_oe, sda_val;
    pullup (sda);
    assign sda = sda_oe ? sda_val : 1'bz;

    wire [15:0] o_reg;
    wire [7:0]  o_data;
    wire        o_valid;
    wire        o_seen;
    wire        dbg_start, dbg_stop;

    i2c_slave #( .DEV_ID ( 8'h6C ) ) dut (
        .clk     ( clk     ),
        .rstn    ( rstn    ),
        .scl_i   ( scl     ),
        .sda_io  ( sda     ),
        .o_reg   ( o_reg   ),
        .o_data  ( o_data  ),
        .o_valid ( o_valid ),
        .o_seen  ( o_seen  ),
        .dbg_start( dbg_start ),
        .dbg_stop ( dbg_stop  )
    );

    localparam integer HALF = 20;

    task scl_lo; begin scl = 1'b0; repeat (HALF) @(negedge clk); end endtask
    task scl_hi; begin scl = 1'b1; repeat (HALF) @(negedge clk); end endtask
    task sda_set(input v); begin sda_oe = 1'b1; sda_val = v; repeat (HALF) @(negedge clk); end endtask
    task sda_rel; begin sda_oe = 1'b0; repeat (HALF) @(negedge clk); end endtask

    reg ack_seen;

    task i2c_start;
        begin
            scl_lo; sda_rel; scl_hi;
            repeat (HALF) @(negedge clk);
            sda_oe = 1'b1; sda_val = 1'b0;      // START
            repeat (HALF) @(negedge clk);
            scl_lo;
        end
    endtask

    task i2c_stop;
        begin
            scl_lo; sda_set(1'b0); scl_hi;
            sda_oe = 1'b0;                       // STOP
            repeat (HALF*2) @(negedge clk);
        end
    endtask

    task send_byte(input [7:0] b);
        integer k;
        begin
            for (k = 7; k >= 0; k = k - 1) begin
                scl_lo;
                sda_set(b[k]);
                scl_hi;
                scl_lo;
            end
            scl_lo;
            sda_rel;
            scl_hi;
            ack_seen = (sda === 1'b0);
            $display("    ACK=%b", ack_seen);
            scl_lo;
        end
    endtask

    reg [15:0] exp_reg;
    reg [7:0]  exp_data;
    integer    errs = 0;
    reg [15:0] got_reg;
    reg [7:0]  got_data;
    reg        got_any = 0;

    always @(posedge clk) begin
        if (o_valid) begin
            got_reg  = o_reg;
            got_data = o_data;
            got_any  = 1'b1;
            $display("  [从机] REG=%04h DATA=%02h", o_reg, o_data);
        end
    end

    initial begin
        scl = 1'b1; sda_oe = 1'b0; sda_val = 1'b1;
        rstn = 1'b0;
        repeat (20) @(negedge clk);
        rstn = 1'b1;
        repeat (50) @(negedge clk);

        $display("==========================================");
        $display("单独测 I2C 从机：写 320e = 05");
        $display("==========================================");
        exp_reg = 16'h320e; exp_data = 8'h05;

        i2c_start;
        send_byte(8'h6C);
        send_byte(8'h32);
        send_byte(8'h0E);
        send_byte(8'h05);
        i2c_stop;

        repeat (200) @(negedge clk);

        $display("------------------------------------------");
        if (!got_any) begin
            $display("FAIL: 从机没有输出 o_valid");
            errs = errs + 1;
        end else if (got_reg !== exp_reg || got_data !== exp_data) begin
            $display("FAIL: 收到 REG=%04h DATA=%02h，期望 %04h / %02h",
                     got_reg, got_data, exp_reg, exp_data);
            errs = errs + 1;
        end else begin
            $display("PASS: REG=%04h DATA=%02h 正确", got_reg, got_data);
        end
        $display("==========================================");
        if (errs == 0) $display("RESULT: PASS");
        else           $display("RESULT: FAIL (%0d)", errs);
        $finish;
    end
    initial begin
        #3000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
