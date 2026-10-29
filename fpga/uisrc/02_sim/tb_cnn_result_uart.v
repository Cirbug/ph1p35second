`timescale 1ns / 1ps
module tb_cnn_result_uart;
    reg clk = 1'b0, rst_n = 1'b0, start = 1'b0;
    wire busy, uart_tx;
    integer i, k;
    reg [7:0] byte_value;
    reg [8*21-1:0] line;
    always #5 clk = ~clk;

    cnn_result_uart #(.CLK_HZ(40000000), .BAUD(10000000)) dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .x(10'd24), .y(9'd45), .w(10'd41), .h(9'd48), .class_id(4'd3),
        .busy(busy), .uart_tx(uart_tx)
    );

    task receive_byte;
        output [7:0] value;
        begin
            @(negedge uart_tx);
            #(40 + 20);
            for (k = 0; k < 8; k = k + 1) begin
                value[k] = uart_tx;
                #40;
            end
        end
    endtask

    initial begin
        fork
            begin #10000; $display("UART timeout"); $fatal(1); end
        join_none
        line = 0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        for (i = 0; i < 21; i = i + 1) begin
            receive_byte(byte_value);
            line[(20-i)*8 +: 8] = byte_value;
        end
        if (line !== {8'h44,8'h2c,8'h30,8'h32,8'h34,8'h2c,8'h30,8'h34,8'h35,
                      8'h2c,8'h30,8'h34,8'h31,8'h2c,8'h30,8'h34,8'h38,
                      8'h2c,8'h33,8'h0d,8'h0a}) begin
            $display("FAIL UART result: %s", line);
            $fatal(1);
        end
        $display("PASS UART result: %s", line);
        $finish;
    end
endmodule
