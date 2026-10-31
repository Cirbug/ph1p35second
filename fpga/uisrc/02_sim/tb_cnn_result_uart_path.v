`timescale 1ns / 1ps

module tb_cnn_result_uart_path;
    reg wr_clk = 1'b0;
    reg clk = 1'b0;
    reg wr_rst_n = 1'b0;
    reg rst_n = 1'b0;
    reg wr_en = 1'b0;
    reg frame_done = 1'b0;
    reg [4:0] frame_digit_count = 0;
    reg [15:0] frame_ink_pixels = 0;
    reg [8:0] frame_active_rows = 0;
    reg [3:0] frame_band_count = 0;
    reg [11:0] frame_column_segments = 0;
    reg [9:0] x = 0, w = 0;
    reg [8:0] y = 0, h = 0;
    reg [3:0] class_id = 0;
    wire uart_tx, overflow;

    integer i, k;
    reg [7:0] byte_value;
    reg [8*24-1:0] line;

    always #5 wr_clk = ~wr_clk;
    always #10 clk = ~clk;

    cnn_result_uart_path #(
        .CLK_HZ(50000000), .BAUD(5000000), .FIFO_ADDR_WIDTH(3)
    ) dut (
        .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .wr_en(wr_en),
        .frame_done(frame_done), .frame_digit_count(frame_digit_count),
        .frame_ink_pixels(frame_ink_pixels), .frame_active_rows(frame_active_rows),
        .frame_band_count(frame_band_count), .frame_column_segments(frame_column_segments),
        .x(x), .y(y), .w(w), .h(h), .class_id(class_id),
        .clk(clk), .rst_n(rst_n), .uart_tx(uart_tx), .overflow(overflow)
    );

    task push_result;
        input [9:0] px;
        input [8:0] py;
        input [9:0] pw;
        input [8:0] ph;
        input [3:0] digit;
        begin
            @(negedge wr_clk);
            x = px; y = py; w = pw; h = ph; class_id = digit; wr_en = 1'b1;
            @(negedge wr_clk);
            wr_en = 1'b0;
        end
    endtask

    task push_frame_summary;
        input [4:0] count;
        begin
            @(negedge wr_clk);
            frame_digit_count = count;
            frame_ink_pixels = (count == 0) ? 16'h04b2 : 16'h0da4;
            frame_active_rows = (count == 0) ? 9'h0b4 : 9'h096;
            frame_band_count = (count == 0) ? 4'h2 : 4'h3;
            frame_column_segments = (count == 0) ? 12'h0a0 : 12'h12c;
            frame_done = 1'b1;
            @(negedge wr_clk);
            frame_done = 1'b0;
            repeat (2) @(negedge wr_clk);
        end
    endtask

    task receive_record;
        input integer byte_count;
        output [8*24-1:0] received;
        begin
            received = 0;
            for (i = 0; i < byte_count; i = i + 1) begin
                @(negedge uart_tx);
                #300;
                for (k = 0; k < 8; k = k + 1) begin
                    byte_value[k] = uart_tx;
                    #200;
                end
                received[(byte_count-1-i)*8 +: 8] = byte_value;
            end
        end
    endtask

    initial begin
        fork
            begin #1000000; $display("UART path timeout"); $fatal(1); end
        join_none

        repeat (5) @(negedge wr_clk);
        wr_rst_n = 1'b1;
        rst_n = 1'b1;

        fork
            begin
                push_frame_summary(5'd0);
                push_result(10'd24, 9'd45, 10'd41, 9'd48, 4'd3);
                push_result(10'd5, 9'd105, 10'd52, 9'd74, 4'd0);
                push_result(10'd319, 9'd179, 10'd1, 9'd180, 4'd9);
                push_frame_summary(5'd3);
            end
            begin
                receive_record(15, line);
                if (line[119:0] !== {8'h46,8'h52,8'h41,8'h4d,8'h45,8'h5f,8'h44,8'h4f,
                                      8'h4e,8'h45,8'h2c,8'h30,8'h30,8'h0d,8'h0a}) begin
                    $display("FAIL zero-detection frame summary: %0s", line[119:0]);
                    $fatal(1);
                end
                receive_record(24, line);
                if (line !== {8'h53,8'h54,8'h41,8'h54,8'h2c,
                              8'h30,8'h34,8'h42,8'h32,8'h2c,
                              8'h30,8'h42,8'h34,8'h2c,8'h32,8'h2c,
                              8'h30,8'h41,8'h30,8'h2c,8'h30,8'h30,8'h0d,8'h0a}) begin
                    $display("FAIL zero-detection frame stats: %0s", line);
                    $fatal(1);
                end
                receive_record(21, line);
                if (line[167:0] !== {8'h44,8'h2c,8'h30,8'h32,8'h34,8'h2c,8'h30,8'h34,8'h35,
                                     8'h2c,8'h30,8'h34,8'h31,8'h2c,8'h30,8'h34,8'h38,
                                     8'h2c,8'h33,8'h0d,8'h0a}) begin
                    $display("FAIL result 1: %0s", line);
                    $fatal(1);
                end
                receive_record(21, line);
                if (line[167:0] !== {8'h44,8'h2c,8'h30,8'h30,8'h35,8'h2c,8'h31,8'h30,8'h35,
                                     8'h2c,8'h30,8'h35,8'h32,8'h2c,8'h30,8'h37,8'h34,
                                     8'h2c,8'h30,8'h0d,8'h0a}) begin
                    $display("FAIL result 2: %0s", line);
                    $fatal(1);
                end
                receive_record(21, line);
                if (line[167:0] !== {8'h44,8'h2c,8'h33,8'h31,8'h39,8'h2c,8'h31,8'h37,8'h39,
                                     8'h2c,8'h30,8'h30,8'h31,8'h2c,8'h31,8'h38,8'h30,
                                     8'h2c,8'h39,8'h0d,8'h0a}) begin
                    $display("FAIL result 3: %0s", line);
                    $fatal(1);
                end
                receive_record(15, line);
                if (line[119:0] !== {8'h46,8'h52,8'h41,8'h4d,8'h45,8'h5f,8'h44,8'h4f,
                                      8'h4e,8'h45,8'h2c,8'h30,8'h33,8'h0d,8'h0a}) begin
                    $display("FAIL detected-frame summary: %0s", line[119:0]);
                    $fatal(1);
                end
                receive_record(24, line);
                if (line !== {8'h53,8'h54,8'h41,8'h54,8'h2c,
                              8'h30,8'h44,8'h41,8'h34,8'h2c,
                              8'h30,8'h39,8'h36,8'h2c,8'h33,8'h2c,
                              8'h31,8'h32,8'h43,8'h2c,8'h30,8'h33,8'h0d,8'h0a}) begin
                    $display("FAIL detected-frame stats: %0s", line);
                    $fatal(1);
                end
            end
        join

        repeat (8) @(negedge clk);
        if (overflow || uart_tx !== 1'b1) begin
            $display("FAIL: result queue overflowed or UART did not return idle");
            $fatal(1);
        end
        $display("PASS CNN detections and frame counts only on UART");
        $finish;
    end
endmodule
