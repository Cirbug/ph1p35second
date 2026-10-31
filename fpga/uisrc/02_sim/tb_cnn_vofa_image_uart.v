`timescale 1ns / 1ps

module tb_cnn_vofa_image_uart;
    localparam integer CLK_HZ = 10000000;
    localparam integer BAUD = 1000000;
    localparam integer BIT_NS = CLK_HZ / BAUD * 100;
    localparam integer HEADER_BYTES = 23;
    localparam integer IMAGE_BYTES = 7200;
    localparam integer TOTAL_BYTES = HEADER_BYTES + IMAGE_BYTES;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg frame_done = 1'b0;
    reg mask_data = 1'b0;
    wire mask_read_enable;
    wire [15:0] mask_read_address;
    wire dump_active;
    wire uart_tx;
    reg [7:0] received [0:TOTAL_BYTES-1];
    reg [7:0] rx_byte;
    integer i;
    integer j;

    always #50 clk = ~clk;

    always @(posedge clk) begin
        if (mask_read_enable)
            mask_data <= mask_read_address[0];
    end

    cnn_vofa_image_uart #(
        .CLK_HZ(CLK_HZ), .BAUD(BAUD), .WIDTH(320), .HEIGHT(180)
    ) dut (
        .clk(clk), .rst_n(rst_n), .frame_done(frame_done),
        .mask_data(mask_data), .mask_read_enable(mask_read_enable),
        .mask_read_address(mask_read_address), .dump_active(dump_active),
        .uart_tx(uart_tx)
    );

    function [7:0] expected_header;
        input integer index;
        begin
            case (index)
                0: expected_header = "i";
                1: expected_header = "m";
                2: expected_header = "a";
                3: expected_header = "g";
                4: expected_header = "e";
                5: expected_header = ":";
                6: expected_header = "0";
                7: expected_header = ",";
                8: expected_header = "7";
                9: expected_header = "2";
                10: expected_header = "0";
                11: expected_header = "0";
                12: expected_header = ",";
                13: expected_header = "3";
                14: expected_header = "2";
                15: expected_header = "0";
                16: expected_header = ",";
                17: expected_header = "1";
                18: expected_header = "8";
                19: expected_header = "0";
                20: expected_header = ",";
                21: expected_header = "1";
                default: expected_header = 8'h0a;
            endcase
        end
    endfunction

    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);
        frame_done = 1'b1;
        @(negedge clk);
        frame_done = 1'b0;

        for (i = 0; i < TOTAL_BYTES; i = i + 1) begin
            @(negedge uart_tx);
            #(BIT_NS / 2);
            if (uart_tx !== 1'b0)
                $fatal(1, "false UART start bit at byte %0d", i);
            for (j = 0; j < 8; j = j + 1) begin
                #BIT_NS;
                rx_byte[j] = uart_tx;
            end
            #BIT_NS;
            if (uart_tx !== 1'b1)
                $fatal(1, "UART stop bit error at byte %0d", i);
            received[i] = rx_byte;
        end

        for (i = 0; i < HEADER_BYTES; i = i + 1)
            if (received[i] !== expected_header(i))
                $fatal(1, "header byte %0d got %02x expected %02x",
                       i, received[i], expected_header(i));
        for (i = HEADER_BYTES; i < TOTAL_BYTES; i = i + 1)
            if (received[i] !== 8'haa)
                $fatal(1, "Mono payload byte %0d got %02x expected AA",
                       i - HEADER_BYTES, received[i]);

        repeat (20) @(posedge clk);
        if (dump_active !== 1'b0 || uart_tx !== 1'b1)
            $fatal(1, "dump did not return idle");
        $display("PASS VOFA FireWater image: header=23 bytes payload=7200 bytes Mono MSB-first");
        $finish;
    end

    initial begin
        #100000000;
        $fatal(1, "timeout");
    end
endmodule
