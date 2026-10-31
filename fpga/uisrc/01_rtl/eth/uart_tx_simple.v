// 极简 UART 发送器（8N1），用于 B 板调试打印
`timescale 1ns / 1ps
module uart_tx_simple #(
    parameter integer CLK_HZ = 125_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst,        // 高有效
    input  wire [7:0] data,
    input  wire       send,       // 单拍脉冲
    output reg        tx,
    output wire       busy
);
    localparam integer DIV = CLK_HZ / BAUD;

    reg [15:0] div_cnt;
    reg [3:0]  bit_idx;
    reg [9:0]  shifter;          // {停止位, 8 数据, 起始位}
    reg        running;

    assign busy = running;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx <= 1'b1;
            running <= 1'b0;
            div_cnt <= 0;
            bit_idx <= 0;
            shifter <= 10'h3FF;
        end else if (!running) begin
            tx <= 1'b1;
            if (send) begin
                shifter <= {1'b1, data, 1'b0};
                running <= 1'b1;
                bit_idx <= 0;
                div_cnt <= 0;
            end
        end else begin
            if (div_cnt == DIV - 1) begin
                div_cnt <= 0;
                tx <= shifter[0];
                shifter <= {1'b1, shifter[9:1]};
                if (bit_idx == 4'd9) begin
                    running <= 1'b0;
                    tx <= 1'b1;
                end else begin
                    bit_idx <= bit_idx + 4'd1;
                end
            end else begin
                div_cnt <= div_cnt + 16'd1;
            end
        end
    end
endmodule
