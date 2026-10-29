`timescale 1ns / 1ps

module cnn_result_uart #(
    parameter integer CLK_HZ = 24000000,
    parameter integer BAUD = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [9:0] x,
    input wire [8:0] y,
    input wire [9:0] w,
    input wire [8:0] h,
    input wire [3:0] class_id,
    output reg busy,
    output reg uart_tx
);
    localparam integer DIV = CLK_HZ / BAUD;
    localparam integer NBYTE = 21;
    localparam [1:0] ST_IDLE = 2'd0, ST_LOAD = 2'd1, ST_BITS = 2'd2, ST_STOP = 2'd3;
    reg [1:0] state;
    reg [5:0] index;
    reg [15:0] div_count;
    reg [3:0] bit_count;
    reg [9:0] shift;
    reg [9:0] x_latched, w_latched;
    reg [8:0] y_latched, h_latched;
    reg [3:0] class_latched;
    reg [7:0] msg_byte;

    function [7:0] dec_hundreds;
        input [9:0] value;
        begin dec_hundreds = 8'd48 + (value / 100); end
    endfunction
    function [7:0] dec_tens;
        input [9:0] value;
        begin dec_tens = 8'd48 + ((value / 10) % 10); end
    endfunction
    function [7:0] dec_ones;
        input [9:0] value;
        begin dec_ones = 8'd48 + (value % 10); end
    endfunction

    always @* begin
        case (index)
        0: msg_byte = "D"; 1: msg_byte = ",";
        2: msg_byte = dec_hundreds(x_latched); 3: msg_byte = dec_tens(x_latched); 4: msg_byte = dec_ones(x_latched);
        5: msg_byte = ",";
        6: msg_byte = dec_hundreds(y_latched); 7: msg_byte = dec_tens(y_latched); 8: msg_byte = dec_ones(y_latched);
        9: msg_byte = ",";
        10: msg_byte = dec_hundreds(w_latched); 11: msg_byte = dec_tens(w_latched); 12: msg_byte = dec_ones(w_latched);
        13: msg_byte = ",";
        14: msg_byte = dec_hundreds(h_latched); 15: msg_byte = dec_tens(h_latched); 16: msg_byte = dec_ones(h_latched);
        17: msg_byte = ",";
        18: msg_byte = 8'd48 + {4'd0, class_latched};
        19: msg_byte = 8'h0D; 20: msg_byte = 8'h0A;
        default: msg_byte = 8'h20;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; index <= 0; div_count <= 0; bit_count <= 0; shift <= 10'h3ff;
            busy <= 1'b0; uart_tx <= 1'b1;
            x_latched <= 0; y_latched <= 0; w_latched <= 0; h_latched <= 0; class_latched <= 0;
        end else begin
            case (state)
            ST_IDLE: begin
                uart_tx <= 1'b1;
                busy <= 1'b0;
                if (start) begin
                    x_latched <= x; y_latched <= y; w_latched <= w; h_latched <= h; class_latched <= class_id;
                    index <= 0; busy <= 1'b1; state <= ST_LOAD;
                end
            end
            ST_LOAD: begin
                shift <= {1'b1, msg_byte, 1'b0};
                uart_tx <= 1'b0;
                bit_count <= 0; div_count <= 0; state <= ST_BITS;
            end
            ST_BITS: begin
                if (div_count >= DIV-1) begin
                    div_count <= 0;
                    if (bit_count == 8) begin
                        uart_tx <= 1'b1; state <= ST_STOP;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                        shift <= {1'b1, shift[9:1]};
                        uart_tx <= shift[1];
                    end
                end else begin
                    div_count <= div_count + 1'b1;
                end
            end
            ST_STOP: begin
                if (div_count >= DIV-1) begin
                    div_count <= 0;
                    if (index == NBYTE-1) begin
                        busy <= 1'b0; uart_tx <= 1'b1; state <= ST_IDLE;
                    end else begin
                        index <= index + 1'b1; state <= ST_LOAD;
                    end
                end else begin
                    div_count <= div_count + 1'b1;
                end
            end
            default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
