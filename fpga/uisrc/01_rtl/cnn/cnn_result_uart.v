`timescale 1ns / 1ps

module cnn_result_uart #(
    parameter integer CLK_HZ = 24000000,
    parameter integer BAUD = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [1:0] record_type,
    input wire [4:0] frame_digit_count,
    input wire [15:0] frame_ink_pixels,
    input wire [8:0] frame_active_rows,
    input wire [3:0] frame_band_count,
    input wire [11:0] frame_column_segments,
    input wire [9:0] x,
    input wire [8:0] y,
    input wire [9:0] w,
    input wire [8:0] h,
    input wire [3:0] class_id,
    output reg busy,
    output reg uart_tx
);
    localparam integer DIV = CLK_HZ / BAUD;
    localparam [5:0] DIGIT_LAST_INDEX = 6'd20;
    localparam [5:0] FRAME_LAST_INDEX = 6'd14;
    localparam [5:0] STAT_LAST_INDEX = 6'd23;
    localparam [1:0] RECORD_DIGIT = 2'b00;
    localparam [1:0] RECORD_FRAME = 2'b01;
    localparam [1:0] RECORD_STAT = 2'b10;
    localparam [1:0] ST_IDLE = 2'd0, ST_LOAD = 2'd1, ST_BITS = 2'd2, ST_STOP = 2'd3;
    reg [1:0] state;
    reg [5:0] index;
    reg [5:0] last_index;
    reg [15:0] div_count;
    reg [3:0] bit_count;
    reg [9:0] shift;
    reg [9:0] x_latched, w_latched;
    reg [8:0] y_latched, h_latched;
    reg [3:0] class_latched;
    reg [1:0] record_type_latched;
    reg [4:0] frame_count_latched;
    reg [15:0] ink_pixels_latched;
    reg [8:0] active_rows_latched;
    reg [3:0] band_count_latched;
    reg [11:0] column_segments_latched;
    reg [7:0] msg_byte;

    function [7:0] hex_char;
        input [3:0] nibble;
        begin
            hex_char = (nibble < 4'd10) ? (8'd48 + nibble) : (8'd55 + nibble);
        end
    endfunction

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
        if (record_type_latched == RECORD_FRAME) begin
            case (index)
             0: msg_byte = "F"; 1: msg_byte = "R"; 2: msg_byte = "A";
             3: msg_byte = "M"; 4: msg_byte = "E"; 5: msg_byte = "_";
             6: msg_byte = "D"; 7: msg_byte = "O"; 8: msg_byte = "N";
             9: msg_byte = "E"; 10: msg_byte = ",";
            11: msg_byte = 8'd48 + (frame_count_latched / 10);
            12: msg_byte = 8'd48 + (frame_count_latched % 10);
            13: msg_byte = 8'h0D; 14: msg_byte = 8'h0A;
            default: msg_byte = 8'h20;
            endcase
        end else if (record_type_latched == RECORD_STAT) begin
            case (index)
             0: msg_byte = "S"; 1: msg_byte = "T"; 2: msg_byte = "A";
             3: msg_byte = "T"; 4: msg_byte = ",";
             5: msg_byte = hex_char(ink_pixels_latched[15:12]);
             6: msg_byte = hex_char(ink_pixels_latched[11:8]);
             7: msg_byte = hex_char(ink_pixels_latched[7:4]);
             8: msg_byte = hex_char(ink_pixels_latched[3:0]);
             9: msg_byte = ",";
            10: msg_byte = hex_char({3'b000, active_rows_latched[8]});
            11: msg_byte = hex_char(active_rows_latched[7:4]);
            12: msg_byte = hex_char(active_rows_latched[3:0]);
            13: msg_byte = ",";
            14: msg_byte = hex_char(band_count_latched);
            15: msg_byte = ",";
            16: msg_byte = hex_char(column_segments_latched[11:8]);
            17: msg_byte = hex_char(column_segments_latched[7:4]);
            18: msg_byte = hex_char(column_segments_latched[3:0]);
            19: msg_byte = ",";
            20: msg_byte = hex_char({3'b000, frame_count_latched[4]});
            21: msg_byte = hex_char(frame_count_latched[3:0]);
            22: msg_byte = 8'h0d; 23: msg_byte = 8'h0a;
            default: msg_byte = 8'h20;
            endcase
        end else begin
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
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE; index <= 0; last_index <= DIGIT_LAST_INDEX;
            div_count <= 0; bit_count <= 0; shift <= 10'h3ff;
            busy <= 1'b0; uart_tx <= 1'b1;
            x_latched <= 0; y_latched <= 0; w_latched <= 0; h_latched <= 0; class_latched <= 0;
            record_type_latched <= RECORD_DIGIT; frame_count_latched <= 5'd0;
            ink_pixels_latched <= 16'd0; active_rows_latched <= 9'd0;
            band_count_latched <= 4'd0; column_segments_latched <= 12'd0;
        end else begin
            case (state)
            ST_IDLE: begin
                uart_tx <= 1'b1;
                busy <= 1'b0;
                if (start) begin
                    record_type_latched <= record_type;
                    frame_count_latched <= frame_digit_count;
                    ink_pixels_latched <= frame_ink_pixels;
                    active_rows_latched <= frame_active_rows;
                    band_count_latched <= frame_band_count;
                    column_segments_latched <= frame_column_segments;
                    if (record_type == RECORD_FRAME)
                        last_index <= FRAME_LAST_INDEX;
                    else if (record_type == RECORD_STAT)
                        last_index <= STAT_LAST_INDEX;
                    else
                        last_index <= DIGIT_LAST_INDEX;
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
                    if (index == last_index) begin
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

module cnn_vofa_image_uart #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BAUD = 115200,
    parameter integer WIDTH = 320,
    parameter integer HEIGHT = 180
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_done,
    input  wire        mask_data,
    output wire        mask_read_enable,
    output wire [15:0] mask_read_address,
    output reg         dump_active,
    output wire        uart_tx
);
    localparam integer PIXELS = WIDTH * HEIGHT;
    localparam integer PAYLOAD_BYTES = PIXELS / 8;
    localparam [3:0] ST_WAIT_FRAME = 4'd0;
    localparam [3:0] ST_HEADER = 4'd1;
    localparam [3:0] ST_HEADER_DRAIN = 4'd2;
    localparam [3:0] ST_READ_REQ = 4'd3;
    localparam [3:0] ST_READ_WAIT = 4'd4;
    localparam [3:0] ST_PAYLOAD = 4'd5;
    localparam [3:0] ST_PAYLOAD_DRAIN = 4'd6;
    localparam [4:0] HEADER_BYTES = 5'd23;

    reg [3:0] state;
    reg [4:0] header_index;
    reg [12:0] payload_index;
    reg [15:0] pixel_address;
    reg [2:0] bit_index;
    reg [7:0] packed_byte;
    reg [7:0] payload_byte;
    wire tx_busy;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_line;

    assign uart_tx = tx_line;
    assign mask_read_enable = dump_active && (state == ST_READ_REQ);
    assign mask_read_address = pixel_address;

    function [7:0] header_byte;
        input [4:0] index;
        begin
            case (index)
                5'd0:  header_byte = "i";
                5'd1:  header_byte = "m";
                5'd2:  header_byte = "a";
                5'd3:  header_byte = "g";
                5'd4:  header_byte = "e";
                5'd5:  header_byte = ":";
                5'd6:  header_byte = "0";
                5'd7:  header_byte = ",";
                5'd8:  header_byte = "7";
                5'd9:  header_byte = "2";
                5'd10: header_byte = "0";
                5'd11: header_byte = "0";
                5'd12: header_byte = ",";
                5'd13: header_byte = "3";
                5'd14: header_byte = "2";
                5'd15: header_byte = "0";
                5'd16: header_byte = ",";
                5'd17: header_byte = "1";
                5'd18: header_byte = "8";
                5'd19: header_byte = "0";
                5'd20: header_byte = ",";
                5'd21: header_byte = "1";
                default: header_byte = 8'h0a;
            endcase
        end
    endfunction

    always @* begin
        tx_start = 1'b0;
        tx_data = 8'd0;
        if ((state == ST_HEADER) && !tx_busy) begin
            tx_start = 1'b1;
            tx_data = header_byte(header_index);
        end else if ((state == ST_PAYLOAD) && !tx_busy) begin
            tx_start = 1'b1;
            tx_data = payload_byte;
        end
    end

    cnn_vofa_uart_byte_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_byte_tx (
        .clk(clk), .rst_n(rst_n), .start(tx_start), .data(tx_data),
        .busy(tx_busy), .uart_tx(tx_line)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_WAIT_FRAME;
            header_index <= 5'd0;
            payload_index <= 13'd0;
            pixel_address <= 16'd0;
            bit_index <= 3'd0;
            packed_byte <= 8'd0;
            payload_byte <= 8'd0;
            dump_active <= 1'b0;
        end else begin
            case (state)
                ST_WAIT_FRAME: begin
                    if (frame_done) begin
                        dump_active <= 1'b1;
                        header_index <= 5'd0;
                        payload_index <= 13'd0;
                        pixel_address <= 16'd0;
                        bit_index <= 3'd0;
                        packed_byte <= 8'd0;
                        state <= ST_HEADER;
                    end
                end

                ST_HEADER: begin
                    if (tx_start) begin
                        if (header_index == HEADER_BYTES - 1'b1)
                            state <= ST_HEADER_DRAIN;
                        else
                            header_index <= header_index + 1'b1;
                    end
                end

                ST_HEADER_DRAIN: begin
                    if (!tx_busy)
                        state <= ST_READ_REQ;
                end

                ST_READ_REQ: state <= ST_READ_WAIT;

                ST_READ_WAIT: begin
                    packed_byte[7 - bit_index] <= ~mask_data;
                    if (pixel_address < PIXELS - 1)
                        pixel_address <= pixel_address + 1'b1;
                    if (bit_index == 3'd7) begin
                        payload_byte <= {packed_byte[7:1], ~mask_data};
                        state <= ST_PAYLOAD;
                    end else begin
                        bit_index <= bit_index + 1'b1;
                        state <= ST_READ_REQ;
                    end
                end

                ST_PAYLOAD: begin
                    if (tx_start)
                        state <= ST_PAYLOAD_DRAIN;
                end

                ST_PAYLOAD_DRAIN: begin
                    if (!tx_busy) begin
                        if (payload_index == PAYLOAD_BYTES - 1) begin
                            dump_active <= 1'b0;
                            state <= ST_WAIT_FRAME;
                        end else begin
                            payload_index <= payload_index + 1'b1;
                            bit_index <= 3'd0;
                            state <= ST_READ_REQ;
                        end
                    end
                end

                default: begin
                    dump_active <= 1'b0;
                    state <= ST_WAIT_FRAME;
                end
            endcase
        end
    end
endmodule

module cnn_vofa_uart_byte_tx #(
    parameter integer CLK_HZ = 100000000,
    parameter integer BAUD = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] data,
    output reg        busy,
    output reg        uart_tx
);
    localparam integer DIVISOR = (CLK_HZ + BAUD / 2) / BAUD;
    reg [15:0] baud_count;
    reg [3:0] bit_index;
    reg [7:0] shift_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_count <= 16'd0;
            bit_index <= 4'd0;
            shift_data <= 8'd0;
            busy <= 1'b0;
            uart_tx <= 1'b1;
        end else if (!busy) begin
            uart_tx <= 1'b1;
            if (start) begin
                shift_data <= data;
                bit_index <= 4'd0;
                baud_count <= DIVISOR - 1'b1;
                busy <= 1'b1;
                uart_tx <= 1'b0;
            end
        end else if (baud_count != 0) begin
            baud_count <= baud_count - 1'b1;
        end else begin
            baud_count <= DIVISOR - 1'b1;
            if (bit_index < 4'd8) begin
                uart_tx <= shift_data[bit_index];
                bit_index <= bit_index + 1'b1;
            end else if (bit_index == 4'd8) begin
                uart_tx <= 1'b1;
                bit_index <= 4'd9;
            end else begin
                uart_tx <= 1'b1;
                busy <= 1'b0;
            end
        end
    end
endmodule

module cnn_result_async_fifo #(
    parameter integer DATA_WIDTH = 42,
    parameter integer ADDR_WIDTH = 5
)(
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire                  full,
    output reg                   overflow,
    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  empty
);
    localparam integer PTR_WIDTH = ADDR_WIDTH + 1;
    localparam integer DEPTH = 1 << ADDR_WIDTH;

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [PTR_WIDTH-1:0] wr_bin, wr_gray;
    reg [PTR_WIDTH-1:0] rd_bin, rd_gray;
    reg [PTR_WIDTH-1:0] rd_gray_meta, rd_gray_sync;
    reg [PTR_WIDTH-1:0] wr_gray_meta, wr_gray_sync;

    wire wr_take = wr_en && !full;
    wire rd_take = rd_en && !empty;
    wire [PTR_WIDTH-1:0] wr_bin_next = wr_bin + wr_take;
    wire [PTR_WIDTH-1:0] rd_bin_next = rd_bin + rd_take;
    wire [PTR_WIDTH-1:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    wire [PTR_WIDTH-1:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

    assign full = wr_gray == {~rd_gray_sync[PTR_WIDTH-1:PTR_WIDTH-2],
                              rd_gray_sync[PTR_WIDTH-3:0]};
    assign empty = rd_gray == wr_gray_sync;
    assign rd_data = mem[rd_bin[ADDR_WIDTH-1:0]];

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin <= {PTR_WIDTH{1'b0}};
            wr_gray <= {PTR_WIDTH{1'b0}};
            rd_gray_meta <= {PTR_WIDTH{1'b0}};
            rd_gray_sync <= {PTR_WIDTH{1'b0}};
            overflow <= 1'b0;
        end else begin
            rd_gray_meta <= rd_gray;
            rd_gray_sync <= rd_gray_meta;
            if (wr_take) begin
                mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
                wr_bin <= wr_bin_next;
                wr_gray <= wr_gray_next;
            end
            if (wr_en && full)
                overflow <= 1'b1;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin <= {PTR_WIDTH{1'b0}};
            rd_gray <= {PTR_WIDTH{1'b0}};
            wr_gray_meta <= {PTR_WIDTH{1'b0}};
            wr_gray_sync <= {PTR_WIDTH{1'b0}};
        end else begin
            wr_gray_meta <= wr_gray;
            wr_gray_sync <= wr_gray_meta;
            if (rd_take) begin
                rd_bin <= rd_bin_next;
                rd_gray <= rd_gray_next;
            end
        end
    end
endmodule

module cnn_result_uart_path #(
    parameter integer CLK_HZ = 24000000,
    parameter integer BAUD = 115200,
    parameter integer FIFO_ADDR_WIDTH = 5
)(
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        wr_en,
    input  wire        frame_done,
    input  wire [4:0]  frame_digit_count,
    input  wire [15:0] frame_ink_pixels,
    input  wire [8:0]  frame_active_rows,
    input  wire [3:0]  frame_band_count,
    input  wire [11:0] frame_column_segments,
    input  wire [9:0]  x,
    input  wire [8:0]  y,
    input  wire [9:0]  w,
    input  wire [8:0]  h,
    input  wire [3:0]  class_id,
    input  wire        clk,
    input  wire        rst_n,
    output wire        uart_tx,
    output wire        overflow
);
    wire fifo_full, fifo_empty, fifo_overflow;
    wire [63:0] fifo_data;
    reg stat_pending;
    reg [15:0] stat_ink_latched;
    reg [8:0] stat_rows_latched;
    reg [3:0] stat_bands_latched;
    reg [11:0] stat_segments_latched;
    reg [4:0] stat_count_latched;
    wire fifo_wr_en = wr_en || frame_done || stat_pending;
    wire [63:0] frame_record = {2'b01, 57'd0, frame_digit_count};
    wire [63:0] stat_record = {2'b10, 16'd0, stat_ink_latched,
                               stat_rows_latched, stat_bands_latched,
                               stat_segments_latched, stat_count_latched};
    wire [63:0] digit_record = {2'b00, 20'd0, x, y, w, h, class_id};
    wire [63:0] fifo_wr_data = frame_done ? frame_record :
                               (wr_en ? digit_record : stat_record);
    wire result_busy;
    wire result_tx;
    wire result_start = !fifo_empty && !result_busy;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            stat_pending <= 1'b0;
            stat_ink_latched <= 16'd0;
            stat_rows_latched <= 9'd0;
            stat_bands_latched <= 4'd0;
            stat_segments_latched <= 12'd0;
            stat_count_latched <= 5'd0;
        end else if (frame_done) begin
            stat_pending <= 1'b1;
            stat_ink_latched <= frame_ink_pixels;
            stat_rows_latched <= frame_active_rows;
            stat_bands_latched <= frame_band_count;
            stat_segments_latched <= frame_column_segments;
            stat_count_latched <= frame_digit_count;
        end else if (stat_pending && !wr_en) begin
            stat_pending <= 1'b0;
        end
    end

    cnn_result_async_fifo #(
        .DATA_WIDTH(64), .ADDR_WIDTH(FIFO_ADDR_WIDTH)
    ) u_result_fifo (
        .wr_clk(wr_clk), .wr_rst_n(wr_rst_n), .wr_en(fifo_wr_en),
        .wr_data(fifo_wr_data), .full(fifo_full), .overflow(fifo_overflow),
        .rd_clk(clk), .rd_rst_n(rst_n), .rd_en(result_start),
        .rd_data(fifo_data), .empty(fifo_empty)
    );

    cnn_result_uart #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_result_uart (
        .clk(clk), .rst_n(rst_n), .start(result_start),
        .record_type(fifo_data[63:62]), .frame_digit_count(fifo_data[4:0]),
        .frame_ink_pixels(fifo_data[45:30]),
        .frame_active_rows(fifo_data[29:21]),
        .frame_band_count(fifo_data[20:17]),
        .frame_column_segments(fifo_data[16:5]),
        .x(fifo_data[41:32]), .y(fifo_data[31:23]),
        .w(fifo_data[22:13]), .h(fifo_data[12:4]), .class_id(fifo_data[3:0]),
        .busy(result_busy), .uart_tx(result_tx)
    );

    assign uart_tx = result_busy ? result_tx : 1'b1;

    reg overflow_meta, overflow_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            overflow_meta <= 1'b0;
            overflow_sync <= 1'b0;
        end else begin
            overflow_meta <= fifo_overflow;
            overflow_sync <= overflow_meta;
        end
    end
    assign overflow = overflow_sync;
endmodule
