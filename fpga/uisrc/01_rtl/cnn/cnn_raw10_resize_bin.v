`timescale 1ns / 1ps

// CSI-2 RAW10 payload (two bytes per input beat) to a 320x180 binary frame.
// The module is deliberately a tap: it never asserts back-pressure and does
// not depend on the CSI->DSI token FIFO.  One selected pixel is emitted at a
// time, while the input packer consumes every payload beat.
module cnn_raw10_resize_bin #(
    parameter [7:0] THRESHOLD = 8'd128,
    parameter integer SRC_WIDTH = 2592,
    parameter integer SRC_HEIGHT = 1944,
    parameter integer DST_WIDTH = 320,
    parameter integer DST_HEIGHT = 180
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        in_valid,
    input  wire [15:0] in_data,
    input  wire        in_line_end,
    input  wire        in_frame_end,
    output reg         frame_start,
    output reg         pixel_valid,
    output reg  [7:0]  pixel_data,
    output reg         frame_complete,
    output reg  [9:0]  pixel_x,
    output reg  [8:0]  pixel_y,
    output reg         format_error
);

    localparam integer H_INC = DST_WIDTH;
    localparam integer H_MOD = SRC_WIDTH;
    localparam integer V_INC = DST_HEIGHT;
    localparam integer V_MOD = SRC_HEIGHT;

    reg [2:0]  pack_count;
    reg [7:0]  b0, b1, b2, b3;
    reg [12:0] h_acc;
    reg [12:0] v_acc;
    reg [11:0] src_x;
    reg [10:0] src_y;
    reg [8:0]  dst_y;
    reg [8:0]  current_dst_y;
    reg        line_started;
    reg        keep_line;
    reg [9:0]  dst_x;
    reg        frame_started;
    reg        frame_end_pending;
    reg        out_pending;
    reg [7:0]  out_pending_data;
    reg [9:0]  out_pending_x;
    reg [8:0]  out_pending_y;

    // Blocking temporaries are used only while closing a 5-byte RAW10 group.
    // They allow all four pixels to update the horizontal phase in one clock.
    integer j;
    reg [12:0] acc_tmp;
    reg        found_tmp;
    reg [7:0]  pix_tmp;

    wire [7:0] in_hi = in_data[15:8];
    wire [7:0] in_lo = in_data[7:0];

    function [7:0] raw10_gray;
        input [7:0] msb;
        begin
            // RAW10 high eight bits are the same luminance ordering used by
            // the camera's ISP preview.  The two LSBs affect threshold only
            // at a boundary, so the compact high-byte path is intentional.
            raw10_gray = msb;
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pack_count       <= 3'd0;
            b0 <= 8'd0; b1 <= 8'd0; b2 <= 8'd0; b3 <= 8'd0;
            h_acc            <= 13'd0;
            v_acc            <= 13'd0;
            src_x            <= 12'd0;
            src_y            <= 11'd0;
            dst_y            <= 9'd0;
            current_dst_y    <= 9'd0;
            line_started     <= 1'b0;
            keep_line        <= 1'b0;
            dst_x            <= 10'd0;
            frame_started    <= 1'b0;
            frame_end_pending<= 1'b0;
            out_pending      <= 1'b0;
            out_pending_data <= 8'd0;
            out_pending_x    <= 10'd0;
            out_pending_y    <= 9'd0;
            frame_start      <= 1'b0;
            pixel_valid      <= 1'b0;
            pixel_data       <= 8'd0;
            frame_complete   <= 1'b0;
            pixel_x          <= 10'd0;
            pixel_y          <= 9'd0;
            format_error     <= 1'b0;
        end else begin
            frame_start    <= 1'b0;
            pixel_valid    <= 1'b0;
            frame_complete <= 1'b0;

            // Drain the one-entry output stage.  Group completion is at
            // least two input clocks apart for this downsampling ratio.
            if (out_pending) begin
                pixel_valid <= 1'b1;
                pixel_data  <= out_pending_data;
                pixel_x     <= out_pending_x;
                pixel_y     <= out_pending_y;
                if (!frame_started) begin
                    frame_start   <= 1'b1;
                    frame_started <= 1'b1;
                end
                out_pending <= 1'b0;
            end else if (frame_end_pending) begin
                frame_complete    <= 1'b1;
                frame_end_pending <= 1'b0;
                frame_started     <= 1'b0;
            end

            // A line starts with the first payload beat.  Updating the
            // vertical phase here selects exactly 180 of the 1944 rows.
            if (in_valid && !line_started) begin
                line_started <= 1'b1;
                src_x        <= 12'd0;
                h_acc        <= 13'd0;
                if ((v_acc + V_INC) >= V_MOD) begin
                    keep_line <= 1'b1;
                    v_acc     <= v_acc + V_INC - V_MOD;
                    current_dst_y <= dst_y;
                    dst_y <= dst_y + 9'd1;
                end else begin
                    keep_line <= 1'b0;
                    v_acc     <= v_acc + V_INC;
                end
            end

            // Input is always two bytes.  Since five-byte groups do not align
            // with the beat, the pack counter cycles 0,2,4,1,3,0.
            if (in_valid) begin
                case (pack_count)
                3'd0: begin b0 <= in_hi; b1 <= in_lo; pack_count <= 3'd2; end
                3'd1: begin b1 <= in_hi; b2 <= in_lo; pack_count <= 3'd3; end
                3'd2: begin b2 <= in_hi; b3 <= in_lo; pack_count <= 3'd4; end
                3'd3: begin
                    // Three bytes are buffered: in_hi is RAW10 byte 3 and
                    // in_lo is byte 4, completing the four-pixel group.
                    acc_tmp = h_acc;
                    found_tmp = 1'b0;
                    pix_tmp = 8'd0;
                    for (j = 0; j < 4; j = j + 1) begin
                        acc_tmp = acc_tmp + H_INC;
                        if (acc_tmp >= H_MOD) begin
                            acc_tmp = acc_tmp - H_MOD;
                            if (keep_line && !found_tmp) begin
                                found_tmp = 1'b1;
                                case (j)
                                0: pix_tmp = (raw10_gray(b0) < THRESHOLD) ? 8'h00 : 8'hFF;
                                1: pix_tmp = (raw10_gray(b1) < THRESHOLD) ? 8'h00 : 8'hFF;
                                2: pix_tmp = (raw10_gray(b2) < THRESHOLD) ? 8'h00 : 8'hFF;
                                default: pix_tmp = (raw10_gray(in_hi) < THRESHOLD) ? 8'h00 : 8'hFF;
                                endcase
                            end
                        end
                    end
                    h_acc <= acc_tmp;
                    if (found_tmp) begin
                        out_pending      <= 1'b1;
                        out_pending_data <= pix_tmp;
                        out_pending_x    <= dst_x;
                        out_pending_y    <= current_dst_y;
                        dst_x            <= dst_x + 10'd1;
                    end
                    src_x <= src_x + 12'd4;
                    pack_count <= 3'd0;
                end
                3'd4: begin
                    // Group = b0,b1,b2,b3,in_lo; in_hi starts the next one.
                    acc_tmp = h_acc;
                    found_tmp = 1'b0;
                    pix_tmp = 8'd0;
                    for (j = 0; j < 4; j = j + 1) begin
                        acc_tmp = acc_tmp + H_INC;
                        if (acc_tmp >= H_MOD) begin
                            acc_tmp = acc_tmp - H_MOD;
                            if (keep_line && !found_tmp) begin
                                found_tmp = 1'b1;
                                case (j)
                                0: pix_tmp = (raw10_gray(b0) < THRESHOLD) ? 8'h00 : 8'hFF;
                                1: pix_tmp = (raw10_gray(b1) < THRESHOLD) ? 8'h00 : 8'hFF;
                                2: pix_tmp = (raw10_gray(b2) < THRESHOLD) ? 8'h00 : 8'hFF;
                                default: pix_tmp = (raw10_gray(b3) < THRESHOLD) ? 8'h00 : 8'hFF;
                                endcase
                            end
                        end
                    end
                    h_acc <= acc_tmp;
                    if (found_tmp) begin
                        out_pending      <= 1'b1;
                        out_pending_data <= pix_tmp;
                        out_pending_x    <= dst_x;
                        out_pending_y    <= current_dst_y;
                        dst_x            <= dst_x + 10'd1;
                    end
                    src_x <= src_x + 12'd4;
                    b0 <= in_lo;
                    pack_count <= 3'd1;
                end
                default: begin
                    format_error <= 1'b1;
                    pack_count <= 3'd0;
                end
                endcase
            end

            if (in_line_end) begin
                if (pack_count != 3'd0)
                    format_error <= 1'b1;
                line_started <= 1'b0;
                src_x        <= 12'd0;
                dst_x        <= 10'd0;
                src_y        <= src_y + 11'd1;
                h_acc        <= 13'd0;
                if (src_y >= SRC_HEIGHT-1)
                    src_y <= 11'd0;
            end

            if (in_frame_end)
                begin
                    frame_end_pending <= 1'b1;
                    src_y <= 11'd0;
                    dst_y <= 9'd0;
                    v_acc <= 13'd0;
                end
        end
    end
endmodule

module cnn_binary_mipi_preview #(
    parameter integer SRC_WIDTH = 320,
    parameter integer SRC_HEIGHT = 180,
    parameter integer SCALE = 6
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        pixel_valid,
    input  wire [7:0]  pixel_data,
    input  wire [9:0]  pixel_x,
    input  wire [8:0]  pixel_y,
    input  wire        out_ready,
    output reg         out_valid,
    output reg  [17:0] out_token,
    output reg         input_overrun
);
    localparam integer OUT_WIDTH = SRC_WIDTH * SCALE;
    localparam integer OUT_LINE_WC = OUT_WIDTH * 10 / 8;
    localparam integer OUT_WORDS = OUT_LINE_WC / 2;
    localparam integer WORD_WIDTH = (OUT_WORDS <= 2) ? 1 : $clog2(OUT_WORDS);

    localparam [2:0] ST_CAPTURE = 3'd0;
    localparam [2:0] ST_FRAME_START = 3'd1;
    localparam [2:0] ST_DATA = 3'd2;
    localparam [2:0] ST_LINE_END = 3'd3;
    localparam [2:0] ST_FRAME_END = 3'd4;

    reg [SRC_WIDTH-1:0] line_buffer0;
    reg [SRC_WIDTH-1:0] line_buffer1;
    reg [2:0] state;
    reg capture_bank;
    reg output_bank;
    reg pending_valid;
    reg pending_bank;
    reg [8:0] pending_y;
    reg [8:0] source_y_latched;
    reg [2:0] repeat_index;
    reg [WORD_WIDTH-1:0] word_index;

    function [7:0] packed_raw10_byte;
        input integer byte_index;
        integer group_index;
        integer byte_in_group;
        integer expanded_x;
        integer source_x;
        integer k;
        begin
            group_index = byte_index / 5;
            byte_in_group = byte_index - group_index * 5;
            if (byte_in_group < 4) begin
                expanded_x = group_index * 4 + byte_in_group;
                source_x = expanded_x / SCALE;
                packed_raw10_byte = (output_bank ? line_buffer1[source_x] :
                                                   line_buffer0[source_x]) ? 8'hff : 8'h00;
            end else begin
                packed_raw10_byte = 8'h00;
                for (k = 0; k < 4; k = k + 1) begin
                    expanded_x = group_index * 4 + k;
                    source_x = expanded_x / SCALE;
                    if (output_bank ? line_buffer1[source_x] : line_buffer0[source_x]) begin
                        case (k)
                            0: packed_raw10_byte[1:0] = 2'b11;
                            1: packed_raw10_byte[3:2] = 2'b11;
                            2: packed_raw10_byte[5:4] = 2'b11;
                            3: packed_raw10_byte[7:6] = 2'b11;
                        endcase
                    end
                end
            end
        end
    endfunction

    wire [11:0] byte_index;
    reg [15:0] packed_word;
    assign byte_index = {word_index, 1'b0};

    always @(byte_index or output_bank or line_buffer0 or line_buffer1) begin
        packed_word = {
            packed_raw10_byte(byte_index), packed_raw10_byte(byte_index + 1'b1)
        };
    end

    always @* begin
        out_valid = (state != ST_CAPTURE);
        out_token = 18'd0;
        case (state)
            ST_FRAME_START: out_token = {2'b01, 16'd0};
            ST_DATA:       out_token = {2'b00, packed_word};
            ST_LINE_END:   out_token = {2'b10, OUT_LINE_WC[15:0]};
            ST_FRAME_END:  out_token = {2'b11, 16'd0};
            default:       out_token = 18'd0;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= ST_CAPTURE;
            capture_bank <= 1'b0;
            output_bank <= 1'b0;
            pending_valid <= 1'b0;
            pending_bank <= 1'b0;
            pending_y <= 9'd0;
            source_y_latched <= 9'd0;
            repeat_index <= 3'd0;
            word_index <= {WORD_WIDTH{1'b0}};
            input_overrun <= 1'b0;
        end else begin
            if (pixel_valid) begin
                if (capture_bank)
                    line_buffer1[pixel_x] <= pixel_data[7];
                else
                    line_buffer0[pixel_x] <= pixel_data[7];

                if (pixel_x == SRC_WIDTH - 1) begin
                    capture_bank <= ~capture_bank;
                    if (state == ST_CAPTURE && !pending_valid) begin
                        source_y_latched <= pixel_y;
                        output_bank <= capture_bank;
                        repeat_index <= 3'd0;
                        word_index <= {WORD_WIDTH{1'b0}};
                        state <= (pixel_y == 0) ? ST_FRAME_START : ST_DATA;
                    end else if (!pending_valid) begin
                        pending_valid <= 1'b1;
                        pending_bank <= capture_bank;
                        pending_y <= pixel_y;
                    end else begin
                        input_overrun <= 1'b1;
                    end
                end
            end

            if (state == ST_CAPTURE && pending_valid &&
                !(pixel_valid && (pixel_x == SRC_WIDTH - 1))) begin
                source_y_latched <= pending_y;
                output_bank <= pending_bank;
                pending_valid <= 1'b0;
                repeat_index <= 3'd0;
                word_index <= {WORD_WIDTH{1'b0}};
                state <= (pending_y == 0) ? ST_FRAME_START : ST_DATA;
            end

            if (out_valid && out_ready) begin
                case (state)
                    ST_FRAME_START: state <= ST_DATA;
                    ST_DATA: begin
                        if (word_index == OUT_WORDS - 1)
                            state <= ST_LINE_END;
                        else
                            word_index <= word_index + 1'b1;
                    end
                    ST_LINE_END: begin
                        if (repeat_index == SCALE - 1) begin
                            if (source_y_latched == SRC_HEIGHT - 1)
                                state <= ST_FRAME_END;
                            else
                                state <= ST_CAPTURE;
                        end else begin
                            repeat_index <= repeat_index + 1'b1;
                            word_index <= {WORD_WIDTH{1'b0}};
                            state <= ST_DATA;
                        end
                    end
                    ST_FRAME_END: state <= ST_CAPTURE;
                    default: state <= ST_CAPTURE;
                endcase
            end
        end
    end
endmodule
