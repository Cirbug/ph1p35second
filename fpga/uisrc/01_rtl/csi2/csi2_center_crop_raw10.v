`timescale 1ns / 1ps

// Word-aligned center crop for a parsed CSI-2 RAW10 payload.
// RAW10 pixels use four-pixel/five-byte groups, so the crop origin and output
// width must be multiples of eight to preserve two-byte beat alignment.
module csi2_center_crop_raw10 #(
    parameter integer SRC_WIDTH  = 2592,
    parameter integer SRC_HEIGHT = 1944,
    parameter integer OUT_WIDTH  = 1920,
    parameter integer OUT_HEIGHT = 1080,
    parameter integer CROP_X     = 336,
    parameter integer CROP_Y     = 432
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        in_valid,
    input  wire [15:0] in_data,
    input  wire        in_line_end,
    input  wire [15:0] in_word_count,
    input  wire [7:0]  in_data_type,
    input  wire        in_frame_end,
    output reg         out_valid,
    output reg  [15:0] out_data,
    output reg         out_line_end,
    output wire [15:0] out_word_count,
    output reg         format_error
);
    localparam integer SRC_LINE_BYTES = SRC_WIDTH * 10 / 8;
    localparam integer SRC_LINE_BEATS = SRC_LINE_BYTES / 2;
    localparam integer CROP_START_BEAT = CROP_X * 5 / 8;
    localparam integer OUT_LINE_BEATS = OUT_WIDTH * 5 / 8;
    localparam integer OUT_LINE_BYTES = OUT_WIDTH * 10 / 8;

    reg [15:0] src_beat_index;
    reg [15:0] src_line_index;
    wire raw10_line = (in_data_type[5:0] == 6'h2B);
    wire keep_line = (src_line_index >= CROP_Y) &&
                     (src_line_index < CROP_Y + OUT_HEIGHT);
    wire keep_beat = (src_beat_index >= CROP_START_BEAT) &&
                     (src_beat_index < CROP_START_BEAT + OUT_LINE_BEATS);

    assign out_word_count = OUT_LINE_BYTES;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            src_beat_index <= 16'd0;
            src_line_index <= 16'd0;
            out_valid <= 1'b0;
            out_data <= 16'd0;
            out_line_end <= 1'b0;
            format_error <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            out_line_end <= 1'b0;

            if (in_valid && raw10_line) begin
                if (keep_line && keep_beat) begin
                    out_valid <= 1'b1;
                    out_data <= in_data;
                end
                src_beat_index <= src_beat_index + 16'd1;
            end

            if (in_line_end && raw10_line) begin
                if ((in_word_count != SRC_LINE_BYTES) ||
                    (src_beat_index != SRC_LINE_BEATS))
                    format_error <= 1'b1;

                if (keep_line)
                    out_line_end <= 1'b1;

                src_beat_index <= 16'd0;
                if (src_line_index >= SRC_HEIGHT - 1)
                    src_line_index <= 16'd0;
                else
                    src_line_index <= src_line_index + 16'd1;
            end

            if (in_frame_end) begin
                src_beat_index <= 16'd0;
                src_line_index <= 16'd0;
            end
        end
    end
endmodule
