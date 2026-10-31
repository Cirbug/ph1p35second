`timescale 1ns / 1ps

module tb_csi2_center_crop_raw10;
    localparam integer SRC_WIDTH = 24;
    localparam integer SRC_HEIGHT = 8;
    localparam integer OUT_WIDTH = 8;
    localparam integer OUT_HEIGHT = 4;
    localparam integer CROP_X = 8;
    localparam integer CROP_Y = 2;
    localparam integer SRC_BEATS = SRC_WIDTH * 5 / 8;
    localparam integer CROP_START_BEAT = CROP_X * 5 / 8;
    localparam integer OUT_BEATS = OUT_WIDTH * 5 / 8;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg in_valid = 1'b0;
    reg [15:0] in_data = 16'd0;
    reg in_line_end = 1'b0;
    reg [15:0] in_word_count = SRC_WIDTH * 10 / 8;
    reg [7:0] in_data_type = 8'h2B;
    reg in_frame_end = 1'b0;
    wire out_valid;
    wire [15:0] out_data;
    wire out_line_end;
    wire [15:0] out_word_count;
    wire format_error;
    integer emitted = 0;
    integer output_lines = 0;
    integer y, beat, expected_y, expected_beat;

    always #5 clk = ~clk;

    csi2_center_crop_raw10 #(
        .SRC_WIDTH(SRC_WIDTH), .SRC_HEIGHT(SRC_HEIGHT),
        .OUT_WIDTH(OUT_WIDTH), .OUT_HEIGHT(OUT_HEIGHT),
        .CROP_X(CROP_X), .CROP_Y(CROP_Y)
    ) dut (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_data(in_data),
        .in_line_end(in_line_end), .in_word_count(in_word_count),
        .in_data_type(in_data_type), .in_frame_end(in_frame_end),
        .out_valid(out_valid), .out_data(out_data),
        .out_line_end(out_line_end), .out_word_count(out_word_count),
        .format_error(format_error)
    );

    always @(posedge clk) begin
        if (!rst && out_valid) begin
            expected_y = CROP_Y + emitted / OUT_BEATS;
            expected_beat = CROP_START_BEAT + emitted % OUT_BEATS;
            if (out_data !== {expected_y[7:0], expected_beat[7:0]}) begin
                $display("FAIL crop beat #%0d got=%h expected row=%0d beat=%0d",
                         emitted, out_data, expected_y, expected_beat);
                $fatal(1);
            end
            emitted = emitted + 1;
        end
        if (!rst && out_line_end) begin
            if (out_word_count != OUT_WIDTH * 10 / 8) begin
                $display("FAIL output WC=%0d", out_word_count);
                $fatal(1);
            end
            output_lines = output_lines + 1;
        end
    end

    initial begin
        repeat (4) @(negedge clk);
        rst = 1'b0;

        for (y = 0; y < SRC_HEIGHT; y = y + 1) begin
            for (beat = 0; beat < SRC_BEATS; beat = beat + 1) begin
                @(negedge clk);
                in_valid = 1'b1;
                in_data = {y[7:0], beat[7:0]};
            end
            @(negedge clk);
            in_valid = 1'b0;
            in_line_end = 1'b1;
            @(negedge clk);
            in_line_end = 1'b0;
        end

        in_frame_end = 1'b1;
        @(negedge clk);
        in_frame_end = 1'b0;
        repeat (4) @(negedge clk);

        if (emitted != OUT_HEIGHT * OUT_BEATS || output_lines != OUT_HEIGHT || format_error) begin
            $display("FAIL beats=%0d lines=%0d format_error=%0d",
                     emitted, output_lines, format_error);
            $fatal(1);
        end
        $display("PASS center crop: %0d RAW10 beats, %0d lines, WC=%0d",
                 emitted, output_lines, out_word_count);
        $finish;
    end
endmodule
