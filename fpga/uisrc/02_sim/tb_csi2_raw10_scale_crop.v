`timescale 1ns / 1ps

module tb_csi2_raw10_scale_crop;
    reg clk = 1'b0;
    reg rst = 1'b1;
    reg in_valid = 1'b0;
    reg [15:0] in_data = 16'd0;
    reg in_line_end = 1'b0;
    reg [15:0] in_word_count = 16'd50;
    reg [7:0] in_data_type = 8'h2B;
    reg in_frame_end = 1'b0;
    wire out_valid;
    wire [15:0] out_data;
    wire out_frame_start;
    wire out_line_end;
    wire [15:0] out_word_count;
    wire out_frame_end;
    wire format_error;
    integer output_words = 0;
    integer output_lines = 0;
    integer frame_starts = 0;
    integer frame_ends = 0;
    integer max_token_count = 0;

    always #5 clk = ~clk;

    csi2_raw10_scale_crop #(
        .SRC_WIDTH(40), .SRC_HEIGHT(16), .SRC_CROP_X(4),
        .SCALE_WIDTH(24), .SCALE_HEIGHT(12),
        .OUT_HEIGHT(8), .CROP_Y(2), .FIFO_AW(4)
    ) dut (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_data(in_data),
        .in_line_end(in_line_end), .in_word_count(in_word_count),
        .in_data_type(in_data_type), .in_frame_end(in_frame_end),
        .out_valid(out_valid), .out_data(out_data),
        .out_frame_start(out_frame_start), .out_line_end(out_line_end),
        .out_word_count(out_word_count), .out_frame_end(out_frame_end),
        .format_error(format_error)
    );

    always @(posedge clk) begin
        if (!rst) begin
            if (out_frame_start) frame_starts = frame_starts + 1;
            if (out_frame_end) frame_ends = frame_ends + 1;
            if (dut.token_count > max_token_count)
                max_token_count = dut.token_count;
            if (out_valid) begin
                if (out_word_count != 30) begin
                    $display("FAIL output word count=%0d", out_word_count);
                    $fatal(1);
                end
                output_words = output_words + 1;
            end
            if (out_line_end) begin
                output_lines = output_lines + 1;
            end
        end
    end

    task send_word;
        input [7:0] hi;
        input [7:0] lo;
        begin
            @(negedge clk);
            in_valid = 1'b1;
            in_data = {hi, lo};
        end
    endtask

    task send_frame;
        input send_explicit_fe;
        integer line_idx, group_idx;
        begin
            for (line_idx = 0; line_idx < 16; line_idx = line_idx + 1) begin
                for (group_idx = 0; group_idx < 10; group_idx = group_idx + 2) begin
                    send_word(8'h40, 8'h40);
                    send_word(8'h40, 8'h40);
                    send_word(8'h40, 8'h00);
                    send_word(8'h40, 8'h40);
                    send_word(8'h40, 8'h00);
                end
                @(negedge clk);
                in_valid = 1'b0;
                in_line_end = 1'b1;
                @(negedge clk);
                in_line_end = 1'b0;
            end

            if (send_explicit_fe) begin
                @(negedge clk);
                in_frame_end = 1'b1;
                @(negedge clk);
                in_frame_end = 1'b0;
            end
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        rst = 1'b0;

        // The first frame has a delayed FE after the scaler's height-based
        // boundary. The second frame has no CSI FS or FE markers at all.
        send_frame(1'b1);
        repeat (40) @(negedge clk);
        send_frame(1'b0);
        repeat (40) @(negedge clk);

        if (output_words != 2*8*15 || output_lines != 2*8 ||
            frame_starts != 2 || frame_ends != 2 || format_error) begin
            $display("FAIL words=%0d lines=%0d FS=%0d FE=%0d err=%0d",
                     output_words, output_lines, frame_starts, frame_ends, format_error);
            $fatal(1);
        end
        if (max_token_count >= 16) begin
            $display("FAIL token FIFO peak=%0d exceeds 16-entry budget", max_token_count);
            $fatal(1);
        end
        $display("PASS RAW10 scale/crop: words=%0d lines=%0d FS=%0d FE=%0d token_peak=%0d",
                 output_words, output_lines, frame_starts, frame_ends, max_token_count);
        $finish;
    end
endmodule
