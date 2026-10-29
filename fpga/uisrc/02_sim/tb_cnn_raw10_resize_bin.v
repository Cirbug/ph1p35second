`timescale 1ns / 1ps

module tb_cnn_raw10_resize_bin;
    reg clk = 1'b0;
    reg rst = 1'b1;
    reg in_valid = 1'b0;
    reg [15:0] in_data = 16'd0;
    reg in_line_end = 1'b0;
    reg in_frame_end = 1'b0;
    wire frame_start;
    wire pixel_valid;
    wire [7:0] pixel_data;
    wire frame_complete;
    wire [9:0] pixel_x;
    wire [8:0] pixel_y;
    wire format_error;

    reg [7:0] line_bytes [0:49];
    integer emitted;
    integer start_count;
    integer complete_count;
    integer y, g, k, n;

    always #5 clk = ~clk;

    cnn_raw10_resize_bin #(
        .THRESHOLD(8'd128),
        .SRC_WIDTH(40), .SRC_HEIGHT(20),
        .DST_WIDTH(5), .DST_HEIGHT(4)
    ) dut (
        .clk(clk), .rst(rst),
        .in_valid(in_valid), .in_data(in_data),
        .in_line_end(in_line_end), .in_frame_end(in_frame_end),
        .frame_start(frame_start), .pixel_valid(pixel_valid),
        .pixel_data(pixel_data), .frame_complete(frame_complete),
        .pixel_x(pixel_x), .pixel_y(pixel_y), .format_error(format_error)
    );

    always @(posedge clk) begin
        if (!rst) begin
            if (frame_start) start_count = start_count + 1;
            if (pixel_valid) begin
                if (pixel_x !== (emitted % 5) || pixel_y !== (emitted / 5)) begin
                    $display("FAIL coord #%0d got %0d,%0d", emitted, pixel_x, pixel_y);
                    $fatal(1);
                end
                if (pixel_data !== 8'h00) begin
                    $display("FAIL binary pixel #%0d = %02h", emitted, pixel_data);
                    $fatal(1);
                end
                emitted = emitted + 1;
            end
            if (frame_complete) complete_count = complete_count + 1;
        end
    end

    task send_beat;
        input [7:0] a;
        input [7:0] b;
        begin
            in_valid = 1'b1;
            in_data = {a,b};
            @(negedge clk);
        end
    endtask

    initial begin
        emitted = 0; start_count = 0; complete_count = 0;
        repeat (4) @(negedge clk);
        rst = 1'b0;

        // 40 RAW10 pixels per row: black high bytes and zero packed LSBs.
        for (y = 0; y < 20; y = y + 1) begin
            for (g = 0; g < 10; g = g + 1) begin
                line_bytes[g*5+0] = 8'h20;
                line_bytes[g*5+1] = 8'h20;
                line_bytes[g*5+2] = 8'h20;
                line_bytes[g*5+3] = 8'h20;
                line_bytes[g*5+4] = 8'h00;
            end
            for (k = 0; k < 25; k = k + 1)
                send_beat(line_bytes[k*2], line_bytes[k*2+1]);
            in_valid = 1'b0;
            in_line_end = 1'b1;
            @(negedge clk);
            in_line_end = 1'b0;
        end

        @(negedge clk);
        in_frame_end = 1'b1;
        @(negedge clk);
        in_frame_end = 1'b0;
        repeat (8) @(negedge clk);

        if (emitted != 20 || start_count != 1 || complete_count != 1 || format_error) begin
            $display("FAIL emitted=%0d start=%0d complete=%0d format_error=%0d",
                     emitted, start_count, complete_count, format_error);
            $fatal(1);
        end
        $display("PASS RAW10 scale/binarize: pixels=%0d frame_start=%0d frame_complete=%0d",
                 emitted, start_count, complete_count);
        $finish;
    end
endmodule
