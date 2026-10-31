`timescale 1ns / 1ps

module tb_csi2_crop_ddr_fanout;
    reg rx_clk = 1'b0;
    reg ddr_clk = 1'b0;
    always #5 rx_clk = ~rx_clk;
    always #4 ddr_clk = ~ddr_clk;

    reg rst_n = 1'b0;
    reg in_valid = 1'b0;
    reg [15:0] in_data = 16'd0;
    reg in_line_end = 1'b0;
    reg in_frame_end = 1'b0;

    wire crop_valid;
    wire [15:0] crop_data;
    wire crop_frame_start;
    wire crop_line_end;
    wire [15:0] crop_word_count;
    wire crop_frame_end;
    wire crop_error;

    csi2_raw10_scale_crop #(
        .SRC_WIDTH(48), .SRC_HEIGHT(24), .SRC_CROP_X(4),
        .SCALE_WIDTH(32), .SCALE_HEIGHT(20),
        .OUT_HEIGHT(16), .CROP_Y(2), .FIFO_AW(4)
    ) u_crop (
        .clk(rx_clk), .rst(~rst_n),
        .in_valid(in_valid), .in_data(in_data),
        .in_line_end(in_line_end), .in_word_count(16'd60),
        .in_data_type(8'h2B), .in_frame_end(in_frame_end),
        .out_valid(crop_valid), .out_data(crop_data),
        .out_frame_start(crop_frame_start), .out_line_end(crop_line_end),
        .out_word_count(crop_word_count), .out_frame_end(crop_frame_end),
        .format_error(crop_error)
    );

    wire cache_wr_en;
    wire [24:0] cache_wr_addr;
    wire [127:0] cache_wr_data;
    wire cache_rd_en;
    wire [24:0] cache_rd_addr;
    wire cache_frame_start;
    wire cache_word_valid;
    wire [15:0] cache_word_data;
    wire cache_line_end;
    wire cache_frame_end;
    wire cache_error;
    wire [15:0] dropped_frames;
    wire [15:0] committed_frames;
    wire [1:0] ready_count;

    reg cache_rd_valid = 1'b0;
    reg [127:0] cache_rd_data = 128'd0;
    reg [7:0] rd_valid_pipe = 8'd0;
    reg [127:0] rd_data_pipe [0:7];
    reg [127:0] memory [0:119];

    ddr_triple_frame_cache #(
        .FRAME_WORDS16(320),
        .LINE_WORDS16(20),
        .FRAME_LINES(16),
        .FRAME_PACK_WORDS(40),
        .FRAME_ADDR_SPAN(320),
        .PACK_FIFO_AW(4),
        .RD_FIFO_AW(4)
    ) u_cache (
        .rx_clk(rx_clk), .rx_rst_n(rst_n),
        .rx_frame_start(crop_frame_start),
        .rx_word_valid(crop_valid), .rx_word_data(crop_data),
        .rx_line_end(crop_line_end), .rx_frame_end(crop_frame_end),
        .ddr_clk(ddr_clk), .ddr_rst_n(rst_n), .ddr_init_done(1'b1),
        .ddr_user_wr_en(cache_wr_en), .ddr_user_wr_addr(cache_wr_addr),
        .ddr_user_wr_data(cache_wr_data),
        .ddr_user_rd_en(cache_rd_en), .ddr_user_rd_addr(cache_rd_addr),
        .ddr_user_ready(1'b1), .ddr_user_rd_valid(cache_rd_valid),
        .ddr_user_rd_data(cache_rd_data),
        .out_ready(1'b1),
        .out_frame_start(cache_frame_start),
        .out_word_valid(cache_word_valid), .out_word_data(cache_word_data),
        .out_line_end(cache_line_end), .out_frame_end(cache_frame_end),
        .frame_busy(), .frame_error(cache_error),
        .dropped_frames(dropped_frames), .committed_frames(committed_frames),
        .ready_count(ready_count)
    );

    integer pipe_i;
    integer addr_index;
    always @(posedge ddr_clk) begin
        if (!rst_n) begin
            cache_rd_valid <= 1'b0;
            cache_rd_data <= 128'd0;
            rd_valid_pipe <= 8'd0;
        end else begin
            rd_valid_pipe <= {rd_valid_pipe[6:0], cache_rd_en};
            cache_rd_valid <= rd_valid_pipe[7];
            cache_rd_data <= rd_data_pipe[7];
            if (cache_wr_en) begin
                addr_index = cache_wr_addr >> 3;
                if (addr_index < 0 || addr_index >= 120)
                    $fatal(1, "DDR write address outside test memory: %0d", addr_index);
                memory[addr_index] <= cache_wr_data;
            end
            if (cache_rd_en) begin
                addr_index = cache_rd_addr >> 3;
                if (addr_index < 0 || addr_index >= 120)
                    $fatal(1, "DDR read address outside test memory: %0d", addr_index);
                rd_data_pipe[0] <= memory[addr_index];
            end
            for (pipe_i = 1; pipe_i < 8; pipe_i = pipe_i + 1)
                rd_data_pipe[pipe_i] <= rd_data_pipe[pipe_i-1];
        end
    end

    reg [15:0] expected [0:319];
    integer expected_count = 0;
    integer crop_fs_count = 0;
    integer crop_line_count = 0;
    integer crop_fe_count = 0;
    integer read_count = 0;
    integer read_line_count = 0;
    integer read_fs_count = 0;
    integer read_fe_count = 0;
    integer cnn_pixel_count = 0;
    integer cnn_fs_count = 0;
    integer cnn_fe_count = 0;
    reg cnn_frame_start;
    wire cnn_pixel_valid;
    wire [7:0] cnn_pixel_data;
    wire cnn_frame_complete;
    wire [9:0] cnn_pixel_x;
    wire [8:0] cnn_pixel_y;
    wire cnn_error;

    always @(posedge rx_clk) begin
        if (rst_n) begin
            if (crop_frame_start)
                crop_fs_count = crop_fs_count + 1;
            if (crop_valid) begin
                if (expected_count >= 320)
                    $fatal(1, "crop produced too many words");
                if (crop_word_count != 16'd40)
                    $fatal(1, "crop WC=%0d, expected 40", crop_word_count);
                expected[expected_count] = crop_data;
                expected_count = expected_count + 1;
            end
            if (crop_line_end)
                crop_line_count = crop_line_count + 1;
            if (crop_frame_end)
                crop_fe_count = crop_fe_count + 1;
        end
    end

    cnn_raw10_resize_bin #(
        .THRESHOLD(8'd128), .SRC_WIDTH(32), .SRC_HEIGHT(16),
        .DST_WIDTH(8), .DST_HEIGHT(4)
    ) u_cnn_resize (
        .clk(ddr_clk), .rst(~rst_n),
        .in_valid(cache_word_valid), .in_data(cache_word_data),
        .in_line_end(cache_line_end), .in_frame_end(cache_frame_end),
        .frame_start(cnn_frame_start), .pixel_valid(cnn_pixel_valid),
        .pixel_data(cnn_pixel_data), .frame_complete(cnn_frame_complete),
        .pixel_x(cnn_pixel_x), .pixel_y(cnn_pixel_y),
        .format_error(cnn_error)
    );

    always @(posedge ddr_clk) begin
        if (rst_n) begin
            if (cache_frame_start)
                read_fs_count = read_fs_count + 1;
            if (cache_word_valid) begin
                if (read_count >= expected_count)
                    $fatal(1, "DDR read exceeded captured crop output");
                if (cache_word_data !== expected[read_count])
                    $fatal(1, "DDR word %0d mismatch: got %h expected %h",
                           read_count, cache_word_data, expected[read_count]);
                read_count = read_count + 1;
            end
            if (cache_line_end)
                read_line_count = read_line_count + 1;
            if (cache_frame_end)
                read_fe_count = read_fe_count + 1;
            if (cnn_frame_start)
                cnn_fs_count = cnn_fs_count + 1;
            if (cnn_pixel_valid)
                cnn_pixel_count = cnn_pixel_count + 1;
            if (cnn_frame_complete)
                cnn_fe_count = cnn_fe_count + 1;
        end
    end

    task send_frame;
        integer line_idx;
        integer beat_idx;
        begin
            for (line_idx = 0; line_idx < 24; line_idx = line_idx + 1) begin
                for (beat_idx = 0; beat_idx < 30; beat_idx = beat_idx + 1) begin
                    @(negedge rx_clk);
                    in_valid = 1'b1;
                    in_data = 16'h2000 + line_idx * 16 + beat_idx;
                    in_line_end = 1'b0;
                    in_frame_end = 1'b0;
                end
                @(negedge rx_clk);
                in_valid = 1'b0;
                in_line_end = 1'b1;
                in_frame_end = (line_idx == 23);
                @(negedge rx_clk);
                in_line_end = 1'b0;
                in_frame_end = 1'b0;
            end
        end
    endtask

    integer cycles;
    initial begin
        for (pipe_i = 0; pipe_i < 120; pipe_i = pipe_i + 1)
            memory[pipe_i] = 128'd0;
        repeat (5) @(posedge rx_clk);
        rst_n = 1'b1;
        repeat (5) @(posedge rx_clk);
        send_frame;

        cycles = 0;
        while ((committed_frames < 1) && (cycles < 10000)) begin
            @(posedge ddr_clk);
            cycles = cycles + 1;
        end
        if (committed_frames != 1)
            $fatal(1, "timeout waiting for DDR frame commit");

        cycles = 0;
        while (((read_fe_count < 1) || (cnn_fe_count < 1)) && (cycles < 10000)) begin
            @(posedge ddr_clk);
            cycles = cycles + 1;
        end

        if (cycles >= 10000 || expected_count != 320 || crop_fs_count != 1 ||
            crop_line_count != 16 || crop_fe_count != 1 || read_count != 320 ||
            read_fs_count != 1 || read_line_count != 16 || read_fe_count != 1 ||
            cnn_fs_count != 1 || cnn_pixel_count != 32 || cnn_fe_count != 1 ||
            crop_error || cache_error || cnn_error) begin
            $display("FAIL crop=%0d/%0d/%0d/%0d DDR=%0d/%0d/%0d/%0d CNN=%0d/%0d/%0d err=%0d/%0d/%0d",
                     expected_count, crop_fs_count, crop_line_count, crop_fe_count,
                     read_count, read_fs_count, read_line_count, read_fe_count,
                     cnn_fs_count, cnn_pixel_count, cnn_fe_count,
                     crop_error, cache_error, cnn_error);
            $fatal(1);
        end

        $display("PASS crop -> DDR -> relay/CNN: words=%0d lines=%0d CNN pixels=%0d",
                 read_count, read_line_count, cnn_pixel_count);
        $finish;
    end
endmodule
