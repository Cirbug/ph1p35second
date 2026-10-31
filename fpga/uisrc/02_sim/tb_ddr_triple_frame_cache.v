`timescale 1ns / 1ps

module tb_ddr_triple_frame_cache;
    reg rx_clk = 1'b0;
    reg ddr_clk = 1'b0;
    always #5 rx_clk = ~rx_clk;
    always #4 ddr_clk = ~ddr_clk;

    reg rst_n = 1'b0;
    reg rx_frame_start = 1'b0;
    reg rx_word_valid = 1'b0;
    reg [15:0] rx_word_data = 16'd0;
    reg rx_line_end = 1'b0;
    reg rx_frame_end = 1'b0;

    wire wr_en;
    wire [24:0] wr_addr;
    wire [127:0] wr_data;
    wire rd_en;
    wire [24:0] rd_addr;
    wire out_frame_start;
    wire out_word_valid;
    wire [15:0] out_word_data;
    wire out_line_end;
    wire out_frame_end;
    wire frame_busy;
    wire frame_error;
    wire [15:0] dropped_frames;
    wire [15:0] committed_frames;
    wire [1:0] ready_count;

    reg rd_valid = 1'b0;
    reg [127:0] rd_data = 128'd0;
    reg [7:0] rd_valid_pipe = 8'd0;
    reg [127:0] rd_data_pipe [0:7];
    reg [127:0] memory [0:5];
    reg out_ready = 1'b1;
    integer i;
    integer pipe_i;
    integer cycles;

    ddr_triple_frame_cache #(
        .FRAME_WORDS16(16),
        .LINE_WORDS16(8),
        .FRAME_LINES(2),
        .FRAME_PACK_WORDS(2),
        .FRAME_ADDR_SPAN(16),
        .PACK_FIFO_AW(2)
    ) dut (
        .rx_clk(rx_clk),
        .rx_rst_n(rst_n),
        .rx_frame_start(rx_frame_start),
        .rx_word_valid(rx_word_valid),
        .rx_word_data(rx_word_data),
        .rx_line_end(rx_line_end),
        .rx_frame_end(rx_frame_end),
        .ddr_clk(ddr_clk),
        .ddr_rst_n(rst_n),
        .ddr_init_done(1'b1),
        .ddr_user_wr_en(wr_en),
        .ddr_user_wr_addr(wr_addr),
        .ddr_user_wr_data(wr_data),
        .ddr_user_rd_en(rd_en),
        .ddr_user_rd_addr(rd_addr),
        .ddr_user_ready(1'b1),
        .ddr_user_rd_valid(rd_valid),
        .ddr_user_rd_data(rd_data),
        .out_ready(out_ready),
        .out_frame_start(out_frame_start),
        .out_word_valid(out_word_valid),
        .out_word_data(out_word_data),
        .out_line_end(out_line_end),
        .out_frame_end(out_frame_end),
        .frame_busy(frame_busy),
        .frame_error(frame_error),
        .dropped_frames(dropped_frames),
        .committed_frames(committed_frames),
        .ready_count(ready_count)
    );

    reg [17:0] captured [0:127];
    integer captured_count = 0;
    always @(posedge ddr_clk) begin
        if (!rst_n) begin
            rd_valid <= 1'b0;
            rd_data <= 128'd0;
            rd_valid_pipe <= 8'd0;
        end else begin
            rd_valid_pipe <= {rd_valid_pipe[6:0], rd_en};
            rd_valid <= rd_valid_pipe[7];
            rd_data <= rd_data_pipe[7];
            if (wr_en)
                memory[wr_addr >> 3] <= wr_data;
            if (rd_en)
                rd_data_pipe[0] <= memory[rd_addr >> 3];
            for (pipe_i = 1; pipe_i < 8; pipe_i = pipe_i + 1)
                rd_data_pipe[pipe_i] <= rd_data_pipe[pipe_i-1];
            if (out_frame_start) begin
                captured[captured_count] = {2'b01, 16'd0};
                captured_count = captured_count + 1;
            end
            if (out_word_valid) begin
                captured[captured_count] = {2'b00, out_word_data};
                captured_count = captured_count + 1;
            end
            if (out_line_end) begin
                captured[captured_count] = {2'b10, 16'd0};
                captured_count = captured_count + 1;
            end
            if (out_frame_end) begin
                captured[captured_count] = {2'b11, 16'd0};
                captured_count = captured_count + 1;
            end
        end
    end

    function [15:0] sample_word;
        input integer frame_id;
        input integer word_id;
        begin
            sample_word = 16'hA000 + (frame_id * 256) + word_id;
        end
    endfunction

    task send_frame;
        input integer frame_id;
        input integer word_count;
        integer word_id;
        begin
            @(negedge rx_clk);
            rx_frame_start = 1'b1;
            rx_word_valid = 1'b0;
            rx_line_end = 1'b0;
            rx_frame_end = 1'b0;
            @(negedge rx_clk);
            rx_frame_start = 1'b0;
            rx_word_valid = 1'b1;
            for (word_id = 0; word_id < word_count; word_id = word_id + 1) begin
                rx_word_data = sample_word(frame_id, word_id);
                @(negedge rx_clk);
                if ((word_id & 7) == 7) begin
                    rx_word_valid = 1'b0;
                    rx_line_end = 1'b1;
                    @(negedge rx_clk);
                    rx_line_end = 1'b0;
                    if (word_id + 1 < word_count)
                        rx_word_valid = 1'b1;
                end
            end
            rx_word_valid = 1'b0;
            rx_frame_end = 1'b1;
            @(negedge rx_clk);
            rx_frame_end = 1'b0;
        end
    endtask

    task wait_value;
        input integer target;
        integer cycles;
        begin
            cycles = 0;
            while ((committed_frames < target) && (cycles < 2000)) begin
                @(posedge ddr_clk);
                cycles = cycles + 1;
            end
            if (committed_frames < target)
                $fatal(1, "timeout waiting for commit %0d", target);
        end
    endtask

    task check_frame;
        input integer base;
        input integer frame_id;
        integer pos;
        integer word_id;
        begin
            if (captured[base] !== {2'b01, 16'd0})
                $fatal(1, "missing frame start at %0d", base);
            pos = base + 1;
            for (word_id = 0; word_id < 16; word_id = word_id + 1) begin
                if (captured[pos] !== {2'b00, sample_word(frame_id, word_id)})
                    $fatal(1, "frame %0d word %0d mismatch: %h", frame_id,
                           word_id, captured[pos]);
                pos = pos + 1;
                if ((word_id == 7) || (word_id == 15)) begin
                    if (captured[pos] !== {2'b10, 16'd0})
                        $fatal(1, "bad line marker after word %0d", word_id);
                    pos = pos + 1;
                end
            end
            if (captured[pos] !== {2'b11, 16'd0})
                $fatal(1, "missing frame end at %0d", pos);
        end
    endtask

    initial begin
        for (i = 0; i < 6; i = i + 1)
            memory[i] = 128'd0;
        repeat (5) @(posedge rx_clk);
        rst_n = 1'b1;
        repeat (6) @(posedge rx_clk);

        send_frame(1, 16);
        wait_value(1);
        // Hold the consumer at the frame boundary; the cache must resume
        // with the same token sequence after backpressure is released.
        out_ready = 1'b0;
        repeat (20) @(posedge ddr_clk);
        out_ready = 1'b1;
        cycles = 0;
        while ((captured_count < 20) && (cycles < 1000)) begin
            @(posedge ddr_clk);
            cycles = cycles + 1;
        end
        if (captured_count != 20)
            $fatal(1, "first frame output token count=%0d", captured_count);
        check_frame(0, 1);

        send_frame(2, 16);
        wait_value(2);
        cycles = 0;
        while ((captured_count < 40) && (cycles < 1000)) begin
            @(posedge ddr_clk);
            cycles = cycles + 1;
        end
        if (captured_count != 40)
            $fatal(1, "second frame output token count=%0d", captured_count);
        check_frame(20, 2);

        send_frame(3, 8);
        cycles = 0;
        while ((dropped_frames == 0) && (cycles < 1000)) begin
            @(posedge rx_clk);
            cycles = cycles + 1;
        end
        cycles = 0;
        while (!frame_error && (cycles < 1000)) begin
            @(posedge ddr_clk);
            cycles = cycles + 1;
        end
        if ((dropped_frames == 0) || !frame_error || committed_frames != 2)
            $fatal(1, "partial frame was not rejected cleanly");

        $display("PASS: raw DDR frame write/read, word order, reconstructed line markers, partial-frame abort");
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "testbench watchdog timeout");
    end
endmodule
