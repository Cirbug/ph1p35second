`timescale 1ns / 1ps

module tb_cnn_binary_mipi_preview;
    localparam integer SRC_WIDTH = 8;
    localparam integer SRC_HEIGHT = 2;
    localparam integer SCALE = 2;
    localparam integer OUT_WORDS = 10;
    localparam integer EXPECTED_TOKENS = 46;

    reg clk = 1'b0;
    reg rst = 1'b1;
    reg pixel_valid = 1'b0;
    reg [7:0] pixel_data = 8'd0;
    reg [9:0] pixel_x = 10'd0;
    reg [8:0] pixel_y = 9'd0;
    reg [7:0] source_row [0:SRC_HEIGHT-1];
    reg [7:0] expected_bytes [0:19];
    reg [17:0] expected_tokens [0:EXPECTED_TOKENS-1];
    wire out_valid;
    wire [17:0] out_token;
    wire input_overrun;
    reg out_ready = 1'b0;
    integer cycle_count = 0;
    integer token_count = 0;
    integer expected_count = 0;
    integer row, x, g, k, word_index;

    always #5 clk = ~clk;
    always @(posedge clk)
        if (!rst) cycle_count <= cycle_count + 1;
    always @* out_ready = ((cycle_count % 5) != 1);

    cnn_binary_mipi_preview #(
        .SRC_WIDTH(SRC_WIDTH), .SRC_HEIGHT(SRC_HEIGHT), .SCALE(SCALE)
    ) dut (
        .clk(clk), .rst(rst),
        .pixel_valid(pixel_valid), .pixel_data(pixel_data),
        .pixel_x(pixel_x), .pixel_y(pixel_y),
        .out_ready(out_ready), .out_valid(out_valid),
        .out_token(out_token), .input_overrun(input_overrun)
    );

    always @(posedge clk) begin
        if (!rst && out_valid && out_ready) begin
            if (token_count >= expected_count || out_token !== expected_tokens[token_count]) begin
                $display("FAIL token #%0d got=%05h expected=%05h",
                         token_count, out_token, expected_tokens[token_count]);
                $display("state=%0d bank=%0d capture=%0d line0=%b line1=%b word=%0d byte_index=%h packed=%h byte0=%02h byte1=%02h",
                         dut.state, dut.output_bank, dut.capture_bank,
                         dut.line_buffer0, dut.line_buffer1, dut.word_index,
                         dut.byte_index, dut.packed_word,
                         dut.packed_raw10_byte({dut.word_index, 1'b0}),
                         dut.packed_raw10_byte({dut.word_index, 1'b0} + 1'b1));
                $fatal(1);
            end
            token_count = token_count + 1;
        end
    end

    task append_row_tokens;
        input integer row_index;
        integer repeat_index;
        integer byte_index;
        integer source_x;
        integer pixel_index;
        begin
            for (repeat_index = 0; repeat_index < SCALE; repeat_index = repeat_index + 1) begin
                for (g = 0; g < (SRC_WIDTH*SCALE)/4; g = g + 1) begin
                    for (k = 0; k < 4; k = k + 1) begin
                        pixel_index = g*4 + k;
                        source_x = pixel_index / SCALE;
                        expected_bytes[g*5+k] = source_row[row_index][source_x] ? 8'hff : 8'h00;
                    end
                    expected_bytes[g*5+4] = 8'h00;
                    for (k = 0; k < 4; k = k + 1) begin
                        pixel_index = g*4 + k;
                        source_x = pixel_index / SCALE;
                        if (source_row[row_index][source_x])
                            expected_bytes[g*5+4][k*2 +: 2] = 2'b11;
                    end
                end
                for (word_index = 0; word_index < OUT_WORDS; word_index = word_index + 1) begin
                    byte_index = word_index * 2;
                    expected_tokens[expected_count] = {
                        2'b00, expected_bytes[byte_index], expected_bytes[byte_index+1]
                    };
                    expected_count = expected_count + 1;
                end
                expected_tokens[expected_count] = {2'b10, 16'd20};
                expected_count = expected_count + 1;
            end
        end
    endtask

    task send_pixel;
        input integer row_index;
        input integer x_index;
        begin
            pixel_valid = 1'b1;
            pixel_x = x_index;
            pixel_y = row_index;
            pixel_data = source_row[row_index][x_index] ? 8'hff : 8'h00;
            @(negedge clk);
        end
    endtask

    initial begin
        source_row[0] = 8'b10101010;
        source_row[1] = 8'b11010011;

        expected_tokens[expected_count] = {2'b01, 16'd0};
        expected_count = expected_count + 1;
        append_row_tokens(0);
        append_row_tokens(1);
        expected_tokens[expected_count] = {2'b11, 16'd0};
        expected_count = expected_count + 1;
        if (expected_count != EXPECTED_TOKENS) begin
            $display("FAIL test vector length=%0d", expected_count);
            $fatal(1);
        end

        repeat (4) @(negedge clk);
        rst = 1'b0;
        for (row = 0; row < SRC_HEIGHT; row = row + 1) begin
            for (x = 0; x < SRC_WIDTH; x = x + 1)
                send_pixel(row, x);
        end
        pixel_valid = 1'b0;

        wait (token_count == EXPECTED_TOKENS);
        repeat (3) @(negedge clk);
        if (input_overrun) begin
            $display("FAIL preview input overrun asserted");
            $fatal(1);
        end
        $display("PASS binary MIPI preview: tokens=%0d RAW10 line WC=20, backpressure exercised",
                 token_count);
        $finish;
    end

    initial begin
        #100000;
        $display("FAIL timeout token_count=%0d", token_count);
        $fatal(1);
    end
endmodule
