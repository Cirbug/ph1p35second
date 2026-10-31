`timescale 1ns / 1ps

module tb_csi2_scale_tx;
    reg clk = 1'b0;
    reg rst = 1'b1;
    reg rx_valid = 1'b0;
    reg [15:0] rx_data = 16'd0;

    wire parsed_valid;
    wire [15:0] parsed_data;
    wire parsed_line_end;
    wire [15:0] parsed_word_count;
    wire [7:0] parsed_data_type;
    wire parsed_frame_start;
    wire parsed_frame_end;
    wire parser_error;

    wire out_valid;
    wire [15:0] out_data;
    wire out_frame_start;
    wire out_line_end;
    wire [15:0] out_word_count;
    wire out_frame_end;
    wire scale_error;

    wire tx_en;
    wire [15:0] tx_data;
    wire tx_last;
    wire tx_overflow;
    wire tx_wc_error;
    wire tx_pkt_done;
    wire [7:0] tx_pkt_dt;
    wire [15:0] tx_pkt_wc;

    integer byte_count = 0;
    integer packet_count = 0;
    integer rx_payload_beats = 0;
    integer rx_lines = 0;
    integer scaled_beats = 0;
    integer scaled_lines = 0;
    integer scaled_frames = 0;
    integer packet_end [0:15];
    integer i;
    integer errors = 0;
    reg [7:0] captured [0:2047];

    always #5 clk = ~clk;

    csi2_rx_parse #(.LANE_NUM(2)) receiver (
        .I_clk(clk), .I_rst(rst), .I_hs_valid(rx_valid), .I_hs_data(rx_data),
        .O_wr_en(parsed_valid), .O_wr_data(parsed_data),
        .O_pkt_end(parsed_line_end), .O_pkt_wc(parsed_word_count),
        .O_pkt_dt(parsed_data_type), .O_frame_start(parsed_frame_start),
        .O_frame_end(parsed_frame_end), .O_wc_err(parser_error)
    );

    csi2_raw10_scale_crop #(
        .SRC_WIDTH(32), .SRC_HEIGHT(16), .SRC_CROP_X(4),
        .SCALE_WIDTH(16), .SCALE_HEIGHT(12),
        .OUT_HEIGHT(8), .CROP_Y(2), .FIFO_AW(4)
    ) scaler (
        .clk(clk), .rst(rst),
        .in_valid(parsed_valid), .in_data(parsed_data),
        .in_line_end(parsed_line_end), .in_word_count(parsed_word_count),
        .in_data_type(parsed_data_type), .in_frame_end(parsed_frame_end),
        .out_valid(out_valid), .out_data(out_data),
        .out_frame_start(out_frame_start), .out_line_end(out_line_end),
        .out_word_count(out_word_count), .out_frame_end(out_frame_end),
        .format_error(scale_error)
    );

    csi2_tx_packet #(
        .LANE_NUM(2), .VC(2'b00), .DT(8'h2B), .BUF_AW(6)
    ) transmitter (
        .I_clk(clk), .I_rst(rst),
        .I_wr_en(out_valid), .I_wr_data(out_data),
        .I_pkt_end(out_line_end), .I_pkt_wc(out_word_count),
        .I_frame_start(out_frame_start), .I_frame_end(out_frame_end),
        .O_hs_en(tx_en), .O_hs_data(tx_data), .O_hs_last(tx_last),
        .I_hs_ready(1'b1), .O_overflow(tx_overflow), .O_wc_err(tx_wc_error),
        .O_dbg_pkt_done(tx_pkt_done), .O_dbg_pkt_dt(tx_pkt_dt),
        .O_dbg_pkt_wc(tx_pkt_wc)
    );

    always @(posedge clk) begin
        if (!rst && parsed_valid) rx_payload_beats = rx_payload_beats + 1;
        if (!rst && parsed_line_end) rx_lines = rx_lines + 1;
        if (!rst && out_valid) scaled_beats = scaled_beats + 1;
        if (!rst && out_line_end) scaled_lines = scaled_lines + 1;
        if (!rst && out_frame_end) scaled_frames = scaled_frames + 1;
        if (!rst && tx_en) begin
            captured[byte_count] = tx_data[15:8];
            captured[byte_count + 1] = tx_data[7:0];
            byte_count = byte_count + 2;
            if (tx_last) begin
                packet_end[packet_count] = byte_count;
                packet_count = packet_count + 1;
            end
        end
    end

    task send_rx_beat;
        input [7:0] high_byte;
        input [7:0] low_byte;
        begin
            rx_valid = 1'b1;
            rx_data = {high_byte, low_byte};
            @(posedge clk);
            #1;
        end
    endtask

    task send_frame_without_markers;
        integer line_idx;
        integer beat_idx;
        begin
            for (line_idx = 0; line_idx < 16; line_idx = line_idx + 1) begin
                // CSI-2 RAW10 long packet: DI/WC, header ECC, 40-byte payload,
                // and one two-byte CRC beat. The parser intentionally ignores
                // the CRC value and consumes its beat as the packet tail.
                send_rx_beat(8'h2B, 8'd40);
                send_rx_beat(8'd0, 8'h1B); // ECC for WC=40, DI=2B
                for (beat_idx = 0; beat_idx < 20; beat_idx = beat_idx + 1)
                    send_rx_beat(8'h40, 8'h40);
                send_rx_beat(8'h00, 8'h00);
                rx_valid = 1'b0;
                repeat (3) @(negedge clk);
            end
        end
    endtask

    integer start;
    integer packet_len;
    initial begin
        for (i = 0; i < 16; i = i + 1) packet_end[i] = 0;
        repeat (4) @(negedge clk);
        rst = 1'b0;

        // No CSI FS or FE is sent. The receiver parses each line packet, then
        // the scaler creates the frame boundary from the configured height.
        send_frame_without_markers();
        repeat (400) @(negedge clk);

        $display("TRACE rx_payload=%0d rx_lines=%0d scaled_beats=%0d scaled_lines=%0d scaled_frames=%0d",
                 rx_payload_beats, rx_lines, scaled_beats, scaled_lines, scaled_frames);

        if (packet_count != 10) begin
            $display("FAIL packet count=%0d expected=10", packet_count);
            errors = errors + 1;
        end else begin
            start = 0;
            packet_len = packet_end[0] - start;
            if (packet_len != 4 || captured[start] != 8'h00 ||
                captured[start + 1] != 8'h00 || captured[start + 2] != 8'h00) begin
                $display("FAIL FS packet len=%0d header=%02h %02h %02h %02h",
                         packet_len, captured[start], captured[start+1],
                         captured[start+2], captured[start+3]);
                errors = errors + 1;
            end

            for (i = 1; i <= 8; i = i + 1) begin
                start = packet_end[i-1];
                packet_len = packet_end[i] - start;
                if (packet_len != 26 || captured[start] != 8'h2B ||
                    captured[start + 1] != 8'd20 || captured[start + 2] != 8'd0) begin
                    $display("FAIL line packet %0d len=%0d header=%02h %02h %02h",
                             i, packet_len, captured[start], captured[start+1],
                             captured[start+2]);
                    errors = errors + 1;
                end
            end

            start = packet_end[8];
            packet_len = packet_end[9] - start;
            if (packet_len != 4 || captured[start] != 8'h01 ||
                captured[start + 1] != 8'h00 || captured[start + 2] != 8'h00) begin
                $display("FAIL FE packet len=%0d header=%02h %02h %02h %02h",
                         packet_len, captured[start], captured[start+1],
                         captured[start+2], captured[start+3]);
                errors = errors + 1;
            end
        end

        if (parser_error || scale_error || tx_overflow || tx_wc_error) begin
            $display("FAIL parser_error=%0b scale_error=%0b tx_overflow=%0b tx_wc_error=%0b",
                     parser_error, scale_error, tx_overflow, tx_wc_error);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS RX parser->scale->TX without CSI markers: FS + 8 RAW10 lines (WC=20) + FE");
        else
            $display("FAIL scale->TX errors=%0d", errors);
        $finish;
    end
endmodule
