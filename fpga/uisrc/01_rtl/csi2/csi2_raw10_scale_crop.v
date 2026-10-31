`timescale 1ns / 1ps

// RAW10 scaler/cropper for the B-board relay path.
//
// The geometry matches the former A-board RGB stage, but the operation is
// performed in Bayer 2-pixel units so the RAW10 phase remains valid:
//   2592x1944 -> remove 8 pixels at each side -> 1920x1440 -> crop 180 rows
//   at the top and bottom -> 1920x1080.
// The output is a CSI-2 RAW10 byte stream (2400 bytes per line).
module csi2_raw10_scale_crop #(
    parameter integer SRC_WIDTH    = 2592,
    parameter integer SRC_HEIGHT   = 1944,
    parameter integer SRC_CROP_X   = 8,
    parameter integer SCALE_WIDTH  = 1920,
    parameter integer SCALE_HEIGHT = 1440,
    parameter integer OUT_HEIGHT   = 1080,
    parameter integer CROP_Y       = 180,
    parameter integer FIFO_AW      = 4
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
    output reg         out_frame_start,
    output reg         out_line_end,
    output reg  [15:0] out_word_count,
    output reg         out_frame_end,
    output reg         format_error
);
    localparam integer SRC_GROUPS      = SRC_WIDTH / 4;
    localparam integer SRC_LINE_BYTES = SRC_WIDTH * 10 / 8;
    localparam integer ACTIVE_WIDTH    = SRC_WIDTH - (SRC_CROP_X * 2);
    localparam integer CROP_GROUPS    = SRC_CROP_X / 4;
    localparam integer ACTIVE_GROUPS  = ACTIVE_WIDTH / 4;
    localparam integer SRC_PAIRS      = ACTIVE_WIDTH / 2;
    localparam integer DST_PAIRS      = SCALE_WIDTH / 2;
    localparam integer SRC_ROW_PAIRS = SRC_HEIGHT / 2;
    localparam integer DST_ROW_PAIRS = SCALE_HEIGHT / 2;
    localparam integer CROP_PAIR_Y   = CROP_Y / 2;
    localparam integer OUT_PAIR_ROWS  = OUT_HEIGHT / 2;
    localparam integer H_ACC_INIT    = SRC_PAIRS - DST_PAIRS;
    localparam integer V_ACC_INIT    = SRC_ROW_PAIRS - DST_ROW_PAIRS;
    localparam integer FIFO_DEPTH    = (1 << FIFO_AW);

    // RAW10 group assembly.  pack_count follows 0,2,4,1,3,0 as 5-byte
    // groups are consumed two bytes at a time.
    reg [2:0] pack_count;
    reg [7:0] b0, b1, b2, b3;
    reg [15:0] src_group_count;
    reg [15:0] src_line_count;
    reg        line_started;
    reg        pair_keep;
    reg        output_line;
    reg        cell_output_line;
    reg        frame_start_sent;
    reg        auto_end_pending;
    reg [10:0] scaled_pair_y;
    reg [10:0] out_x;
    reg [12:0] h_acc_pair;
    reg [12:0] v_acc_pair;
    reg [9:0]  pending_p0, pending_p1;
    reg [1:0]  pending_pixels;

    // A compact shift FIFO keeps data bytes and boundary tokens in order.
    // Token type 00=data byte, 01=line end, 10=frame end, 11=frame start.
    reg [9:0] token_q [0:FIFO_DEPTH-1];
    reg [9:0] token_tmp [0:FIFO_DEPTH-1];
    reg [FIFO_AW:0] token_count;

    integer i;
    integer j;
    integer pop_n;
    integer base_count;
    integer next_count;
    integer push_n;
    reg [9:0] gen_tok [0:10];
    reg [9:0] p0, p1, p2, p3;
    reg [9:0] cur0, cur1;
    reg [12:0] h_sum;
    reg [12:0] h_acc_tmp;
    reg [10:0] out_x_tmp;
    reg        selected_pair;
    reg [2:0]  work_count;
    reg [9:0]  work_p0, work_p1, work_p2, work_p3;
    reg [9:0]  raw10_low;
    reg        raw10_line;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pack_count      <= 3'd0;
            b0              <= 8'd0;
            b1              <= 8'd0;
            b2              <= 8'd0;
            b3              <= 8'd0;
            src_group_count <= 16'd0;
            src_line_count  <= 16'd0;
            line_started    <= 1'b0;
            pair_keep       <= 1'b0;
            output_line     <= 1'b0;
            cell_output_line <= 1'b0;
            frame_start_sent<= 1'b0;
            auto_end_pending<= 1'b0;
            scaled_pair_y   <= 11'd0;
            out_x           <= 11'd0;
            h_acc_pair      <= H_ACC_INIT;
            v_acc_pair      <= V_ACC_INIT;
            pending_p0      <= 10'd0;
            pending_p1      <= 10'd0;
            pending_pixels  <= 2'd0;
            token_count     <= {(FIFO_AW+1){1'b0}};
            out_valid       <= 1'b0;
            out_data        <= 16'd0;
            out_frame_start <= 1'b0;
            out_line_end    <= 1'b0;
            out_word_count  <= SCALE_WIDTH * 10 / 8;
            out_frame_end   <= 1'b0;
            format_error    <= 1'b0;
        end else begin
            out_valid      <= 1'b0;
            out_frame_start<= 1'b0;
            out_line_end   <= 1'b0;
            out_frame_end  <= 1'b0;
            out_word_count <= SCALE_WIDTH * 10 / 8;

            // Some sensor modes omit CSI frame markers. Once the configured
            // line count closes a frame, ignore a delayed explicit FE and
            // clear the guard when the next frame begins.
            if (in_valid && (in_data_type[5:0] == 6'h2B) && auto_end_pending)
                auto_end_pending <= 1'b0;

            // Pop the oldest token.  Data bytes are emitted in pairs.
            pop_n = 0;
            if (token_count != 0) begin
                if (token_q[0][9:8] == 2'b00) begin
                    if (token_count == 1) begin
                        // Hold an odd trailing byte until the next five-byte
                        // RAW10 group supplies its byte pair.
                        pop_n = 0;
                    end else if (token_q[1][9:8] == 2'b00) begin
                        out_valid <= 1'b1;
                        out_data  <= {token_q[0][7:0], token_q[1][7:0]};
                        pop_n = 2;
                    end else begin
                        format_error <= 1'b1;
                        pop_n = 1;
                    end
                end else if (token_q[0][9:8] == 2'b01) begin
                    out_line_end <= 1'b1;
                    pop_n = 1;
                end else if (token_q[0][9:8] == 2'b10) begin
                    out_frame_end <= 1'b1;
                    pop_n = 1;
                end else if (token_q[0][9:8] == 2'b11) begin
                    out_frame_start <= 1'b1;
                    pop_n = 1;
                end else begin
                    format_error <= 1'b1;
                    pop_n = 1;
                end
            end

            push_n = 0;
            for (i = 0; i < 11; i = i + 1) gen_tok[i] = 10'd0;
            raw10_line = (in_data_type[5:0] == 6'h2B);

            // Assemble and process a complete source RAW10 group.
            if (in_valid && raw10_line) begin
                if (pack_count == 3'd4) begin
                    // Fifth RAW10 byte is in_hi; in_lo starts the next group.
                    // Its bits [1:0],[3:2],[5:4],[7:6] belong to P0..P3.
                    p0 = {b0, in_data[9:8]};
                    p1 = {b1, in_data[11:10]};
                    p2 = {b2, in_data[13:12]};
                    p3 = {b3, in_data[15:14]};
                    b0 <= in_data[7:0];
                    pack_count <= 3'd1;
                end else if (pack_count == 3'd3) begin
                    p0 = {b0, in_data[1:0]};
                    p1 = {b1, in_data[3:2]};
                    p2 = {b2, in_data[5:4]};
                    p3 = {in_data[15:8], in_data[7:6]};
                    pack_count <= 3'd0;
                end else if (pack_count == 3'd0) begin
                    b0 <= in_data[15:8];
                    b1 <= in_data[7:0];
                    pack_count <= 3'd2;
                end else if (pack_count == 3'd2) begin
                    b2 <= in_data[15:8];
                    b3 <= in_data[7:0];
                    pack_count <= 3'd4;
                end else begin
                    b1 <= in_data[15:8];
                    b2 <= in_data[7:0];
                    pack_count <= 3'd3;
                end

                if ((pack_count == 3'd3) || (pack_count == 3'd4)) begin
                    h_acc_tmp = h_acc_pair;
                    out_x_tmp = out_x;
                    work_count = pending_pixels;
                    work_p0 = pending_p0;
                    work_p1 = pending_p1;

                    // Each source group contains two Bayer pairs.  Selecting
                    // pairs, rather than individual pixels, preserves the
                    // output RAW10 Bayer phase.
                    for (j = 0; j < 2; j = j + 1) begin
                        if (j == 0) begin cur0 = p0; cur1 = p1; end
                        else begin cur0 = p2; cur1 = p3; end

                        if ((src_group_count >= CROP_GROUPS) &&
                            (src_group_count < CROP_GROUPS + ACTIVE_GROUPS)) begin
                            h_sum = h_acc_tmp + DST_PAIRS;
                            selected_pair = (h_sum >= SRC_PAIRS);
                            if (selected_pair) h_acc_tmp = h_sum - SRC_PAIRS;
                            else               h_acc_tmp = h_sum;

                            if (selected_pair && pair_keep && output_line) begin
                                if (work_count == 0) begin
                                    work_p0 = cur0;
                                    work_p1 = cur1;
                                    work_count = 2;
                                end else if (work_count == 2) begin
                                    work_p2 = cur0;
                                    work_p3 = cur1;
                                    work_count = 4;
                                    gen_tok[push_n] = {2'b00, work_p0[9:2]};
                                    gen_tok[push_n+1] = {2'b00, work_p1[9:2]};
                                    gen_tok[push_n+2] = {2'b00, work_p2[9:2]};
                                    gen_tok[push_n+3] = {2'b00, work_p3[9:2]};
                                    gen_tok[push_n+4] = {2'b00,
                                        {work_p3[1:0], work_p2[1:0],
                                         work_p1[1:0], work_p0[1:0]}};
                                    push_n = push_n + 5;
                                    work_count = 0;
                                end else begin
                                    format_error <= 1'b1;
                                end
                                out_x_tmp = out_x_tmp + 2;
                            end
                        end
                    end
                    h_acc_pair <= h_acc_tmp;
                    out_x <= out_x_tmp;
                    src_group_count <= src_group_count + 1'b1;

                    pending_pixels <= work_count;
                    pending_p0 <= work_p0;
                    pending_p1 <= work_p1;
                    if ((push_n != 0) && !frame_start_sent) begin
                        for (i = 10; i > 0; i = i - 1)
                            if (i <= push_n) gen_tok[i] = gen_tok[i-1];
                        gen_tok[0] = {2'b11, 8'd0};
                        push_n = push_n + 1;
                        frame_start_sent <= 1'b1;
                    end
                end
            end

            // The first payload beat of a source line establishes its vertical
            // pair phase.  The beat cannot complete a RAW10 group, so the new
            // state is active before the first selected pair is processed.
            if (in_valid && raw10_line && !line_started) begin
                line_started <= 1'b1;
                out_x <= 11'd0;
                h_acc_pair <= H_ACC_INIT;
                pending_pixels <= 2'd0;
                if (src_line_count[0] == 1'b0) begin
                    if ((v_acc_pair + DST_ROW_PAIRS) >= SRC_ROW_PAIRS) begin
                        pair_keep <= 1'b1;
                        cell_output_line <= (scaled_pair_y >= CROP_PAIR_Y) &&
                                             (scaled_pair_y < CROP_PAIR_Y + OUT_PAIR_ROWS);
                        output_line <= (scaled_pair_y >= CROP_PAIR_Y) &&
                                       (scaled_pair_y < CROP_PAIR_Y + OUT_PAIR_ROWS);
                        v_acc_pair <= v_acc_pair + DST_ROW_PAIRS - SRC_ROW_PAIRS;
                        scaled_pair_y <= scaled_pair_y + 1'b1;
                    end else begin
                        pair_keep <= 1'b0;
                        output_line <= 1'b0;
                        cell_output_line <= 1'b0;
                        v_acc_pair <= v_acc_pair + DST_ROW_PAIRS;
                    end
                end else begin
                    output_line <= pair_keep && cell_output_line;
                end
            end

            if (in_line_end && raw10_line) begin
                if ((in_word_count != SRC_LINE_BYTES) ||
                    (src_group_count != SRC_GROUPS) || (pack_count != 3'd0))
                    format_error <= 1'b1;
                if (output_line && (out_x != SCALE_WIDTH))
                    format_error <= 1'b1;

                if (output_line) begin
                    if (push_n < 11) begin
                        gen_tok[push_n] = {2'b01, 8'd0};
                        push_n = push_n + 1;
                    end else format_error <= 1'b1;
                end

                src_group_count <= 16'd0;
                src_line_count <= (src_line_count >= SRC_HEIGHT-1) ?
                                  16'd0 : src_line_count + 1'b1;
                line_started <= 1'b0;
                pair_keep <= pair_keep;
                if (src_line_count[0]) output_line <= 1'b0;
                out_x <= 11'd0;
                h_acc_pair <= H_ACC_INIT;
                pending_pixels <= 2'd0;
                pack_count <= 3'd0;

                if ((src_line_count >= SRC_HEIGHT-1) && !in_frame_end) begin
                    if (push_n < 11) begin
                        gen_tok[push_n] = {2'b10, 8'd0};
                        push_n = push_n + 1;
                    end else format_error <= 1'b1;
                    pair_keep <= 1'b0;
                    output_line <= 1'b0;
                    cell_output_line <= 1'b0;
                    frame_start_sent <= 1'b0;
                    scaled_pair_y <= 11'd0;
                    h_acc_pair <= H_ACC_INIT;
                    v_acc_pair <= V_ACC_INIT;
                    pending_pixels <= 2'd0;
                    auto_end_pending <= 1'b1;
                end
            end

            if (in_frame_end) begin
                // FS also closes/reset-aligns the previous source interval.
                // If no visible crop was opened, reset phase without emitting
                // an orphan FE (which would look like an empty/bad frame).
                if (!auto_end_pending && frame_start_sent) begin
                    if (push_n < 11) begin
                        gen_tok[push_n] = {2'b10, 8'd0};
                        push_n = push_n + 1;
                    end else format_error <= 1'b1;
                end
                auto_end_pending <= 1'b0;
                src_group_count <= 16'd0;
                src_line_count <= 16'd0;
                line_started <= 1'b0;
                pair_keep <= 1'b0;
                output_line <= 1'b0;
                cell_output_line <= 1'b0;
                frame_start_sent <= 1'b0;
                scaled_pair_y <= 11'd0;
                out_x <= 11'd0;
                h_acc_pair <= H_ACC_INIT;
                v_acc_pair <= V_ACC_INIT;
                pending_pixels <= 2'd0;
                pack_count <= 3'd0;
            end

            // Shift out the words already consumed, then append newly
            // generated bytes/markers.  The queue is intentionally wider than
            // the maximum short burst (five data bytes plus one marker).
            base_count = token_count - pop_n;
            if (base_count < 0) base_count = 0;
            for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
                if (i < base_count) token_tmp[i] = token_q[i + pop_n];
                else token_tmp[i] = 10'd0;
            end
            next_count = base_count + push_n;
            if (next_count > FIFO_DEPTH) begin
                format_error <= 1'b1;
                next_count = base_count;
                push_n = 0;
            end else begin
                for (i = 0; i < 11; i = i + 1)
                    if (i < push_n)
                        token_tmp[base_count+i] = gen_tok[i];
            end
            for (i = 0; i < FIFO_DEPTH; i = i + 1)
                token_q[i] <= token_tmp[i];
            token_count <= next_count;
        end
    end
endmodule
