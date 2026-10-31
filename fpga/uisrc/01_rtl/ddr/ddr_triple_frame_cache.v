`timescale 1ns / 1ps

module ddr_triple_frame_cache #(
    parameter integer FRAME_WORDS16    = 2592 * 1944 * 10 / 16,
    parameter integer LINE_WORDS16     = 2592 * 10 / 16,
    parameter integer FRAME_LINES       = 1944,
    parameter integer FRAME_PACK_WORDS = (2592 * 1944 * 10 / 16) / 8,
    parameter integer FRAME_ADDR_SPAN  = ((2592 * 1944 * 10 / 16) / 8) * 8,
    parameter integer PACK_FIFO_AW     = 6,
    parameter integer RD_FIFO_AW       = 6
)(
    input  wire         rx_clk,
    input  wire         rx_rst_n,
    input  wire         rx_frame_start,
    input  wire         rx_word_valid,
    input  wire [15:0]  rx_word_data,
    input  wire         rx_line_end,
    input  wire         rx_frame_end,

    input  wire         ddr_clk,
    input  wire         ddr_rst_n,
    input  wire         ddr_init_done,
    output wire         ddr_user_wr_en,
    output wire [24:0]  ddr_user_wr_addr,
    output wire [127:0] ddr_user_wr_data,
    output wire         ddr_user_rd_en,
    output wire [24:0]  ddr_user_rd_addr,
    input  wire         ddr_user_ready,
    input  wire         ddr_user_rd_valid,
    input  wire [127:0] ddr_user_rd_data,

    input  wire         out_ready,
    output wire         out_frame_start,
    output wire         out_word_valid,
    output wire [15:0]  out_word_data,
    output wire         out_line_end,
    output wire         out_frame_end,

    output wire         frame_busy,
    output wire         frame_error,
    output reg  [15:0]  dropped_frames,
    output reg  [15:0]  committed_frames,
    output reg  [1:0]   ready_count
);

    localparam [1:0] REC_DATA   = 2'b00;
    localparam [1:0] REC_COMMIT = 2'b01;
    localparam [1:0] REC_ABORT  = 2'b10;

    localparam [2:0] RD_IDLE = 3'd0;
    localparam [2:0] RD_FS   = 3'd1;
    localparam [2:0] RD_DATA = 3'd2;
    localparam [2:0] RD_LINE = 3'd3;
    localparam [2:0] RD_FE   = 3'd4;

    function [1:0] choose_bank;
        input [2:0] free_mask;
        input [1:0] start_bank;
        integer offset;
        integer index;
        reg found;
        begin
            choose_bank = 2'd3;
            found = 1'b0;
            for (offset = 0; offset < 3; offset = offset + 1) begin
                index = start_bank + offset;
                if (index >= 3) index = index - 3;
                if (!found && free_mask[index]) begin
                    choose_bank = index[1:0];
                    found = 1'b1;
                end
            end
        end
    endfunction

    function [24:0] bank_base;
        input [1:0] bank;
        begin
            case (bank)
                2'd0: bank_base = 25'd0;
                2'd1: bank_base = FRAME_ADDR_SPAN;
                2'd2: bank_base = FRAME_ADDR_SPAN * 2;
                default: bank_base = 25'd0;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // RX domain: pack eight 16-bit words into each 128-bit DDR beat.
    //------------------------------------------------------------------
    reg [2:0] bank_free;
    (* ASYNC_REG = "TRUE" *) reg [2:0] bank_free_rx_meta;
    (* ASYNC_REG = "TRUE" *) reg [2:0] bank_free_rx_sync;
    (* ASYNC_REG = "TRUE" *) reg init_rx_meta;
    (* ASYNC_REG = "TRUE" *) reg init_rx_sync;
    reg rx_active;
    reg rx_drop;
    reg rx_bank_valid;
    reg [1:0] rx_bank;
    reg [1:0] rx_round_robin;
    reg [22:0] rx_seen_count;
    reg [11:0] rx_line_word_count;
    reg [11:0] rx_line_count;
    reg        rx_frame_bad;
    reg [2:0] rx_pack_count;
    reg [127:0] rx_pack_buf;
    reg ctrl_pending;
    reg [1:0] ctrl_pending_kind;
    reg [1:0] ctrl_pending_bank;
    reg rx_error_latched;

    wire pack_fifo_full;
    wire pack_fifo_empty;
    wire [131:0] pack_fifo_rdata;
    wire pack_fifo_re;
    wire ctrl_fifo_push;

    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            bank_free_rx_meta <= 3'b111;
            bank_free_rx_sync <= 3'b111;
            init_rx_meta <= 1'b0;
            init_rx_sync <= 1'b0;
        end else begin
            bank_free_rx_meta <= bank_free;
            bank_free_rx_sync <= bank_free_rx_meta;
            init_rx_meta <= ddr_init_done;
            init_rx_sync <= init_rx_meta;
        end
    end

    wire [1:0] candidate_bank = choose_bank(bank_free_rx_sync, rx_round_robin);
    wire candidate_valid = init_rx_sync && (candidate_bank != 2'd3) && !ctrl_pending;
    wire rx_in_frame = rx_active || rx_frame_start;
    wire [1:0] rx_current_bank = rx_active ? rx_bank : candidate_bank;
    wire rx_current_bank_valid = rx_active ? rx_bank_valid : candidate_valid;
    wire rx_current_drop = rx_active ? rx_drop : !candidate_valid;
    wire [2:0] rx_current_pack_count = rx_active ? rx_pack_count : 3'd0;
    wire [22:0] rx_current_seen_count = rx_active ? rx_seen_count : 23'd0;
    wire [11:0] rx_current_line_word_count = rx_active ? rx_line_word_count : 12'd0;
    wire [11:0] rx_current_line_count = rx_active ? rx_line_count : 12'd0;
    wire [127:0] rx_current_pack_buf = rx_active ? rx_pack_buf : 128'd0;
    wire rx_data_accept = rx_word_valid && rx_in_frame &&
                          rx_current_bank_valid && !rx_current_drop &&
                          !ctrl_pending &&
                          !((rx_current_pack_count == 3'd7) && pack_fifo_full);
    wire rx_data_push = rx_data_accept && (rx_current_pack_count == 3'd7);
    wire [22:0] rx_seen_after_word = rx_current_seen_count +
                                      (rx_data_accept ? 23'd1 : 23'd0);
    wire [11:0] rx_line_words_after = rx_current_line_word_count +
                                       (rx_data_accept ? 12'd1 : 12'd0);
    wire [11:0] rx_lines_after_end = rx_current_line_count +
                                      (rx_line_end ? 12'd1 : 12'd0);
    wire [11:0] rx_line_words_after_end = rx_line_end ? 12'd0 : rx_line_words_after;
    wire [2:0] rx_pack_after_word = rx_data_accept ?
                                      ((rx_current_pack_count == 3'd7) ? 3'd0 :
                                       rx_current_pack_count + 3'd1) :
                                      rx_current_pack_count;

    reg [127:0] rx_pack_next;
    always @* begin
        rx_pack_next = rx_current_pack_buf;
        rx_pack_next[rx_current_pack_count*16 +: 16] = rx_word_data;
    end

    assign ctrl_fifo_push = ctrl_pending && !pack_fifo_full;
    wire pack_fifo_we = ctrl_fifo_push || rx_data_push;
    wire [131:0] pack_fifo_wdata = ctrl_fifo_push ?
        {ctrl_pending_kind, ctrl_pending_bank, 128'd0} :
        {REC_DATA, rx_current_bank, rx_pack_next};

    csi2_ptr_fifo #(
        .DW(132),
        .AW(PACK_FIFO_AW)
    ) u_frame_pack_fifo (
        .I_wclk(rx_clk),
        .I_wrstn(rx_rst_n),
        .I_we(pack_fifo_we),
        .I_wdata(pack_fifo_wdata),
        .O_wfull(pack_fifo_full),
        .I_rclk(ddr_clk),
        .I_rrstn(ddr_rst_n),
        .I_re(pack_fifo_re),
        .O_rdata(pack_fifo_rdata),
        .O_rempty(pack_fifo_empty)
    );

    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            rx_active <= 1'b0;
            rx_drop <= 1'b0;
            rx_bank_valid <= 1'b0;
            rx_bank <= 2'd0;
            rx_round_robin <= 2'd0;
            rx_seen_count <= 23'd0;
            rx_line_word_count <= 12'd0;
            rx_line_count <= 12'd0;
            rx_frame_bad <= 1'b0;
            rx_pack_count <= 3'd0;
            rx_pack_buf <= 128'd0;
            ctrl_pending <= 1'b0;
            ctrl_pending_kind <= REC_ABORT;
            ctrl_pending_bank <= 2'd0;
            rx_error_latched <= 1'b0;
            dropped_frames <= 16'd0;
        end else begin
            if (ctrl_fifo_push)
                ctrl_pending <= 1'b0;

            if (rx_frame_start && !rx_active) begin
                rx_active <= 1'b1;
                rx_drop <= !candidate_valid;
                rx_bank_valid <= candidate_valid;
                rx_bank <= candidate_bank;
                rx_seen_count <= 23'd0;
                rx_line_word_count <= 12'd0;
                rx_line_count <= 12'd0;
                rx_frame_bad <= 1'b0;
                rx_pack_count <= 3'd0;
                rx_pack_buf <= 128'd0;
                if (candidate_valid)
                    rx_round_robin <= (candidate_bank == 2'd2) ? 2'd0 :
                                      candidate_bank + 2'd1;
                else if (dropped_frames != 16'hFFFF)
                    dropped_frames <= dropped_frames + 16'd1;
            end else if (rx_frame_start && rx_active) begin
                rx_drop <= 1'b1;
                rx_error_latched <= 1'b1;
            end

            if (rx_word_valid && rx_in_frame) begin
                if (rx_data_accept) begin
                    rx_seen_count <= rx_current_seen_count + 23'd1;
                    rx_line_word_count <= rx_current_line_word_count + 12'd1;
                    if (rx_current_pack_count == 3'd7) begin
                        rx_pack_count <= 3'd0;
                        rx_pack_buf <= 128'd0;
                    end else begin
                        rx_pack_count <= rx_current_pack_count + 3'd1;
                        rx_pack_buf <= rx_pack_next;
                    end
                end else if (rx_current_bank_valid && !rx_current_drop) begin
                    rx_drop <= 1'b1;
                    rx_error_latched <= 1'b1;
                end
            end

            if (rx_line_end && rx_in_frame) begin
                if (rx_line_words_after != LINE_WORDS16)
                    rx_frame_bad <= 1'b1;
                rx_line_word_count <= 12'd0;
                rx_line_count <= rx_current_line_count + 12'd1;
            end

            if (rx_frame_end && rx_in_frame) begin
                rx_active <= 1'b0;
                rx_drop <= 1'b0;
                rx_bank_valid <= 1'b0;
                rx_seen_count <= 23'd0;
                rx_line_word_count <= 12'd0;
                rx_line_count <= 12'd0;
                rx_frame_bad <= 1'b0;
                rx_pack_count <= 3'd0;
                rx_pack_buf <= 128'd0;
                if (rx_current_bank_valid) begin
                    ctrl_pending <= 1'b1;
                    ctrl_pending_bank <= rx_current_bank;
                    if (!rx_current_drop &&
                        !rx_frame_bad &&
                        !(rx_line_end && (rx_line_words_after != LINE_WORDS16)) &&
                        (rx_seen_after_word == FRAME_WORDS16) &&
                        (rx_pack_after_word == 3'd0) &&
                        (rx_line_words_after_end == 12'd0) &&
                        (rx_lines_after_end == FRAME_LINES))
                        ctrl_pending_kind <= REC_COMMIT;
                    else begin
                        ctrl_pending_kind <= REC_ABORT;
                        rx_error_latched <= 1'b1;
                        if (dropped_frames != 16'hFFFF)
                            dropped_frames <= dropped_frames + 16'd1;
                    end
                end
            end
        end
    end

    //------------------------------------------------------------------
    // DDR domain: write committed frames, keep only the newest pending
    // frame, then emit its RAW10 words and reconstructed line boundaries.
    //------------------------------------------------------------------
    reg [2:0] frame_ready;
    reg wr_active;
    reg [1:0] wr_bank;
    reg [22:0] wr_count;
    reg rd_active;
    reg [1:0] rd_bank;
    reg [22:0] rd_total_count;
    reg [22:0] rd_request_count;
    reg [6:0] rd_reserved_count;
    reg [11:0] rd_line_count;
    reg [2:0] rd_subword;
    reg [24:0] rd_addr;
    reg [127:0] rd_buf;
    reg rd_buf_valid;
    reg rd_last_line;
    reg [2:0] rd_state;
    reg [2:0] arb_write_count;
    reg [127:0] rd_fifo [0:(1<<RD_FIFO_AW)-1];
    reg [RD_FIFO_AW-1:0] rd_fifo_wr_ptr;
    reg [RD_FIFO_AW-1:0] rd_fifo_rd_ptr;
    reg [RD_FIFO_AW:0] rd_fifo_count;
    wire rd_fifo_push = ddr_user_rd_valid;
    wire rd_fifo_pop = rd_active && (rd_state == RD_DATA) &&
                       !rd_buf_valid && (rd_fifo_count != 0);
    wire rd_beat_consumed = out_word_valid && (rd_subword == 3'd7);
    reg ddr_error_latched;
    (* ASYNC_REG = "TRUE" *) reg rx_error_ddr_meta;
    (* ASYNC_REG = "TRUE" *) reg rx_error_ddr_sync;
    reg [1:0] selected_ready_bank;
    reg any_ready;

    always @* begin
        ready_count = {1'b0, frame_ready[0]} +
                      {1'b0, frame_ready[1]} +
                      {1'b0, frame_ready[2]};
        any_ready = |frame_ready;
        selected_ready_bank = 2'd3;
        if (frame_ready[0]) selected_ready_bank = 2'd0;
        else if (frame_ready[1]) selected_ready_bank = 2'd1;
        else if (frame_ready[2]) selected_ready_bank = 2'd2;
    end

    wire [1:0] pack_kind = pack_fifo_rdata[131:130];
    wire [1:0] pack_bank = pack_fifo_rdata[129:128];
    wire pack_is_data = !pack_fifo_empty && (pack_kind == REC_DATA);
    wire pack_is_control = !pack_fifo_empty && (pack_kind != REC_DATA);
    wire pack_write_overflow = pack_is_data && wr_active &&
                               (pack_bank == wr_bank) &&
                               (wr_count >= FRAME_PACK_WORDS);
    wire rd_start = ddr_init_done && !rd_active &&
                    (rd_state == RD_IDLE) && any_ready;
    wire read_can = ddr_init_done && rd_active && (rd_state == RD_DATA) &&
                    (rd_request_count < FRAME_PACK_WORDS) &&
                    (rd_reserved_count < ((1<<RD_FIFO_AW)-4)) &&
                    ddr_user_ready;
    wire write_can = pack_is_data && !pack_write_overflow &&
                     ddr_init_done && ddr_user_ready;
    wire do_write = write_can && (!read_can || (arb_write_count < 3'd7));
    wire do_read = read_can && !do_write;
    wire do_pop_bad_data = pack_write_overflow;

    assign pack_fifo_re = pack_is_control || do_write || do_pop_bad_data;
    assign ddr_user_wr_en = do_write;
    assign ddr_user_wr_data = pack_fifo_rdata[127:0];
    assign ddr_user_wr_addr = (!wr_active || (pack_bank != wr_bank)) ?
                              bank_base(pack_bank) :
                              bank_base(wr_bank) + (wr_count << 3);
    assign ddr_user_rd_en = do_read;
    assign ddr_user_rd_addr = rd_addr;

    assign out_frame_start = rd_active && (rd_state == RD_FS) && out_ready;
    assign out_word_valid  = rd_active && (rd_state == RD_DATA) && rd_buf_valid && out_ready;
    assign out_word_data   = rd_buf[rd_subword*16 +: 16];
    assign out_line_end    = rd_active && (rd_state == RD_LINE) && out_ready;
    assign out_frame_end   = rd_active && (rd_state == RD_FE) && out_ready;

    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if (!ddr_rst_n) begin
            frame_ready <= 3'b000;
            bank_free <= 3'b111;
            wr_active <= 1'b0;
            wr_bank <= 2'd0;
            wr_count <= 23'd0;
            rd_active <= 1'b0;
            rd_bank <= 2'd0;
            rd_total_count <= 23'd0;
            rd_request_count <= 23'd0;
            rd_reserved_count <= 7'd0;
            rd_line_count <= 12'd0;
            rd_subword <= 3'd0;
            rd_addr <= 25'd0;
            rd_buf <= 128'd0;
            rd_buf_valid <= 1'b0;
            rd_last_line <= 1'b0;
            rd_state <= RD_IDLE;
            arb_write_count <= 3'd0;
            rd_fifo_wr_ptr <= {RD_FIFO_AW{1'b0}};
            rd_fifo_rd_ptr <= {RD_FIFO_AW{1'b0}};
            rd_fifo_count <= {(RD_FIFO_AW+1){1'b0}};
            ddr_error_latched <= 1'b0;
            rx_error_ddr_meta <= 1'b0;
            rx_error_ddr_sync <= 1'b0;
            committed_frames <= 16'd0;
        end else begin
            rx_error_ddr_meta <= rx_error_latched;
            rx_error_ddr_sync <= rx_error_ddr_meta;

            if (do_write) begin
                if (!wr_active || (pack_bank != wr_bank)) begin
                    if (wr_active) begin
                        bank_free[wr_bank] <= 1'b1;
                        frame_ready[wr_bank] <= 1'b0;
                        ddr_error_latched <= 1'b1;
                    end
                    wr_active <= 1'b1;
                    wr_bank <= pack_bank;
                    wr_count <= 23'd1;
                    bank_free[pack_bank] <= 1'b0;
                    frame_ready[pack_bank] <= 1'b0;
                end else begin
                    wr_count <= wr_count + 23'd1;
                end
            end

            if (do_pop_bad_data) begin
                wr_active <= 1'b0;
                wr_count <= 23'd0;
                bank_free[wr_bank] <= 1'b1;
                frame_ready[wr_bank] <= 1'b0;
                ddr_error_latched <= 1'b1;
            end

            if (pack_is_control && pack_fifo_re) begin
                if (pack_kind == REC_COMMIT) begin
                    if (wr_active && (pack_bank == wr_bank) &&
                        (wr_count == FRAME_PACK_WORDS)) begin
                        if (frame_ready[0] && !(rd_start && selected_ready_bank == 2'd0)) begin
                            frame_ready[0] <= 1'b0;
                            bank_free[0] <= 1'b1;
                        end
                        if (frame_ready[1] && !(rd_start && selected_ready_bank == 2'd1)) begin
                            frame_ready[1] <= 1'b0;
                            bank_free[1] <= 1'b1;
                        end
                        if (frame_ready[2] && !(rd_start && selected_ready_bank == 2'd2)) begin
                            frame_ready[2] <= 1'b0;
                            bank_free[2] <= 1'b1;
                        end
                        frame_ready[pack_bank] <= 1'b1;
                        bank_free[pack_bank] <= 1'b0;
                        wr_active <= 1'b0;
                        wr_count <= 23'd0;
                        if (committed_frames != 16'hFFFF)
                            committed_frames <= committed_frames + 16'd1;
                    end else begin
                        if (wr_active && (pack_bank == wr_bank)) begin
                            bank_free[wr_bank] <= 1'b1;
                            frame_ready[wr_bank] <= 1'b0;
                        end
                        wr_active <= 1'b0;
                        wr_count <= 23'd0;
                        ddr_error_latched <= 1'b1;
                    end
                end else if (pack_kind == REC_ABORT) begin
                    if (wr_active && (pack_bank == wr_bank)) begin
                        bank_free[wr_bank] <= 1'b1;
                        frame_ready[wr_bank] <= 1'b0;
                        wr_active <= 1'b0;
                        wr_count <= 23'd0;
                    end
                    ddr_error_latched <= 1'b1;
                end
            end

            if (do_read) begin
                rd_addr <= rd_addr + 25'd8;
                rd_request_count <= rd_request_count + 23'd1;
            end

            if (rd_fifo_push) begin
                rd_fifo[rd_fifo_wr_ptr] <= ddr_user_rd_data;
                rd_fifo_wr_ptr <= rd_fifo_wr_ptr + 1'b1;
            end
            if (rd_fifo_pop) begin
                rd_buf <= rd_fifo[rd_fifo_rd_ptr];
                rd_buf_valid <= 1'b1;
                rd_fifo_rd_ptr <= rd_fifo_rd_ptr + 1'b1;
            end
            case ({rd_fifo_push, rd_fifo_pop})
                2'b10: rd_fifo_count <= rd_fifo_count + 1'b1;
                2'b01: rd_fifo_count <= rd_fifo_count - 1'b1;
                default: begin end
            endcase

            case ({do_read, rd_beat_consumed})
                2'b10: rd_reserved_count <= rd_reserved_count + 1'b1;
                2'b01: rd_reserved_count <= rd_reserved_count - 1'b1;
                default: begin end
            endcase

            if (do_write && do_read)
                arb_write_count <= 3'd0;
            else if (do_write && (arb_write_count != 3'd7))
                arb_write_count <= arb_write_count + 3'd1;
            else if (do_read)
                arb_write_count <= 3'd0;

            if (rd_start) begin
                rd_active <= 1'b1;
                rd_bank <= selected_ready_bank;
                frame_ready[selected_ready_bank] <= 1'b0;
                rd_total_count <= 23'd0;
                rd_request_count <= 23'd0;
                rd_reserved_count <= 7'd0;
                rd_line_count <= 12'd0;
                rd_subword <= 3'd0;
                rd_addr <= bank_base(selected_ready_bank);
                rd_buf_valid <= 1'b0;
                rd_fifo_wr_ptr <= {RD_FIFO_AW{1'b0}};
                rd_fifo_rd_ptr <= {RD_FIFO_AW{1'b0}};
                rd_fifo_count <= {(RD_FIFO_AW+1){1'b0}};
                rd_state <= RD_FS;
            end

            if (out_frame_start)
                rd_state <= RD_DATA;

            if (out_word_valid) begin
                if (rd_subword == 3'd7) begin
                    rd_subword <= 3'd0;
                    rd_buf_valid <= 1'b0;
                end else begin
                    rd_subword <= rd_subword + 3'd1;
                end

                if (rd_line_count == LINE_WORDS16 - 1) begin
                    rd_line_count <= 12'd0;
                    rd_last_line <= (rd_total_count == FRAME_WORDS16 - 1);
                    rd_state <= RD_LINE;
                end else begin
                    rd_line_count <= rd_line_count + 12'd1;
                end
                rd_total_count <= rd_total_count + 23'd1;
            end

            if (out_line_end)
                rd_state <= rd_last_line ? RD_FE : RD_DATA;

            if (out_frame_end) begin
                bank_free[rd_bank] <= 1'b1;
                rd_active <= 1'b0;
                rd_buf_valid <= 1'b0;
                rd_state <= RD_IDLE;
            end
        end
    end

    assign frame_busy = !ddr_init_done || wr_active || rd_active || any_ready;
    assign frame_error = rx_error_ddr_sync || ddr_error_latched;

endmodule
