`timescale 1ns / 1ps

module tb_dbg_uart_frame_stats;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    wire uart_tx;
    integer received_count = 0;
    integer bit_index;
    reg receiver_busy = 1'b0;
    reg [7:0] sample_byte;
    reg [7:0] received [0:511];

    always #5 clk = ~clk;

    dbg_uart #(.CLK_HZ(1000), .BAUD(100), .GAP_MS(1)) dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_st_cfg(1'b1), .I_st_wc(16'h0CA8), .I_st_dt(8'h2B),
        .I_st_burst(16'd0), .I_st_h0(16'd0), .I_st_h1(16'd0),
        .I_st_frame(16'd1), .I_st_rxval(16'd2), .I_st_werr(1'b0),
        .I_st_gapmax(16'd1944), .I_st_pktcnt(16'd123),
        .I_st_rx_fs(16'd1), .I_st_rx_fe(16'd1), .I_st_raw_fe(16'd1),
        .I_st_rx_rows(16'h0798), .I_st_rx_wc(16'h0CA8),
        .I_st_rx_bad_rows(16'd0), .I_st_rx_parse_err(1'b0),
        .I_st_scale_rows(16'h0438), .I_st_scale_wc(16'h0960),
        .I_st_scale_err(1'b0), .I_st_tx_packets(16'h043A),
        .I_st_tx_lines(16'h0438), .I_st_tx_bytes(32'h0027A658),
        .I_st_tx_wc(16'h0960), .I_st_tx_err(1'b0),
        .I_st_ddr_init(1'b1), .I_st_ddr_commit(16'h0001),
        .I_st_ddr_drop(16'h0000), .I_st_ddr_ready_count(2'd2),
        .I_st_ddr_rd_req(16'h1122), .I_st_ddr_rd_rsp(16'h2233),
        .I_st_cache_words(16'h3344), .I_st_cache_lines(16'h0438),
        .I_st_cache_frames(16'h0001), .I_st_relay_writes(16'h4455),
        .I_st_relay_reads(16'h5566), .I_st_relay_full(1'b0),
        .I_st_tx_words(16'h6677), .I_st_tx_long_lines(16'h0438),
        .I_st_tx_dpll_locked(1'b1), .I_st_tx_ready(1'b0),
        .I_st_ddr_busy(1'b1), .I_st_ddr_error(1'b1),
        .I_st_tx_error_live(1'b0),
        .O_uart_tx(uart_tx)
    );

    always begin
        @(negedge uart_tx);
        if (!receiver_busy && rst_n) begin
            receiver_busy = 1'b1;
            #(150);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                sample_byte[bit_index] = uart_tx;
                #(100);
            end
            received[received_count] = sample_byte;
            received_count = received_count + 1;
            receiver_busy = 1'b0;
        end
    end

    task check_byte;
        input integer index;
        input [7:0] expected;
        begin
            if (received[index] !== expected) begin
                $display("FAIL byte[%0d]=%02h expected=%02h",
                         index, received[index], expected);
                $fatal(1);
            end
        end
    endtask

    integer i;
    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        for (i = 0; i < 50000 && received_count < 320; i = i + 1)
            @(posedge clk);

        if (received_count != 320) begin
            $display("FAIL UART byte count=%0d expected=320", received_count);
            $fatal(1);
        end
        check_byte(98, 8'h30); check_byte(99, 8'h37);
        check_byte(100, 8'h39); check_byte(101, 8'h38);
        check_byte(106, 8'h30); check_byte(107, 8'h43);
        check_byte(108, 8'h41); check_byte(109, 8'h38);
        check_byte(114, 8'h30); check_byte(117, 8'h30);
        check_byte(128, 8'h30); check_byte(129, 8'h34);
        check_byte(130, 8'h33); check_byte(131, 8'h38);
        check_byte(136, 8'h30); check_byte(137, 8'h39);
        check_byte(138, 8'h36); check_byte(139, 8'h30);
        check_byte(149, 8'h30); check_byte(150, 8'h34);
        check_byte(151, 8'h33); check_byte(152, 8'h41);
        check_byte(157, 8'h30); check_byte(158, 8'h34);
        check_byte(159, 8'h33); check_byte(160, 8'h38);
        check_byte(165, 8'h30); check_byte(166, 8'h30);
        check_byte(167, 8'h32); check_byte(168, 8'h37);
        check_byte(169, 8'h41); check_byte(170, 8'h36);
        check_byte(171, 8'h35); check_byte(172, 8'h38);
        check_byte(177, 8'h30); check_byte(178, 8'h39);
        check_byte(179, 8'h36); check_byte(180, 8'h30);
        check_byte(185, 8'h30);
        check_byte(190, 8'h31);
        check_byte(195, 8'h30); check_byte(198, 8'h31);
        check_byte(203, 8'h30); check_byte(206, 8'h30);
        check_byte(211, 8'h32);
        check_byte(216, 8'h31); check_byte(219, 8'h32);
        check_byte(224, 8'h32); check_byte(227, 8'h33);
        check_byte(232, 8'h33); check_byte(235, 8'h34);
        check_byte(240, 8'h30); check_byte(243, 8'h38);
        check_byte(248, 8'h30); check_byte(251, 8'h31);
        check_byte(256, 8'h34); check_byte(259, 8'h35);
        check_byte(264, 8'h35); check_byte(267, 8'h36);
        check_byte(272, 8'h30);
        check_byte(278, 8'h36); check_byte(281, 8'h37);
        check_byte(287, 8'h30); check_byte(290, 8'h38);
        check_byte(295, 8'h31); check_byte(301, 8'h30);
        check_byte(306, 8'h31); check_byte(311, 8'h31);
        check_byte(317, 8'h30);
        check_byte(318, 8'h0D); check_byte(319, 8'h0A);
        $display("PASS UART frame and DDR/TX trace fields");
        $finish;
    end
endmodule
