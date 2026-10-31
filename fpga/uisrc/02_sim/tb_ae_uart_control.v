`timescale 1ns / 1ps

module tb_ae_uart_control;
    localparam CLK_HZ = 2400;
    localparam BAUD = 100;
    localparam BIT_CYCLES = 24;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg uart_rx = 1'b1;
    reg local_req = 1'b0;
    reg cam_cfg_done = 1'b0;
    reg ae_cfg_done = 1'b0;
    reg [15:0] local_ae = 16'd3000;
    reg [15:0] local_ag = 16'd60;
    wire cfg_req;
    wire [15:0] cfg_ae, cfg_ag, cfg_vts;
    wire remote_active;
    wire [15:0] packet_count, error_count;
    integer req_pulses = 0;

    always #5 clk = ~clk;
    always @(posedge clk)
        if (rst_n && cfg_req) req_pulses <= req_pulses + 1;
    ae_uart_control #(
        .CLK_HZ(CLK_HZ), .BAUD(BAUD), .DEFAULT_VTS(16'd2000)
    ) dut (
        .clk(clk), .rst_n(rst_n), .uart_rx(uart_rx),
        .local_req(local_req), .local_ae(local_ae), .local_ag(local_ag),
        .cam_cfg_done(cam_cfg_done), .ae_cfg_done(ae_cfg_done),
        .cfg_req(cfg_req), .cfg_ae(cfg_ae), .cfg_ag(cfg_ag), .cfg_vts(cfg_vts),
        .remote_active(remote_active), .packet_count(packet_count),
        .error_count(error_count)
    );

    reg [8:0] reg_index = 0;
    wire [31:0] reg_data;
    wire [8:0] reg_size;
    uics500regAE ae_table (
        .REG_INDEX(reg_index), .REG_DATA(reg_data), .REG_SIZE(reg_size),
        .I_AE(16'd4000), .I_AG(16'h0CA8), .I_VTS(16'd2000)
    );

    function [7:0] crc8_byte;
        input [7:0] crc_in;
        input [7:0] data;
        integer bit_index;
        reg [7:0] crc;
        begin
            crc = crc_in ^ data;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                crc = crc[7] ? {crc[6:0], 1'b0} ^ 8'h07 : {crc[6:0], 1'b0};
            crc8_byte = crc;
        end
    endfunction

    function [7:0] hex_char;
        input [3:0] nibble;
        begin
            hex_char = (nibble < 10) ? (8'h30 + nibble) : (8'h41 + nibble - 10);
        end
    endfunction

    task send_uart_byte;
        input [7:0] value;
        integer bit_index;
        begin
            @(negedge clk);
            uart_rx = 1'b0;
            repeat (BIT_CYCLES) @(negedge clk);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                uart_rx = value[bit_index];
                repeat (BIT_CYCLES) @(negedge clk);
            end
            uart_rx = 1'b1;
            repeat (BIT_CYCLES) @(negedge clk);
        end
    endtask

    task send_hex16;
        input [15:0] value;
        begin
            send_uart_byte(hex_char(value[15:12]));
            send_uart_byte(hex_char(value[11:8]));
            send_uart_byte(hex_char(value[7:4]));
            send_uart_byte(hex_char(value[3:0]));
        end
    endtask

    task send_frame;
        input [15:0] exposure;
        input [15:0] gain;
        input [15:0] vts;
        input corrupt_crc;
        reg [7:0] crc;
        reg [7:0] tx_crc;
        begin
            crc = 0;
            crc = crc8_byte(crc, exposure[15:8]);
            crc = crc8_byte(crc, exposure[7:0]);
            crc = crc8_byte(crc, gain[15:8]);
            crc = crc8_byte(crc, gain[7:0]);
            crc = crc8_byte(crc, vts[15:8]);
            crc = crc8_byte(crc, vts[7:0]);
            tx_crc = crc ^ (corrupt_crc ? 8'h01 : 8'h00);

            send_uart_byte("@"); send_uart_byte("A"); send_uart_byte("E");
            send_uart_byte(","); send_hex16(exposure); send_uart_byte(",");
            send_hex16(gain); send_uart_byte(","); send_hex16(vts);
            send_uart_byte("*");
            send_uart_byte(hex_char(tx_crc[7:4]));
            send_uart_byte(hex_char(tx_crc[3:0]));
            send_uart_byte(8'h0A);
        end
    endtask

    task check_ae_table;
        begin
            if (reg_size !== 9'd7) $fatal(1, "AE table size=%0d expected=7", reg_size);
            reg_index = 0; #1; if (reg_data[23:0] !== 24'h320e07) $fatal(1, "AE table VTS high");
            reg_index = 1; #1; if (reg_data[23:0] !== 24'h320fd0) $fatal(1, "AE table VTS low");
            reg_index = 2; #1; if (reg_data[23:0] !== 24'h3e0000) $fatal(1, "AE table exposure high");
            reg_index = 3; #1; if (reg_data[23:0] !== 24'h3e01fa) $fatal(1, "AE table exposure middle");
            reg_index = 4; #1; if (reg_data[23:0] !== 24'h3e0200) $fatal(1, "AE table exposure low");
            reg_index = 5; #1; if (reg_data[23:0] !== 24'h3e080c) $fatal(1, "AE table gain high");
            reg_index = 6; #1; if (reg_data[23:0] !== 24'h3e09a8) $fatal(1, "AE table gain low");
        end
    endtask

    initial begin
        check_ae_table;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        send_frame(16'd3900, 16'd60, 16'd2000, 1'b0);
        repeat (5) @(posedge clk);
        if (packet_count !== 16'd1 || !remote_active) begin
            $display("debug rx=%b sync=%b prev=%b busy=%b baud=%0d bit=%0d packets=%h errors=%h parser=%b index=%0d byte=%h ae=%h ag=%h vts=%h crc=%h",
                     uart_rx, dut.rx_sync, dut.rx_prev, dut.rx_busy,
                     dut.rx_baud_count, dut.rx_bit_index,
                     packet_count, error_count, dut.parser_active, dut.parser_index,
                     dut.rx_byte, dut.parser_ae, dut.parser_ag, dut.parser_vts, dut.parser_crc);
            $fatal(1, "valid frame was not accepted");
        end
        if (cfg_ae !== 16'd3900 || cfg_ag !== 16'd60 || cfg_vts !== 16'd2000)
            $fatal(1, "remote fields mismatch");
        if (cfg_req) $fatal(1, "remote apply before camera tables completed");

        cam_cfg_done = 1'b1;
        ae_cfg_done = 1'b1;
        wait (req_pulses == 1);
        send_frame(16'd3900, 16'd60, 16'd2000, 1'b0);
        repeat (10) @(posedge clk);
        if (packet_count !== 16'd2 || req_pulses != 1)
            $fatal(1, "keepalive caused an unnecessary I2C update");

        send_frame(16'd4100, 16'd70, 16'd2100, 1'b1);
        repeat (5) @(posedge clk);
        if (error_count !== 16'd1 || cfg_ae !== 16'd3900)
            $fatal(1, "bad CRC changed camera settings");

        send_frame(16'd5000, 16'd70, 16'd2100, 1'b0);
        repeat (5) @(posedge clk);
        if (error_count !== 16'd2 || packet_count !== 16'd2)
            $fatal(1, "out-of-range exposure was not rejected");

        local_ae = 16'd3200;
        local_ag = 16'd80;
        @(negedge clk); local_req = 1'b1;
        @(negedge clk); local_req = 1'b0;
        repeat (5) @(posedge clk);
        if (remote_active || cfg_ae !== 16'd3200 || cfg_ag !== 16'd80)
            $fatal(1, "local controls did not take over");

        send_frame(16'd4200, 16'd75, 16'd2200, 1'b0);
        repeat (5) @(posedge clk);
        if (!remote_active || cfg_ae !== 16'd4200 || cfg_ag !== 16'd75 || cfg_vts !== 16'd2200)
            $fatal(1, "remote controls did not resume");

        $display("PASS AE UART framing, CRC/range checks, mode switching, and SC500 register table");
        $finish;
    end
endmodule
