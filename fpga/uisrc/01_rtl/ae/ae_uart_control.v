`timescale 1ns / 1ps

module ae_uart_control #(
    parameter integer CLK_HZ = 24000000,
    parameter integer BAUD = 115200,
    parameter [15:0] DEFAULT_VTS = 16'd2000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx,
    input  wire        local_req,
    input  wire [15:0] local_ae,
    input  wire [15:0] local_ag,
    input  wire        cam_cfg_done,
    input  wire        ae_cfg_done,
    output wire        cfg_req,
    output wire [15:0] cfg_ae,
    output wire [15:0] cfg_ag,
    output wire [15:0] cfg_vts,
    output reg         remote_active,
    output wire [15:0] packet_count,
    output wire [15:0] error_count
);

    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;

    reg [1:0] rx_sync;
    reg rx_prev;
    reg rx_busy;
    reg rx_start_check;
    reg [15:0] rx_baud_count;
    reg [3:0] rx_bit_index;
    reg [7:0] rx_shift;
    reg [7:0] rx_byte;
    reg rx_byte_valid;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            rx_sync <= 2'b11;
            rx_prev <= 1'b1;
            rx_busy <= 1'b0;
            rx_start_check <= 1'b0;
            rx_baud_count <= 16'd0;
            rx_bit_index <= 4'd0;
            rx_shift <= 8'd0;
            rx_byte <= 8'd0;
            rx_byte_valid <= 1'b0;
        end else begin
            rx_sync <= {rx_sync[0], uart_rx};
            rx_prev <= rx_sync[1];
            rx_byte_valid <= 1'b0;
            if(!rx_busy) begin
                if(rx_prev && !rx_sync[1]) begin
                    rx_busy <= 1'b1;
                    rx_start_check <= 1'b1;
                    rx_baud_count <= CLKS_PER_BIT / 2 - 1;
                    rx_bit_index <= 4'd0;
                end
            end else if(rx_baud_count != 0) begin
                rx_baud_count <= rx_baud_count - 1'b1;
            end else begin
                if(rx_start_check) begin
                    if(!rx_sync[1]) begin
                        rx_start_check <= 1'b0;
                        rx_baud_count <= CLKS_PER_BIT - 1;
                    end else begin
                        rx_busy <= 1'b0;
                        rx_start_check <= 1'b0;
                    end
                end else begin
                    rx_baud_count <= CLKS_PER_BIT - 1;
                    if(rx_bit_index < 8) begin
                        rx_shift[rx_bit_index] <= rx_sync[1];
                        rx_bit_index <= rx_bit_index + 1'b1;
                    end else begin
                        rx_busy <= 1'b0;
                        if(rx_sync[1]) begin
                            rx_byte <= rx_shift;
                            rx_byte_valid <= 1'b1;
                        end
                    end
                end
            end
        end
    end

    function is_hex;
        input [7:0] c;
        begin
            is_hex = ((c >= "0") && (c <= "9")) ||
                     ((c >= "A") && (c <= "F")) ||
                     ((c >= "a") && (c <= "f"));
        end
    endfunction

    function [3:0] hex_nibble;
        input [7:0] c;
        begin
            if((c >= "0") && (c <= "9")) hex_nibble = c - "0";
            else if((c >= "A") && (c <= "F")) hex_nibble = c - "A" + 4'd10;
            else if((c >= "a") && (c <= "f")) hex_nibble = c - "a" + 4'd10;
            else hex_nibble = 4'd0;
        end
    endfunction

    function [7:0] crc8_byte;
        input [7:0] crc_in;
        input [7:0] data;
        integer bitn;
        reg [7:0] c;
        begin
            c = crc_in ^ data;
            for(bitn = 0; bitn < 8; bitn = bitn + 1)
                c = c[7] ? {c[6:0], 1'b0} ^ 8'h07 : {c[6:0], 1'b0};
            crc8_byte = c;
        end
    endfunction

    function [7:0] crc8_fields;
        input [15:0] exposure;
        input [15:0] gain;
        input [15:0] vts;
        reg [7:0] c;
        begin
            c = 8'd0;
            c = crc8_byte(c, exposure[15:8]);
            c = crc8_byte(c, exposure[7:0]);
            c = crc8_byte(c, gain[15:8]);
            c = crc8_byte(c, gain[7:0]);
            c = crc8_byte(c, vts[15:8]);
            c = crc8_byte(c, vts[7:0]);
            crc8_fields = c;
        end
    endfunction

    reg parser_active;
    reg [4:0] parser_index;
    reg [15:0] parser_ae;
    reg [15:0] parser_ag;
    reg [15:0] parser_vts;
    reg [7:0] parser_crc;
    reg [15:0] packet_ae;
    reg [15:0] packet_ag;
    reg [15:0] packet_vts;
    reg packet_valid;
    reg [15:0] packet_count_r;
    reg [15:0] error_count_r;

    assign packet_count = packet_count_r;
    assign error_count = error_count_r;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            parser_active <= 1'b0;
            parser_index <= 5'd0;
            parser_ae <= 16'd0;
            parser_ag <= 16'd0;
            parser_vts <= 16'd0;
            parser_crc <= 8'd0;
            packet_ae <= 16'd0;
            packet_ag <= 16'd0;
            packet_vts <= DEFAULT_VTS;
            packet_valid <= 1'b0;
            packet_count_r <= 16'd0;
            error_count_r <= 16'd0;
        end else begin
            packet_valid <= 1'b0;
            if(rx_byte_valid) begin
                if(rx_byte == "@") begin
                    parser_active <= 1'b1;
                    parser_index <= 5'd0;
                    parser_ae <= 16'd0;
                    parser_ag <= 16'd0;
                    parser_vts <= 16'd0;
                    parser_crc <= 8'd0;
                end else if(parser_active) begin
                    case(parser_index)
                        0: if(rx_byte == "A") parser_index <= 1;
                           else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        1: if(rx_byte == "E") parser_index <= 2;
                           else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        2: if(rx_byte == ",") parser_index <= 3;
                           else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        3,4,5,6: if(is_hex(rx_byte)) begin
                               parser_ae <= {parser_ae[11:0], hex_nibble(rx_byte)};
                               parser_index <= parser_index + 1'b1;
                           end else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        7: if(rx_byte == ",") parser_index <= 8;
                           else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        8,9,10,11: if(is_hex(rx_byte)) begin
                               parser_ag <= {parser_ag[11:0], hex_nibble(rx_byte)};
                               parser_index <= parser_index + 1'b1;
                           end else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        12: if(rx_byte == ",") parser_index <= 13;
                            else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        13,14,15,16: if(is_hex(rx_byte)) begin
                               parser_vts <= {parser_vts[11:0], hex_nibble(rx_byte)};
                               parser_index <= parser_index + 1'b1;
                           end else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        17: if(rx_byte == "*") parser_index <= 18;
                            else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        18: if(is_hex(rx_byte)) begin
                               parser_crc[7:4] <= hex_nibble(rx_byte);
                               parser_index <= 19;
                           end else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        19: if(is_hex(rx_byte)) begin
                               parser_crc[3:0] <= hex_nibble(rx_byte);
                               parser_index <= 20;
                           end else begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                        20: begin
                            parser_active <= 1'b0;
                            parser_index <= 5'd0;
                            if(rx_byte == 8'h0A && parser_crc == crc8_fields(parser_ae, parser_ag, parser_vts) &&
                               parser_ae >= 16'd3 && parser_ag >= 16'd16 && parser_ag <= 16'd143 &&
                               parser_vts >= 16'd1980 && parser_vts <= 16'd2500 &&
                               {1'b0, parser_ae} <= (({1'b0, parser_vts} << 1) - 17'd10)) begin
                                packet_ae <= parser_ae;
                                packet_ag <= parser_ag;
                                packet_vts <= parser_vts;
                                packet_valid <= 1'b1;
                                packet_count_r <= packet_count_r + 1'b1;
                            end else begin
                                error_count_r <= error_count_r + 1'b1;
                            end
                        end
                        default: begin parser_active <= 1'b0; error_count_r <= error_count_r + 1'b1; end
                    endcase
                end
            end
        end
    end

    reg [15:0] remote_ae;
    reg [15:0] remote_ag;
    reg [15:0] remote_vts;
    reg remote_pending;
    reg remote_apply;
    reg local_reapply;

    assign cfg_req = remote_apply | local_reapply | (local_req & ~remote_active);
    assign cfg_ae = remote_active ? remote_ae : local_ae;
    assign cfg_ag = remote_active ? remote_ag : local_ag;
    assign cfg_vts = remote_active ? remote_vts : DEFAULT_VTS;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            remote_active <= 1'b0;
            remote_ae <= 16'd3000;
            remote_ag <= 16'd60;
            remote_vts <= DEFAULT_VTS;
            remote_pending <= 1'b0;
            remote_apply <= 1'b0;
            local_reapply <= 1'b0;
        end else begin
            remote_apply <= 1'b0;
            local_reapply <= 1'b0;
            if(packet_valid && (!remote_active || packet_ae != remote_ae ||
                                packet_ag != remote_ag || packet_vts != remote_vts)) begin
                remote_active <= 1'b1;
                remote_ae <= packet_ae;
                remote_ag <= packet_ag;
                remote_vts <= packet_vts;
                remote_pending <= 1'b1;
            end
            if(local_req && remote_active) begin
                remote_active <= 1'b0;
                remote_pending <= 1'b0;
                local_reapply <= 1'b1;
            end
            if(remote_pending && cam_cfg_done && ae_cfg_done) begin
                remote_apply <= 1'b1;
                remote_pending <= 1'b0;
            end
        end
    end

endmodule
