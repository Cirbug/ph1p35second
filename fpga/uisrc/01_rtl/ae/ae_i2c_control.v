`timescale 1ns / 1ps

module ae_i2c_control #(
    parameter [6:0] SLAVE_ADDRESS = 7'h2A,
    parameter [15:0] DEFAULT_VTS = 16'd2000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        scl,
    inout  wire        sda,
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

    localparam [2:0] ST_IDLE          = 3'd0;
    localparam [2:0] ST_RX            = 3'd1;
    localparam [2:0] ST_ACK_WAIT_FALL = 3'd2;
    localparam [2:0] ST_ACK_HOLD      = 3'd3;
    localparam [2:0] ST_ACK_WAIT_END  = 3'd4;
    localparam [2:0] ST_IGNORE        = 3'd5;

    reg [2:0] state;
    reg [1:0] scl_sync;
    reg [1:0] sda_sync;
    reg scl_prev;
    reg sda_prev;
    reg [2:0] bit_count;
    reg [3:0] byte_index;
    reg [7:0] rx_shift;
    reg selected;
    reg ack_pending;
    reg sda_drive_low;

    reg [7:0] rx_ae_hi;
    reg [7:0] rx_ae_lo;
    reg [7:0] rx_ag_hi;
    reg [7:0] rx_ag_lo;
    reg [7:0] rx_vts_hi;
    reg [7:0] rx_vts_lo;
    reg [15:0] packet_ae;
    reg [15:0] packet_ag;
    reg [15:0] packet_vts;
    reg packet_valid;
    reg [15:0] packet_count_r;
    reg [15:0] error_count_r;

    reg remote_pending;
    reg remote_apply;
    reg local_reapply;

    wire scl_rise = scl_sync[1] & ~scl_prev;
    wire scl_fall = ~scl_sync[1] & scl_prev;
    wire start_condition = sda_prev & ~sda_sync[1] & scl_sync[1];
    wire stop_condition = ~sda_prev & sda_sync[1] & scl_sync[1];
    wire [7:0] rx_byte_now = {rx_shift[6:0], sda_sync[1]};

    assign sda = sda_drive_low ? 1'b0 : 1'bz;
    assign packet_count = packet_count_r;
    assign error_count = error_count_r;
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
        input [15:0] ae;
        input [15:0] ag;
        input [15:0] frame_length;
        reg [7:0] c;
        begin
            c = crc8_byte(8'd0, ae[15:8]);
            c = crc8_byte(c, ae[7:0]);
            c = crc8_byte(c, ag[15:8]);
            c = crc8_byte(c, ag[7:0]);
            c = crc8_byte(c, frame_length[15:8]);
            c = crc8_byte(c, frame_length[7:0]);
            crc8_fields = c;
        end
    endfunction

    function fields_valid;
        input [15:0] ae;
        input [15:0] ag;
        input [15:0] frame_length;
        reg [16:0] max_exposure;
        begin
            max_exposure = ({1'b0, frame_length} << 1) - 17'd10;
            fields_valid = (ae >= 16'd3) &&
                           (ag >= 16'd16) && (ag <= 16'd143) &&
                           (frame_length >= 16'd1980) &&
                           (frame_length <= 16'd2500) &&
                           ({1'b0, ae} <= max_exposure);
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
            scl_prev <= 1'b1;
            sda_prev <= 1'b1;
            state <= ST_IDLE;
            bit_count <= 3'd0;
            byte_index <= 4'd0;
            rx_shift <= 8'd0;
            selected <= 1'b0;
            ack_pending <= 1'b0;
            sda_drive_low <= 1'b0;
            rx_ae_hi <= 8'd0;
            rx_ae_lo <= 8'd0;
            rx_ag_hi <= 8'd0;
            rx_ag_lo <= 8'd0;
            rx_vts_hi <= 8'd0;
            rx_vts_lo <= 8'd0;
            packet_ae <= 16'd0;
            packet_ag <= 16'd0;
            packet_vts <= DEFAULT_VTS;
            packet_valid <= 1'b0;
            packet_count_r <= 16'd0;
            error_count_r <= 16'd0;
        end else begin
            scl_sync <= {scl_sync[0], scl};
            sda_sync <= {sda_sync[0], sda};
            scl_prev <= scl_sync[1];
            sda_prev <= sda_sync[1];
            packet_valid <= 1'b0;

            if(start_condition) begin
                state <= ST_RX;
                bit_count <= 3'd0;
                byte_index <= 4'd0;
                rx_shift <= 8'd0;
                selected <= 1'b0;
                ack_pending <= 1'b0;
                sda_drive_low <= 1'b0;
            end else if(stop_condition) begin
                if(selected && byte_index != 4'd0 && byte_index != 4'd8)
                    error_count_r <= error_count_r + 1'b1;
                state <= ST_IDLE;
                bit_count <= 3'd0;
                byte_index <= 4'd0;
                selected <= 1'b0;
                ack_pending <= 1'b0;
                sda_drive_low <= 1'b0;
            end else begin
                case(state)
                    ST_IDLE: begin end
                    ST_RX: begin
                        if(scl_rise) begin
                            rx_shift <= rx_byte_now;
                            if(bit_count == 3'd7) begin
                                bit_count <= 3'd0;
                                state <= ST_ACK_WAIT_FALL;
                                if(byte_index == 4'd0) begin
                                    if(rx_byte_now[7:1] == SLAVE_ADDRESS && !rx_byte_now[0]) begin
                                        selected <= 1'b1;
                                        ack_pending <= 1'b1;
                                    end else begin
                                        selected <= 1'b0;
                                        ack_pending <= 1'b0;
                                    end
                                end else if(selected) begin
                                    case(byte_index)
                                        4'd1: begin rx_ae_hi <= rx_byte_now; ack_pending <= 1'b1; end
                                        4'd2: begin rx_ae_lo <= rx_byte_now; ack_pending <= 1'b1; end
                                        4'd3: begin rx_ag_hi <= rx_byte_now; ack_pending <= 1'b1; end
                                        4'd4: begin rx_ag_lo <= rx_byte_now; ack_pending <= 1'b1; end
                                        4'd5: begin rx_vts_hi <= rx_byte_now; ack_pending <= 1'b1; end
                                        4'd6: begin rx_vts_lo <= rx_byte_now; ack_pending <= 1'b1; end
                                        4'd7: begin
                                            if(rx_byte_now == crc8_fields(
                                                   {rx_ae_hi, rx_ae_lo},
                                                   {rx_ag_hi, rx_ag_lo},
                                                   {rx_vts_hi, rx_vts_lo}) &&
                                               fields_valid({rx_ae_hi, rx_ae_lo},
                                                            {rx_ag_hi, rx_ag_lo},
                                                            {rx_vts_hi, rx_vts_lo})) begin
                                                packet_ae <= {rx_ae_hi, rx_ae_lo};
                                                packet_ag <= {rx_ag_hi, rx_ag_lo};
                                                packet_vts <= {rx_vts_hi, rx_vts_lo};
                                                packet_valid <= 1'b1;
                                                packet_count_r <= packet_count_r + 1'b1;
                                                ack_pending <= 1'b1;
                                            end else begin
                                                error_count_r <= error_count_r + 1'b1;
                                                ack_pending <= 1'b0;
                                            end
                                        end
                                        default: begin
                                            error_count_r <= error_count_r + 1'b1;
                                            ack_pending <= 1'b0;
                                        end
                                    endcase
                                end else begin
                                    ack_pending <= 1'b0;
                                end
                            end else begin
                                bit_count <= bit_count + 1'b1;
                            end
                        end
                    end
                    ST_ACK_WAIT_FALL: begin
                        if(scl_fall) begin
                            sda_drive_low <= ack_pending;
                            state <= ST_ACK_HOLD;
                        end
                    end
                    ST_ACK_HOLD: begin
                        if(scl_rise)
                            state <= ST_ACK_WAIT_END;
                    end
                    ST_ACK_WAIT_END: begin
                        if(scl_fall) begin
                            sda_drive_low <= 1'b0;
                            if(ack_pending) begin
                                byte_index <= byte_index + 1'b1;
                                state <= ST_RX;
                            end else begin
                                state <= ST_IGNORE;
                            end
                            ack_pending <= 1'b0;
                        end
                    end
                    ST_IGNORE: begin end
                    default: state <= ST_IDLE;
                endcase
            end
        end
    end

    reg [15:0] remote_pending_ae;
    reg [15:0] remote_pending_ag;
    reg [15:0] remote_pending_vts;
    reg remote_apply_r;
    reg local_reapply_r;

    assign cfg_req = remote_apply_r | local_reapply_r | (local_req & ~remote_active);
    assign cfg_ae = remote_active ? remote_pending_ae : local_ae;
    assign cfg_ag = remote_active ? remote_pending_ag : local_ag;
    assign cfg_vts = remote_active ? remote_pending_vts : DEFAULT_VTS;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            remote_active <= 1'b0;
            remote_pending_ae <= 16'd3000;
            remote_pending_ag <= 16'd60;
            remote_pending_vts <= DEFAULT_VTS;
            remote_apply_r <= 1'b0;
            local_reapply_r <= 1'b0;
            remote_pending <= 1'b0;
        end else begin
            remote_apply_r <= 1'b0;
            local_reapply_r <= 1'b0;
            if(packet_valid && (!remote_active || packet_ae != remote_pending_ae ||
                                packet_ag != remote_pending_ag || packet_vts != remote_pending_vts)) begin
                remote_active <= 1'b1;
                remote_pending_ae <= packet_ae;
                remote_pending_ag <= packet_ag;
                remote_pending_vts <= packet_vts;
                remote_pending <= 1'b1;
            end
            if(local_req && remote_active) begin
                remote_active <= 1'b0;
                remote_pending <= 1'b0;
                local_reapply_r <= 1'b1;
            end
            if(remote_pending && cam_cfg_done && ae_cfg_done) begin
                remote_apply_r <= 1'b1;
                remote_pending <= 1'b0;
            end
        end
    end

endmodule
