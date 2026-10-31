// DS3231 real-time-clock reader on a dedicated open-drain I2C bus.
//
// The device is read at its standard 7-bit address 0x68.  The controller
// fetches seconds, minutes, hours, day, date, month and year (registers
// 0x00..0x06) once per second.  Values remain in the DS3231's BCD format so
// the MCU can display them without losing the original register contents.
module ds3231_i2c_ctrl #(
    parameter integer CLK_DIV = 16'd499,       // 50 MHz -> approximately 100 kHz
    parameter integer POLL_CYCLES = 50000000,  // one refresh per second
    parameter integer GAP_CYCLES = 1000
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire       I_set,
    input  wire [31:0] I_set_time0, I_set_time1,
    // [0] busy (includes queued request), [1] done, [2] failed,
    // [7:4] fault, [31:16] completed request sequence. Sticky until next set.
    output reg [31:0] O_set_status,
    inout  wire       IO_scl,
    inout  wire       IO_sda,
    output reg [7:0]  O_seconds,
    output reg [7:0]  O_minutes,
    output reg [7:0]  O_hours,
    output reg [7:0]  O_day,
    output reg [7:0]  O_date,
    output reg [7:0]  O_month,
    output reg [7:0]  O_year,
    output reg        O_busy,
    output reg        O_valid,
    output reg        O_error,
    output reg [3:0]  O_fault,
    output reg        O_osf,
    output reg [31:0] O_read_count
);
    localparam [2:0] ST_WAIT = 3'd0;
    localparam [2:0] ST_START = 3'd1;
    localparam [2:0] ST_XFER = 3'd2;
    localparam [2:0] ST_GAP = 3'd3;

    reg [2:0] state;
    reg [3:0] index;
    reg [7:0] pending [0:6];
    reg [1:0] rollover_retries;
    reg [31:0] poll_count;
    reg [31:0] gap_count;
    reg start;
    localparam [3:0] READ=0, CTRL_READ=1, CTRL_WRITE=2, WRITE=3,
        VERIFY=4, STATUS_READ=5, STATUS_CLEAR=6, FINAL_READ=7;
    reg [3:0] operation;
    reg set_pending, untrusted;
    reg [55:0] requested;
    reg [7:0] write_value;
    wire normal_read = operation==READ || operation==FINAL_READ;
    wire [7:0] requested_byte = requested >> (index*8);
    wire [7:0] address = normal_read ? (index==8 ? 8'h0f : index==7 ? 8'h00 : {4'd0,index}) :
        (operation==CTRL_READ || operation==CTRL_WRITE) ? 8'h0e :
        (operation==STATUS_READ || operation==STATUS_CLEAR) ? 8'h0f : {4'd0,index};
    wire done;
    wire [2:0] ack_mask;
    wire [3:0] fault;
    wire [7:0] nack_byte;
    wire [7:0] rx_data;
    wire [31:0] bus_state_unused;

    // The generic master is the same bounded-recovery engine used by VCM;
    // only the slave address and bus pins differ.
    lens_i2c_master #(
        .PERIOD_CYCLES (CLK_DIV + 1),
        .SLAVE_ADDR    (7'h68)
    ) u_ds3231_bus (
        .clk       (I_clk),
        .rst_n     (I_rst_n),
        .start     (start),
        .read_reg  (operation!=WRITE && operation!=CTRL_WRITE && operation!=STATUS_CLEAR),
        .reg_addr  (address),
        .reg_value (operation==WRITE ? requested_byte : write_value),
        .scl       (IO_scl),
        .sda       (IO_sda),
        .done      (done),
        .ack_mask  (ack_mask),
        .fault     (fault),
        .nack_byte (nack_byte),
        .rx_data   (rx_data),
        .bus_state (bus_state_unused)
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state       <= ST_WAIT;
            index       <= 3'd0;
            poll_count  <= 32'd1000;
            gap_count   <= 32'd0;
            start       <= 1'b0;
            operation <= READ; set_pending <= 0; untrusted <= 0;
            requested <= 0; write_value <= 0; O_set_status <= 0;
            O_seconds   <= 8'd0;
            O_minutes   <= 8'd0;
            O_hours     <= 8'd0;
            O_day       <= 8'd0;
            O_date      <= 8'd0;
            O_month     <= 8'd0;
            O_year      <= 8'd0;
            O_busy      <= 1'b0;
            O_valid     <= 1'b0;
            O_error     <= 1'b0;
            O_fault     <= 4'd0;
            O_osf       <= 1'b1;
            rollover_retries <= 0;
            O_read_count <= 32'd0;
        end else begin
            start <= 1'b0;
            // Snapshot software staging registers; a second command cannot
            // replace a pending/active transaction. Finish any current read.
            if(I_set && !O_set_status[0]) begin
                requested <= {I_set_time1,I_set_time0[23:0]};
                set_pending <= 1; untrusted <= 1; O_valid <= 0;
                O_set_status[15:0] <= 16'h0001;
            end
            case (state)
                ST_WAIT: begin
                    O_busy <= 1'b0;
                    if(set_pending) begin
                        set_pending <= 0; operation <= CTRL_READ;
                        index <= 0; state <= ST_START; O_busy <= 1;
                    end else if (poll_count == 0) begin
                        operation <= READ;
                        index <= 3'd0;
                        rollover_retries <= 0;
                        state <= ST_START;
                        O_busy <= 1'b1;
                    end else begin
                        poll_count <= poll_count - 1'b1;
                    end
                end
                ST_START: begin
                    // A one-cycle request starts one register read.
                    start <= 1'b1;
                    state <= ST_XFER;
                end
                ST_XFER: begin
                    if (done) begin
                        if (fault != 0 || ack_mask != 3'b111) begin
                            O_error <= 1'b1;
                            O_valid <= 1'b0;
                            O_fault <= (fault != 0) ? fault : 4'd1;
                            O_busy <= 1'b0;
                            poll_count <= POLL_CYCLES;
                            state <= ST_WAIT;
                            if(operation!=READ) begin
                                O_set_status <= {O_set_status[31:16]+16'd1,8'd0,
                                    (fault!=0 ? fault : 4'd1),4'b0110};
                                operation <= READ;
                            end
                        end else if(!normal_read) begin
                            // A seconds write resets the countdown chain. All
                            // seven writes and exact readback finish well within
                            // one second, before OSF is permitted to be cleared.
                            gap_count <= GAP_CYCLES; state <= ST_GAP;
                            case(operation)
                                CTRL_READ: begin write_value<=rx_data & 8'h7f; operation<=CTRL_WRITE; end
                                CTRL_WRITE: begin operation<=WRITE; index<=0; end
                                WRITE: if(index==6) begin operation<=VERIFY; index<=0; end
                                       else index<=index+1'b1;
                                VERIFY: begin
                                    if(rx_data!=requested_byte) begin
                                        O_valid<=0; O_error<=1; O_fault<=5;
                                        O_busy<=0; poll_count<=POLL_CYCLES; state<=ST_WAIT;
                                        O_set_status<={O_set_status[31:16]+16'd1,16'h0056};
                                        operation<=READ;
                                    end else if(index==6) operation<=STATUS_READ;
                                    else index<=index+1'b1;
                                end
                                STATUS_READ: begin
                                    // Preserve EN32kHz and both W0C alarm flags.
                                    write_value <= (rx_data & 8'h08) | 8'h03;
                                    operation <= STATUS_CLEAR;
                                end
                                STATUS_CLEAR: begin
                                    operation<=FINAL_READ; index<=0; rollover_retries<=0;
                                end
                            endcase
                        end else if (index == 7 && rx_data != pending[0]) begin
                            // Every START refreshes the DS3231 shadow buffer.
                            // Repeat the entire scan if a second rolled over.
                            if (rollover_retries == 2) begin
                                O_valid <= 0; O_error <= 1; O_fault <= 4;
                                O_busy <= 0; poll_count <= POLL_CYCLES;
                                state <= ST_WAIT;
                                if(operation==FINAL_READ) begin
                                    O_set_status<={O_set_status[31:16]+16'd1,16'h0046};
                                    operation<=READ;
                                end
                            end else begin
                                rollover_retries <= rollover_retries + 1'b1;
                                index <= 0; gap_count <= GAP_CYCLES;
                                state <= ST_GAP;
                            end
                        end else begin
                            if (index < 7) pending[index] <= rx_data;
                            if (index == 8) begin
                                // Publish one coherent snapshot only after all
                                // transfers succeed. Failed setting keeps the
                                // visible clock invalid until a successful set.
                                O_seconds <= pending[0]; O_minutes <= pending[1];
                                O_hours <= pending[2]; O_day <= pending[3];
                                O_date <= pending[4]; O_month <= pending[5];
                                O_year <= pending[6]; O_osf <= rx_data[7];
                                O_valid <= !rx_data[7] && (!untrusted || operation==FINAL_READ) && !I_set;
                                O_error <= 1'b0;
                                O_fault <= 4'd0;
                                O_read_count <= O_read_count + 1'b1;
                                O_busy <= 1'b0;
                                poll_count <= POLL_CYCLES;
                                state <= ST_WAIT;
                                if(operation==FINAL_READ) begin
                                    untrusted<=rx_data[7];
                                    O_error<=rx_data[7]; O_fault<=rx_data[7] ? 6 : 0;
                                    O_set_status<={O_set_status[31:16]+16'd1,
                                        (rx_data[7] ? 16'h0066 : 16'h0002)};
                                    operation<=READ;
                                end
                            end else begin
                                index <= index + 1'b1;
                                gap_count <= GAP_CYCLES;
                                state <= ST_GAP;
                            end
                        end
                    end
                end
                ST_GAP: begin
                    if (gap_count == 0)
                        state <= ST_START;
                    else
                        gap_count <= gap_count - 1'b1;
                end
                default: state <= ST_WAIT;
            endcase
        end
    end
endmodule
