`timescale 1ns / 1ps
`define CNN_PH1P35

module cnn_frame_digit_detector #(
    parameter [7:0] DARK_THRESHOLD = 8'd128,
    parameter [9:0] ROW_ACTIVE_MIN = 10'd2,
    parameter [8:0] MIN_DIGIT_HEIGHT = 9'd18,
    parameter [15:0] MIN_DIGIT_AREA = 16'd18,
    parameter [4:0] MAX_DIGIT_ASPECT_TENTHS = 5'd20,
    parameter integer MAX_BANDS = 8,
    parameter integer MAX_DIGITS = 16
)(
    input  wire       capture_clk,
    input  wire       infer_clk,
    input  wire       rst,
    input  wire       frame_start,
    input  wire       pixel_valid,
    input  wire [7:0] pixel_data,
    input  wire       frame_complete,
    output reg        digit_done,
    output reg  [3:0] digit_class,
    output reg  [9:0] digit_x,
    output reg  [8:0] digit_y,
    output reg  [9:0] digit_w,
    output reg  [8:0] digit_h,
    output reg        frame_done,
    output reg  [4:0] frame_digit_count,
    output reg        busy,
    output wire       model_check_done,
    output wire [31:0] model_weight_crc32,
    output reg  [31:0] digit_patch_crc32,
    output reg  [31:0] digit_conv1_signature,
    output reg  [31:0] digit_pool1_signature,
    output reg  [31:0] digit_conv2_signature,
    output reg  [31:0] digit_pool2_signature,
    output reg  [31:0] digit_conv3_signature,
    output reg  [31:0] digit_pool3_signature,
    output reg  [31:0] digit_conv1_input_signature,
    output reg  [31:0] digit_conv1_weight_signature,
    output reg  [31:0] digit_conv1_product_signature,
    output reg  [31:0] digit_conv1_accumulator_signature,
    output reg  [31:0] digit_conv1_raw_signature,
    output reg  [255:0] digit_conv1_channel_signatures,
    output reg  [255:0] digit_pool1_channel_signatures,
    output reg  [7:0] digit_conv1_first_weight,
    output reg  [7:0] digit_conv1_first_activation,
    output reg  [15:0] digit_conv1_first_product
);
    localparam integer FRAME_WIDTH = 320;
    localparam integer FRAME_HEIGHT = 180;
    localparam integer FRAME_PIXELS = FRAME_WIDTH * FRAME_HEIGHT;

    localparam [5:0] ST_IDLE             = 6'd0;
    localparam [5:0] ST_ROW_ISSUE        = 6'd1;
    localparam [5:0] ST_ROW_CHECK        = 6'd2;
    localparam [5:0] ST_ROW_FINALIZE     = 6'd3;
    localparam [5:0] ST_BAND_INIT        = 6'd4;
    localparam [5:0] ST_BAND_CLEAR       = 6'd5;
    localparam [5:0] ST_BAND_SCAN_ISSUE  = 6'd6;
    localparam [5:0] ST_BAND_SCAN_ACCUM  = 6'd7;
    localparam [5:0] ST_COL_SCAN_INIT    = 6'd8;
    localparam [5:0] ST_COL_ISSUE        = 6'd9;
    localparam [5:0] ST_COL_CHECK        = 6'd10;
    localparam [5:0] ST_EXTENT_INIT      = 6'd11;
    localparam [5:0] ST_EXTENT_ISSUE     = 6'd12;
    localparam [5:0] ST_EXTENT_ACCUM     = 6'd13;
    localparam [5:0] ST_EXTENT_FINISH    = 6'd14;
    localparam [5:0] ST_BAND_NEXT        = 6'd15;
    localparam [5:0] ST_CANDIDATE_NEXT   = 6'd16;
    localparam [5:0] ST_PATCH_CLEAR      = 6'd17;
    localparam [5:0] ST_SCALE_W_INIT     = 6'd18;
    localparam [5:0] ST_SCALE_W_RUN      = 6'd19;
    localparam [5:0] ST_SCALE_H_INIT     = 6'd20;
    localparam [5:0] ST_SCALE_H_RUN      = 6'd21;
    localparam [5:0] ST_PATCH_ROW_INIT   = 6'd22;
    localparam [5:0] ST_PATCH_COL_INIT   = 6'd23;
    localparam [5:0] ST_PATCH_ISSUE      = 6'd24;
    localparam [5:0] ST_PATCH_CAPTURE    = 6'd25;
    localparam [5:0] ST_X_ADJUST         = 6'd26;
    localparam [5:0] ST_Y_ADJUST         = 6'd27;
    localparam [5:0] ST_CNN_START        = 6'd28;
    localparam [5:0] ST_CNN_SEND         = 6'd29;
    localparam [5:0] ST_CNN_WAIT         = 6'd30;
    localparam [5:0] ST_ROW_COUNT_CLEAR  = 6'd31;
    localparam [5:0] ST_SPLIT_INIT       = 6'd32;
    localparam [5:0] ST_SPLIT_SCAN       = 6'd33;
    localparam [5:0] ST_SPLIT_FINISH     = 6'd34;

    reg [7:0] patch_mem [0:783];

    reg [9:0] capture_x;
    reg [8:0] capture_y;
    reg [15:0] capture_address;
    reg frame_capture_enabled;
    reg frame_toggle;
    reg detector_busy_meta;
    reg detector_busy_sync;
    reg frame_toggle_meta;
    reg frame_toggle_sync;
    reg frame_toggle_seen;

    reg [8:0] row_read_index;
    reg [8:0] row_ink_accumulator;
    reg [8:0] row_scan_x;
    reg row_band_open;
    reg [8:0] row_band_start;
    reg [8:0] row_band_last_active;
    reg [3:0] row_band_count;
    reg [8:0] band_top [0:MAX_BANDS-1];
    reg [8:0] band_bottom [0:MAX_BANDS-1];
    reg [3:0] band_index;

    reg [9:0] clear_index;
    reg col_active [0:FRAME_WIDTH-1];
    reg col_active_read_data;
    reg [8:0] scan_y;
    reg [9:0] scan_x;
    reg [15:0] mask_read_address;
    reg mask_read_enable;
    reg [15:0] mask_read_address_q;
    wire mask_read_data;
    wire mask_write_enable = pixel_valid &&
        ((frame_start && !detector_busy_sync) ||
         (!frame_start && frame_capture_enabled));
    wire [15:0] mask_write_address = frame_start ? 16'd0 : capture_address;
    wire mask_write_data = (pixel_data < DARK_THRESHOLD);

    reg [9:0] column_index;
    reg segment_open;
    reg [9:0] segment_start;
    reg [9:0] segment_end;
    reg extent_resume_band;
    reg [9:0] extent_x;
    reg [8:0] extent_y;
    reg [8:0] extent_min_y;
    reg [8:0] extent_max_y;
    reg [15:0] extent_area;
    // A row occupancy bitmap separates vertically disconnected strokes.
    reg row_has_ink [0:FRAME_HEIGHT-1];
    reg [8:0] split_y;
    reg [8:0] split_start;
    reg [8:0] split_last;
    reg [15:0] split_area;
    reg split_open;

    reg [4:0] candidate_count;
    reg [4:0] candidate_index;
    reg [9:0] candidate_x [0:MAX_DIGITS-1];
    reg [8:0] candidate_y [0:MAX_DIGITS-1];
    reg [9:0] candidate_w [0:MAX_DIGITS-1];
    reg [8:0] candidate_h [0:MAX_DIGITS-1];
    reg [9:0] active_x;
    reg [8:0] active_y;
    reg [9:0] active_w;
    reg [8:0] active_h;

    reg [9:0] patch_clear_index;
    reg [12:0] scale_work;
    reg [6:0] scale_divisor;
    reg [4:0] scale_quotient;
    reg [4:0] scaled_w;
    reg [4:0] scaled_h;
    reg [4:0] patch_row;
    reg [4:0] patch_col;
    reg [7:0] source_x;
    reg [7:0] source_y;
    reg [11:0] x_error;
    reg [11:0] y_error;
    reg [9:0] send_count;
    reg [31:0] patch_crc_accumulator;
    reg [31:0] active_patch_crc32;
    reg [5:0] state;

    wire infer_start = (state == ST_CNN_START);
    wire infer_pixel_valid = (state == ST_CNN_SEND);
    wire [7:0] infer_pixel_data = patch_mem[send_count];
    wire infer_busy;
    wire infer_done;
    wire [3:0] infer_class;
    wire infer_weight_check_done;
    wire [31:0] infer_weight_crc32;
    wire [31:0] infer_conv1_signature;
    wire [31:0] infer_pool1_signature;
    wire [31:0] infer_conv2_signature;
    wire [31:0] infer_pool2_signature;
    wire [31:0] infer_conv3_signature;
    wire [31:0] infer_pool3_signature;
    wire [31:0] infer_conv1_input_signature;
    wire [31:0] infer_conv1_weight_signature;
    wire [31:0] infer_conv1_product_signature;
    wire [31:0] infer_conv1_accumulator_signature;
    wire [31:0] infer_conv1_raw_signature;
    wire [255:0] infer_conv1_channel_signatures;
    wire [255:0] infer_pool1_channel_signatures;
    wire [7:0] infer_conv1_first_weight;
    wire [7:0] infer_conv1_first_activation;
    wire [15:0] infer_conv1_first_product;
    wire [9:0] candidate_width = segment_end - segment_start + 10'd1;
    wire [8:0] measured_height = extent_max_y - extent_min_y + 9'd1;
    wire [15:0] width_aspect = candidate_width * 16'd10;
    wire [15:0] height_aspect = measured_height * 16'd18;
    wire [6:0] active_size = (active_w > active_h) ? active_w[6:0] : active_h[6:0];
    wire [12:0] width_dividend = active_w * 13'd20 + {6'd0, active_size[6:1]};
    wire [12:0] height_dividend = active_h * 13'd20 + {6'd0, active_size[6:1]};
    wire [4:0] normalized_x = 5'd4 + ((5'd20 - scaled_w) >> 1) + patch_col;
    wire [4:0] normalized_y = 5'd4 + ((5'd20 - scaled_h) >> 1) + patch_row;
    wire [9:0] patch_write_address = normalized_y * 10'd28 + normalized_x;

    function [31:0] crc32_byte;
        input [31:0] crc;
        input [7:0] data;
        reg [31:0] value;
        integer bit_index;
        begin
            value = crc ^ {24'd0, data};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                value = value[0] ? ((value >> 1) ^ 32'hedb88320) : (value >> 1);
            crc32_byte = value;
        end
    endfunction

    function [15:0] frame_address;
        input [8:0] y;
        input [9:0] x;
        reg [16:0] row_base;
        begin
            row_base = ({8'd0, y} << 8) + ({8'd0, y} << 6);
            frame_address = row_base[15:0] + x;
        end
    endfunction

`ifdef CNN_SIM
    reg simulation_mask [0:FRAME_PIXELS-1];
    reg simulation_mask_read_data;
    assign mask_read_data = simulation_mask_read_data;

    always @(posedge capture_clk) begin
        if (mask_write_enable)
            simulation_mask[mask_write_address] <= mask_write_data;
    end

    always @(posedge infer_clk) begin
        if (mask_read_enable)
            simulation_mask_read_data <= simulation_mask[mask_read_address];
    end
`else
`ifdef CNN_PH1P35
    wire [3:0] mask_bank_read_data;
    wire [3:0] mask_bank_write_enable;
    wire [3:0] mask_bank_read_enable;
    wire [1:0] mask_write_bank = mask_write_address[15:14];
    wire [1:0] mask_read_bank = mask_read_address[15:14];
    wire [1:0] mask_read_bank_q = mask_read_address_q[15:14];
    assign mask_bank_write_enable[0] = mask_write_enable && (mask_write_bank == 2'd0);
    assign mask_bank_write_enable[1] = mask_write_enable && (mask_write_bank == 2'd1);
    assign mask_bank_write_enable[2] = mask_write_enable && (mask_write_bank == 2'd2);
    assign mask_bank_write_enable[3] = mask_write_enable && (mask_write_bank == 2'd3);
    assign mask_bank_read_enable[0] = mask_read_enable && (mask_read_bank == 2'd0);
    assign mask_bank_read_enable[1] = mask_read_enable && (mask_read_bank == 2'd1);
    assign mask_bank_read_enable[2] = mask_read_enable && (mask_read_bank == 2'd2);
    assign mask_bank_read_enable[3] = mask_read_enable && (mask_read_bank == 2'd3);
    assign mask_read_data = mask_bank_read_data[mask_read_bank_q];

    genvar mask_bank;
    generate for (mask_bank = 0; mask_bank < 4; mask_bank = mask_bank + 1) begin : g_mask_bank
        PH1P_LOGIC_ERAM #(
            .DATA_WIDTH_A(1), .ADDR_WIDTH_A(14), .DATA_DEPTH_A(16384),
            .DATA_WIDTH_B(1), .ADDR_WIDTH_B(14), .DATA_DEPTH_B(16384),
            .MODE("PDPW"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
            .IMPLEMENT("20K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
            .INIT_FILE("NONE"), .FILL_ALL("NONE")
        ) u_mask (
            .dia(mask_write_data), .dib(1'b0),
            .addra(mask_write_address[13:0]), .addrb(mask_read_address[13:0]),
            .cea(mask_bank_write_enable[mask_bank]), .ceb(mask_bank_read_enable[mask_bank]),
            .ocea(1'b0), .oceb(1'b0), .clka(capture_clk), .clkb(infer_clk),
            .wea(mask_bank_write_enable[mask_bank]), .web(1'b0), .bea(1'b0), .beb(1'b0),
            .rsta(1'b0), .rstb(1'b0), .doa(), .dob(mask_bank_read_data[mask_bank])
        );
    end endgenerate
`else
    wire mask_32k_read_data;
    wire [3:0] mask_9k_read_data;
    wire [3:0] mask_9k_write_enable;
    wire [3:0] mask_9k_read_enable;
    wire [1:0] mask_9k_write_bank = mask_write_address[14:13];
    wire [1:0] mask_9k_read_bank = mask_read_address[14:13];
    wire [1:0] mask_9k_read_bank_q = mask_read_address_q[14:13];
    assign mask_9k_write_enable[0] = mask_write_enable && mask_write_address[15] && mask_9k_write_bank == 2'd0;
    assign mask_9k_write_enable[1] = mask_write_enable && mask_write_address[15] && mask_9k_write_bank == 2'd1;
    assign mask_9k_write_enable[2] = mask_write_enable && mask_write_address[15] && mask_9k_write_bank == 2'd2;
    assign mask_9k_write_enable[3] = mask_write_enable && mask_write_address[15] && mask_9k_write_bank == 2'd3;
    assign mask_9k_read_enable[0] = mask_read_enable && mask_read_address[15] && mask_9k_read_bank == 2'd0;
    assign mask_9k_read_enable[1] = mask_read_enable && mask_read_address[15] && mask_9k_read_bank == 2'd1;
    assign mask_9k_read_enable[2] = mask_read_enable && mask_read_address[15] && mask_9k_read_bank == 2'd2;
    assign mask_9k_read_enable[3] = mask_read_enable && mask_read_address[15] && mask_9k_read_bank == 2'd3;
    assign mask_read_data = mask_read_address_q[15] ?
        mask_9k_read_data[mask_9k_read_bank_q] : mask_32k_read_data;

    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(1), .ADDR_WIDTH_A(15), .DATA_DEPTH_A(32768),
        .DATA_WIDTH_B(1), .ADDR_WIDTH_B(15), .DATA_DEPTH_B(32768),
        .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .IMPLEMENT("32K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("NONE"), .FILL_ALL("NONE")
    ) u_frame_mask_32k (
        .dia(mask_write_data), .dib(1'b0),
        .addra(mask_write_address[14:0]), .addrb(mask_read_address[14:0]),
        .cea(mask_write_enable && !mask_write_address[15]),
        .ceb(mask_read_enable && !mask_read_address[15]),
        .ocea(1'b0), .oceb(1'b0), .clka(capture_clk), .clkb(infer_clk),
        .wea(mask_write_enable && !mask_write_address[15]), .web(1'b0), .bea(1'b1), .beb(1'b1),
        .rsta(1'b0), .rstb(1'b0), .doa(), .dob(mask_32k_read_data)
    );

    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(1), .ADDR_WIDTH_A(13), .DATA_DEPTH_A(8192),
        .DATA_WIDTH_B(1), .ADDR_WIDTH_B(13), .DATA_DEPTH_B(8192),
        .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .IMPLEMENT("9K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("NONE"), .FILL_ALL("NONE")
    ) u_frame_mask_9k_bank0 (
        .dia(mask_write_data), .dib(1'b0),
        .addra(mask_write_address[12:0]), .addrb(mask_read_address[12:0]),
        .cea(mask_9k_write_enable[0]), .ceb(mask_9k_read_enable[0]),
        .ocea(1'b0), .oceb(1'b0), .clka(capture_clk), .clkb(infer_clk),
        .wea(mask_9k_write_enable[0]), .web(1'b0), .bea(1'b1), .beb(1'b1),
        .rsta(1'b0), .rstb(1'b0), .doa(), .dob(mask_9k_read_data[0])
    );

    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(1), .ADDR_WIDTH_A(13), .DATA_DEPTH_A(8192),
        .DATA_WIDTH_B(1), .ADDR_WIDTH_B(13), .DATA_DEPTH_B(8192),
        .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .IMPLEMENT("9K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("NONE"), .FILL_ALL("NONE")
    ) u_frame_mask_9k_bank1 (
        .dia(mask_write_data), .dib(1'b0),
        .addra(mask_write_address[12:0]), .addrb(mask_read_address[12:0]),
        .cea(mask_9k_write_enable[1]), .ceb(mask_9k_read_enable[1]),
        .ocea(1'b0), .oceb(1'b0), .clka(capture_clk), .clkb(infer_clk),
        .wea(mask_9k_write_enable[1]), .web(1'b0), .bea(1'b1), .beb(1'b1),
        .rsta(1'b0), .rstb(1'b0), .doa(), .dob(mask_9k_read_data[1])
    );

    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(1), .ADDR_WIDTH_A(13), .DATA_DEPTH_A(8192),
        .DATA_WIDTH_B(1), .ADDR_WIDTH_B(13), .DATA_DEPTH_B(8192),
        .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .IMPLEMENT("9K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("NONE"), .FILL_ALL("NONE")
    ) u_frame_mask_9k_bank2 (
        .dia(mask_write_data), .dib(1'b0),
        .addra(mask_write_address[12:0]), .addrb(mask_read_address[12:0]),
        .cea(mask_9k_write_enable[2]), .ceb(mask_9k_read_enable[2]),
        .ocea(1'b0), .oceb(1'b0), .clka(capture_clk), .clkb(infer_clk),
        .wea(mask_9k_write_enable[2]), .web(1'b0), .bea(1'b1), .beb(1'b1),
        .rsta(1'b0), .rstb(1'b0), .doa(), .dob(mask_9k_read_data[2])
    );

    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(1), .ADDR_WIDTH_A(13), .DATA_DEPTH_A(8192),
        .DATA_WIDTH_B(1), .ADDR_WIDTH_B(13), .DATA_DEPTH_B(8192),
        .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .IMPLEMENT("9K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("NONE"), .FILL_ALL("NONE")
    ) u_frame_mask_9k_bank3 (
        .dia(mask_write_data), .dib(1'b0),
        .addra(mask_write_address[12:0]), .addrb(mask_read_address[12:0]),
        .cea(mask_9k_write_enable[3]), .ceb(mask_9k_read_enable[3]),
        .ocea(1'b0), .oceb(1'b0), .clka(capture_clk), .clkb(infer_clk),
        .wea(mask_9k_write_enable[3]), .web(1'b0), .bea(1'b1), .beb(1'b1),
        .rsta(1'b0), .rstb(1'b0), .doa(), .dob(mask_9k_read_data[3])
    );
`endif
`endif

    always @(posedge infer_clk or posedge rst) begin
        if (rst)
            mask_read_address_q <= 16'd0;
        else if (mask_read_enable)
            mask_read_address_q <= mask_read_address;
    end

    cnn_digit_infer u_infer (
        .clk(infer_clk),
        .rst_n(~rst),
        .start(infer_start),
        .pixel_valid(infer_pixel_valid),
        .pixel_data(infer_pixel_data),
        .busy(infer_busy),
        .done(infer_done),
        .class_id(infer_class),
        .weight_check_done(infer_weight_check_done),
        .weight_crc32(infer_weight_crc32),
        .conv1_signature(infer_conv1_signature),
        .pool1_signature(infer_pool1_signature),
        .conv2_signature(infer_conv2_signature),
        .pool2_signature(infer_pool2_signature),
        .conv3_signature(infer_conv3_signature),
        .pool3_signature(infer_pool3_signature),
        .conv1_input_signature(infer_conv1_input_signature),
        .conv1_weight_signature(infer_conv1_weight_signature),
        .conv1_product_signature(infer_conv1_product_signature),
        .conv1_accumulator_signature(infer_conv1_accumulator_signature),
        .conv1_raw_signature(infer_conv1_raw_signature),
        .conv1_channel_signatures(infer_conv1_channel_signatures),
        .pool1_channel_signatures(infer_pool1_channel_signatures),
        .conv1_first_weight(infer_conv1_first_weight),
        .conv1_first_activation(infer_conv1_first_activation),
        .conv1_first_product(infer_conv1_first_product)
    );

    assign model_check_done = infer_weight_check_done;
    assign model_weight_crc32 = infer_weight_crc32;

    always @(posedge infer_clk) begin
        col_active_read_data <= col_active[column_index];
        if (state == ST_BAND_CLEAR)
            col_active[clear_index] <= 1'b0;
        else if (state == ST_BAND_SCAN_ACCUM && mask_read_data)
            col_active[scan_x] <= 1'b1;
    end

    always @* begin
        mask_read_enable = 1'b0;
        mask_read_address = 16'd0;
        case (state)
            ST_ROW_ISSUE: begin
                mask_read_enable = 1'b1;
                mask_read_address = frame_address(row_read_index, {1'b0, row_scan_x});
            end
            ST_BAND_SCAN_ISSUE: begin
                mask_read_enable = 1'b1;
                mask_read_address = frame_address(scan_y, scan_x);
            end
            ST_EXTENT_ISSUE: begin
                mask_read_enable = 1'b1;
                mask_read_address = frame_address(extent_y, extent_x);
            end
            ST_PATCH_ISSUE: begin
                mask_read_enable = 1'b1;
                mask_read_address = frame_address(active_y + source_y,
                                                  active_x + source_x);
            end
            default: begin end
        endcase
    end

    always @(posedge capture_clk or posedge rst) begin
        if (rst) begin
            capture_x <= 10'd0;
            capture_y <= 9'd0;
            capture_address <= 16'd0;
            frame_capture_enabled <= 1'b0;
            frame_toggle <= 1'b0;
            detector_busy_meta <= 1'b0;
            detector_busy_sync <= 1'b0;
        end else begin
            detector_busy_meta <= busy;
            detector_busy_sync <= detector_busy_meta;

            if (frame_start) begin
                capture_x <= pixel_valid ? 10'd1 : 10'd0;
                capture_y <= 9'd0;
                capture_address <= pixel_valid ? 16'd1 : 16'd0;
                frame_capture_enabled <= !detector_busy_sync;
            end else if (pixel_valid) begin
                if (capture_address == FRAME_PIXELS - 1)
                    capture_address <= 16'd0;
                else
                    capture_address <= capture_address + 1'b1;

                if (capture_x == FRAME_WIDTH - 1) begin
                    capture_x <= 10'd0;
                    if (capture_y == FRAME_HEIGHT - 1)
                        capture_y <= 9'd0;
                    else
                        capture_y <= capture_y + 1'b1;
                end else begin
                    capture_x <= capture_x + 1'b1;
                end
            end

            if (frame_complete) begin
                if (frame_capture_enabled && !detector_busy_sync)
                    frame_toggle <= ~frame_toggle;
                frame_capture_enabled <= 1'b0;
            end
        end
    end

    always @(posedge infer_clk or posedge rst) begin
        if (rst) begin
            frame_toggle_meta <= 1'b0;
            frame_toggle_sync <= 1'b0;
            frame_toggle_seen <= 1'b0;
            row_read_index <= 9'd0;
            row_ink_accumulator <= 9'd0;
            row_scan_x <= 9'd0;
            row_band_open <= 1'b0;
            row_band_start <= 9'd0;
            row_band_last_active <= 9'd0;
            row_band_count <= 4'd0;
            band_index <= 4'd0;
            clear_index <= 10'd0;
            scan_y <= 9'd0;
            scan_x <= 10'd0;
            column_index <= 10'd0;
            segment_open <= 1'b0;
            segment_start <= 10'd0;
            segment_end <= 10'd0;
            extent_resume_band <= 1'b0;
            extent_x <= 10'd0;
            extent_y <= 9'd0;
            extent_min_y <= 9'd180;
            extent_max_y <= 9'd0;
            extent_area <= 16'd0;
            split_y <= 9'd0;
            split_start <= 9'd0;
            split_last <= 9'd0;
            split_area <= 16'd0;
            split_open <= 1'b0;
            candidate_count <= 5'd0;
            candidate_index <= 5'd0;
            active_x <= 10'd0;
            active_y <= 9'd0;
            active_w <= 10'd0;
            active_h <= 9'd0;
            patch_clear_index <= 10'd0;
            scale_work <= 13'd0;
            scale_divisor <= 7'd1;
            scale_quotient <= 5'd0;
            scaled_w <= 5'd1;
            scaled_h <= 5'd1;
            patch_row <= 5'd0;
            patch_col <= 5'd0;
            source_x <= 8'd0;
            source_y <= 8'd0;
            x_error <= 12'd0;
            y_error <= 12'd0;
            send_count <= 10'd0;
            patch_crc_accumulator <= 32'hffffffff;
            active_patch_crc32 <= 32'd0;
            state <= ST_IDLE;
            digit_done <= 1'b0;
            digit_class <= 4'd0;
            digit_x <= 10'd0;
            digit_y <= 9'd0;
            digit_w <= 10'd0;
            digit_h <= 9'd0;
            digit_patch_crc32 <= 32'd0;
            digit_conv1_signature <= 32'd0;
            digit_pool1_signature <= 32'd0;
            digit_conv2_signature <= 32'd0;
            digit_pool2_signature <= 32'd0;
            digit_conv3_signature <= 32'd0;
            digit_pool3_signature <= 32'd0;
            digit_conv1_input_signature <= 32'd0;
            digit_conv1_weight_signature <= 32'd0;
            digit_conv1_product_signature <= 32'd0;
            digit_conv1_accumulator_signature <= 32'd0;
            digit_conv1_raw_signature <= 32'd0;
            digit_conv1_channel_signatures <= 256'd0;
            digit_pool1_channel_signatures <= 256'd0;
            digit_conv1_first_weight <= 8'd0;
            digit_conv1_first_activation <= 8'd0;
            digit_conv1_first_product <= 16'd0;
            frame_done <= 1'b0;
            frame_digit_count <= 5'd0;
            busy <= 1'b0;
        end else begin
            frame_toggle_meta <= frame_toggle;
            frame_toggle_sync <= frame_toggle_meta;
            digit_done <= 1'b0;
            frame_done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (frame_toggle_sync != frame_toggle_seen) begin
                        frame_toggle_seen <= frame_toggle_sync;
                        row_read_index <= 9'd0;
                        row_ink_accumulator <= 9'd0;
                        row_scan_x <= 9'd0;
                        row_band_open <= 1'b0;
                        row_band_count <= 4'd0;
                        candidate_count <= 5'd0;
                        busy <= 1'b1;
                        state <= ST_ROW_ISSUE;
                    end
                end

                ST_ROW_ISSUE: state <= ST_ROW_CHECK;

                ST_ROW_CHECK: begin
                    if (mask_read_data)
                        row_ink_accumulator <= row_ink_accumulator + 1'b1;

                    if (row_scan_x == FRAME_WIDTH - 1) begin
                        if ((row_ink_accumulator + mask_read_data) >= ROW_ACTIVE_MIN) begin
                            if (!row_band_open) begin
                                row_band_open <= 1'b1;
                                row_band_start <= row_read_index;
                                row_band_last_active <= row_read_index;
                            end else if (row_read_index - row_band_last_active <= 9'd1) begin
                                row_band_last_active <= row_read_index;
                            end else begin
                                if (row_band_last_active - row_band_start + 1'b1 >= MIN_DIGIT_HEIGHT &&
                                    row_band_count < MAX_BANDS) begin
                                    band_top[row_band_count] <= row_band_start;
                                    band_bottom[row_band_count] <= row_band_last_active;
                                    row_band_count <= row_band_count + 1'b1;
                                end
                                row_band_start <= row_read_index;
                                row_band_last_active <= row_read_index;
                                row_band_open <= 1'b1;
                            end
                        end else if (row_band_open &&
                                     row_read_index - row_band_last_active > 9'd1) begin
                            if (row_band_last_active - row_band_start + 1'b1 >= MIN_DIGIT_HEIGHT &&
                                row_band_count < MAX_BANDS) begin
                                band_top[row_band_count] <= row_band_start;
                                band_bottom[row_band_count] <= row_band_last_active;
                                row_band_count <= row_band_count + 1'b1;
                            end
                            row_band_open <= 1'b0;
                        end

                        if (row_read_index == FRAME_HEIGHT - 1)
                            state <= ST_ROW_FINALIZE;
                        else begin
                            row_read_index <= row_read_index + 1'b1;
                            row_scan_x <= 9'd0;
                            row_ink_accumulator <= 9'd0;
                            state <= ST_ROW_ISSUE;
                        end
                    end else begin
                        row_scan_x <= row_scan_x + 1'b1;
                        state <= ST_ROW_ISSUE;
                    end
                end

                ST_ROW_FINALIZE: begin
                    if (row_band_open &&
                        row_band_last_active - row_band_start + 1'b1 >= MIN_DIGIT_HEIGHT &&
                        row_band_count < MAX_BANDS) begin
                        band_top[row_band_count] <= row_band_start;
                        band_bottom[row_band_count] <= row_band_last_active;
                        row_band_count <= row_band_count + 1'b1;
                    end
                    band_index <= 4'd0;
                    state <= ST_BAND_INIT;
                end

                ST_BAND_INIT: begin
                    if (band_index >= row_band_count) begin
                        candidate_index <= 5'd0;
                        state <= ST_CANDIDATE_NEXT;
                    end else begin
                        clear_index <= 10'd0;
                        state <= ST_BAND_CLEAR;
                    end
                end

                ST_BAND_CLEAR: begin
                    if (clear_index == FRAME_WIDTH - 1) begin
                        scan_y <= band_top[band_index];
                        scan_x <= 10'd0;
                        state <= ST_BAND_SCAN_ISSUE;
                    end else begin
                        clear_index <= clear_index + 1'b1;
                    end
                end

                ST_BAND_SCAN_ISSUE: state <= ST_BAND_SCAN_ACCUM;

                ST_BAND_SCAN_ACCUM: begin
                    if (scan_x == FRAME_WIDTH - 1) begin
                        scan_x <= 10'd0;
                        if (scan_y == band_bottom[band_index])
                            state <= ST_COL_SCAN_INIT;
                        else begin
                            scan_y <= scan_y + 1'b1;
                            state <= ST_BAND_SCAN_ISSUE;
                        end
                    end else begin
                        scan_x <= scan_x + 1'b1;
                        state <= ST_BAND_SCAN_ISSUE;
                    end
                end

                ST_COL_SCAN_INIT: begin
                    column_index <= 10'd0;
                    segment_open <= 1'b0;
                    state <= ST_COL_ISSUE;
                end

                ST_COL_ISSUE: state <= ST_COL_CHECK;

                ST_COL_CHECK: begin
                    if (col_active_read_data) begin
                        if (!segment_open)
                            segment_start <= column_index;
                        segment_end <= column_index;
                        segment_open <= 1'b1;
                        if (column_index == FRAME_WIDTH - 1) begin
                            extent_resume_band <= 1'b1;
                            state <= ST_EXTENT_INIT;
                        end else begin
                            column_index <= column_index + 1'b1;
                            state <= ST_COL_ISSUE;
                        end
                    end else if (segment_open) begin
                        segment_end <= column_index - 1'b1;
                        extent_resume_band <= 1'b0;
                        state <= ST_EXTENT_INIT;
                    end else if (column_index == FRAME_WIDTH - 1) begin
                        state <= ST_BAND_NEXT;
                    end else begin
                        column_index <= column_index + 1'b1;
                        state <= ST_COL_ISSUE;
                    end
                end

                ST_EXTENT_INIT: begin
                    clear_index <= 10'd0;
                    state <= ST_ROW_COUNT_CLEAR;
                end

                ST_ROW_COUNT_CLEAR: begin
                    row_has_ink[clear_index] <= 1'b0;
                    if (clear_index == FRAME_HEIGHT - 1) begin
                        extent_x <= segment_start;
                        extent_y <= band_top[band_index];
                        extent_min_y <= 9'd180;
                        extent_max_y <= 9'd0;
                        extent_area <= 16'd0;
                        state <= ST_EXTENT_ISSUE;
                    end else begin
                        clear_index <= clear_index + 1'b1;
                    end
                end

                ST_EXTENT_ISSUE: state <= ST_EXTENT_ACCUM;

                ST_EXTENT_ACCUM: begin
                    if (mask_read_data) begin
                        if (extent_y < extent_min_y)
                            extent_min_y <= extent_y;
                        if (extent_y > extent_max_y)
                            extent_max_y <= extent_y;
                        if (extent_area != 16'hffff)
                            extent_area <= extent_area + 1'b1;
                        row_has_ink[extent_y] <= 1'b1;
                    end

                    if (extent_x == segment_end) begin
                        extent_x <= segment_start;
                        if (extent_y == band_bottom[band_index])
                            state <= ST_EXTENT_FINISH;
                        else begin
                            extent_y <= extent_y + 1'b1;
                            state <= ST_EXTENT_ISSUE;
                        end
                    end else begin
                        extent_x <= extent_x + 1'b1;
                        state <= ST_EXTENT_ISSUE;
                    end
                end

                ST_EXTENT_FINISH: begin
                    split_y <= 9'd0;
                    split_start <= 9'd0;
                    split_last <= 9'd0;
                    split_area <= 16'd0;
                    split_open <= 1'b0;
                    state <= ST_SPLIT_INIT;
                end

                ST_SPLIT_INIT: begin
                    split_y <= 9'd0;
                    state <= ST_SPLIT_SCAN;
                end

                ST_SPLIT_SCAN: begin
                    if (row_has_ink[split_y]) begin
                        if (!split_open) begin
                            split_open <= 1'b1;
                            split_start <= split_y;
                            split_last <= split_y;
                            split_area <= 16'd1;
                        end else begin
                            split_last <= split_y;
                            if (split_area != 16'hffff)
                                split_area <= split_area + 1'b1;
                        end
                    end else if (split_open && split_y - split_last > 9'd1) begin
                        if (split_last - split_start + 1'b1 >= MIN_DIGIT_HEIGHT &&
                            candidate_width >= 10'd3 &&
                            candidate_width <= 10'd96 &&
                            split_last - split_start + 1'b1 <= 9'd96 &&
                            split_area >= MIN_DIGIT_AREA &&
                            candidate_width * 16'd10 <= (split_last - split_start + 1'b1) * MAX_DIGIT_ASPECT_TENTHS &&
                            candidate_count < MAX_DIGITS) begin
                            candidate_x[candidate_count] <= segment_start;
                            candidate_y[candidate_count] <= split_start;
                            candidate_w[candidate_count] <= candidate_width;
                            candidate_h[candidate_count] <= split_last - split_start + 1'b1;
                            candidate_count <= candidate_count + 1'b1;
                        end
                        split_open <= 1'b0;
                        split_area <= 16'd0;
                    end

                    if (split_y == FRAME_HEIGHT - 1)
                        state <= ST_SPLIT_FINISH;
                    else
                        split_y <= split_y + 1'b1;
                end

                ST_SPLIT_FINISH: begin
                    if (split_open &&
                        split_last - split_start + 1'b1 >= MIN_DIGIT_HEIGHT &&
                        candidate_width >= 10'd3 &&
                        candidate_width <= 10'd96 &&
                        split_last - split_start + 1'b1 <= 9'd96 &&
                        split_area >= MIN_DIGIT_AREA &&
                        candidate_width * 16'd10 <= (split_last - split_start + 1'b1) * MAX_DIGIT_ASPECT_TENTHS &&
                        candidate_count < MAX_DIGITS) begin
                        candidate_x[candidate_count] <= segment_start;
                        candidate_y[candidate_count] <= split_start;
                        candidate_w[candidate_count] <= candidate_width;
                        candidate_h[candidate_count] <= split_last - split_start + 1'b1;
                        candidate_count <= candidate_count + 1'b1;
                    end
                    if (extent_resume_band) begin
                        segment_open <= 1'b0;
                        state <= ST_BAND_NEXT;
                    end else begin
                        segment_open <= 1'b0;
                        state <= ST_COL_ISSUE;
                    end
                end

                ST_BAND_NEXT: begin
                    band_index <= band_index + 1'b1;
                    state <= ST_BAND_INIT;
                end

                ST_CANDIDATE_NEXT: begin
                    if (candidate_index >= candidate_count) begin
                        frame_digit_count <= candidate_count;
                        frame_done <= 1'b1;
                        busy <= 1'b0;
                        state <= ST_IDLE;
                    end else begin
                        active_x <= candidate_x[candidate_index];
                        active_y <= candidate_y[candidate_index];
                        active_w <= candidate_w[candidate_index];
                        active_h <= candidate_h[candidate_index];
                        patch_clear_index <= 10'd0;
                        state <= ST_PATCH_CLEAR;
                    end
                end

                ST_PATCH_CLEAR: begin
                    patch_mem[patch_clear_index] <= 8'd0;
                    if (patch_clear_index == 10'd783)
                        state <= ST_SCALE_W_INIT;
                    else
                        patch_clear_index <= patch_clear_index + 1'b1;
                end

                ST_SCALE_W_INIT: begin
                    scale_work <= width_dividend;
                    scale_divisor <= active_size;
                    scale_quotient <= 5'd0;
                    state <= ST_SCALE_W_RUN;
                end

                ST_SCALE_W_RUN: begin
                    if (scale_work >= scale_divisor) begin
                        scale_work <= scale_work - scale_divisor;
                        scale_quotient <= scale_quotient + 1'b1;
                    end else begin
                        scaled_w <= (scale_quotient == 0) ? 5'd1 : scale_quotient;
                        state <= ST_SCALE_H_INIT;
                    end
                end

                ST_SCALE_H_INIT: begin
                    scale_work <= height_dividend;
                    scale_divisor <= active_size;
                    scale_quotient <= 5'd0;
                    state <= ST_SCALE_H_RUN;
                end

                ST_SCALE_H_RUN: begin
                    if (scale_work >= scale_divisor) begin
                        scale_work <= scale_work - scale_divisor;
                        scale_quotient <= scale_quotient + 1'b1;
                    end else begin
                        scaled_h <= (scale_quotient == 0) ? 5'd1 : scale_quotient;
                        state <= ST_PATCH_ROW_INIT;
                    end
                end

                ST_PATCH_ROW_INIT: begin
                    patch_row <= 5'd0;
                    source_y <= 8'd0;
                    y_error <= 12'd0;
                    state <= ST_PATCH_COL_INIT;
                end

                ST_PATCH_COL_INIT: begin
                    patch_col <= 5'd0;
                    source_x <= 8'd0;
                    x_error <= 12'd0;
                    state <= ST_PATCH_ISSUE;
                end

                ST_PATCH_ISSUE: state <= ST_PATCH_CAPTURE;

                ST_PATCH_CAPTURE: begin
                    if (mask_read_data)
                        patch_mem[patch_write_address] <= 8'hff;
                    x_error <= x_error + active_w;
                    state <= ST_X_ADJUST;
                end

                ST_X_ADJUST: begin
                    if (x_error >= scaled_w) begin
                        x_error <= x_error - scaled_w;
                        source_x <= source_x + 1'b1;
                    end else if (patch_col + 1'b1 < scaled_w) begin
                        patch_col <= patch_col + 1'b1;
                        state <= ST_PATCH_ISSUE;
                    end else begin
                        y_error <= y_error + active_h;
                        state <= ST_Y_ADJUST;
                    end
                end

                ST_Y_ADJUST: begin
                    if (y_error >= scaled_h) begin
                        y_error <= y_error - scaled_h;
                        source_y <= source_y + 1'b1;
                    end else if (patch_row + 1'b1 < scaled_h) begin
                        patch_row <= patch_row + 1'b1;
                        state <= ST_PATCH_COL_INIT;
                    end else begin
                        state <= ST_CNN_START;
                    end
                end

                ST_CNN_START: begin
                    send_count <= 10'd0;
                    patch_crc_accumulator <= 32'hffffffff;
                    state <= ST_CNN_SEND;
                end

                ST_CNN_SEND: begin
                    patch_crc_accumulator <= crc32_byte(patch_crc_accumulator, infer_pixel_data);
                    if (send_count == 10'd783)
                        active_patch_crc32 <= ~crc32_byte(patch_crc_accumulator, infer_pixel_data);
                    if (send_count == 10'd783)
                        state <= ST_CNN_WAIT;
                    else
                        send_count <= send_count + 1'b1;
                end

                ST_CNN_WAIT: begin
                    if (infer_done) begin
                        digit_class <= infer_class;
                        digit_x <= active_x;
                        digit_y <= active_y;
                        digit_w <= active_w;
                        digit_h <= active_h;
                        digit_patch_crc32 <= active_patch_crc32;
                        digit_conv1_signature <= infer_conv1_signature;
                        digit_pool1_signature <= infer_pool1_signature;
                        digit_conv2_signature <= infer_conv2_signature;
                        digit_pool2_signature <= infer_pool2_signature;
                        digit_conv3_signature <= infer_conv3_signature;
                        digit_pool3_signature <= infer_pool3_signature;
                        digit_conv1_input_signature <= infer_conv1_input_signature;
                        digit_conv1_weight_signature <= infer_conv1_weight_signature;
                        digit_conv1_product_signature <= infer_conv1_product_signature;
                        digit_conv1_accumulator_signature <= infer_conv1_accumulator_signature;
                        digit_conv1_raw_signature <= infer_conv1_raw_signature;
                        digit_conv1_channel_signatures <= infer_conv1_channel_signatures;
                        digit_pool1_channel_signatures <= infer_pool1_channel_signatures;
                        digit_conv1_first_weight <= infer_conv1_first_weight;
                        digit_conv1_first_activation <= infer_conv1_first_activation;
                        digit_conv1_first_product <= infer_conv1_first_product;
                        digit_done <= 1'b1;
                        candidate_index <= candidate_index + 1'b1;
                        state <= ST_CANDIDATE_NEXT;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
