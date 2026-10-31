`timescale 1ns / 1ps

module cnn_digit_infer (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire pixel_valid,
    input wire [7:0] pixel_data,
    output reg busy,
    output reg done,
    output reg [3:0] class_id,
    output reg weight_check_done,
    output reg [31:0] weight_crc32,
    output reg [31:0] conv1_signature,
    output reg [31:0] pool1_signature,
    output reg [31:0] conv2_signature,
    output reg [31:0] pool2_signature,
    output reg [31:0] conv3_signature,
    output reg [31:0] pool3_signature,
    output reg [31:0] conv1_input_signature,
    output reg [31:0] conv1_weight_signature,
    output reg [31:0] conv1_product_signature,
    output reg [31:0] conv1_accumulator_signature,
    output reg [31:0] conv1_raw_signature,
      output reg [255:0] conv1_channel_signatures,
    output reg [255:0] pool1_channel_signatures,
    output reg [7:0] conv1_first_weight,
    output reg [7:0] conv1_first_activation,
    output reg [15:0] conv1_first_product
);

    localparam [4:0] ST_IDLE         = 5'd0;
    localparam [4:0] ST_LOAD         = 5'd1;
    localparam [4:0] ST_CONV_ISSUE   = 5'd2;
    localparam [4:0] ST_CONV_ACCUM   = 5'd3;
    localparam [4:0] ST_CONV_REQUANT = 5'd4;
    localparam [4:0] ST_CONV_STORE   = 5'd5;
    localparam [4:0] ST_POOL_ISSUE   = 5'd6;
    localparam [4:0] ST_POOL_CAPTURE = 5'd7;
    localparam [4:0] ST_POOL_WRITE   = 5'd8;
    localparam [4:0] ST_GAP_ISSUE    = 5'd9;
    localparam [4:0] ST_GAP_ACCUM    = 5'd10;
    localparam [4:0] ST_FC_START     = 5'd11;
    localparam [4:0] ST_FC_ISSUE     = 5'd12;
    localparam [4:0] ST_FC_ACCUM     = 5'd13;
    localparam [4:0] ST_FC_SUM       = 5'd14;
    localparam [4:0] ST_FC_COMPARE   = 5'd15;
    localparam [4:0] ST_CONV_PRODUCT = 5'd16;
    localparam [4:0] ST_CONV_BIAS    = 5'd17;
    localparam [4:0] ST_CONV_COORD   = 5'd18;
    localparam [4:0] ST_CONV_ADDRESS = 5'd19;
    localparam [4:0] ST_CONV_OPERAND = 5'd20;
    localparam [4:0] ST_FC_PRODUCT   = 5'd21;
    localparam [4:0] ST_POOL_ADDRESS = 5'd22;
    localparam [4:0] ST_WEIGHT_SCAN_ISSUE = 5'd23;
    localparam [4:0] ST_WEIGHT_SCAN_CAPTURE = 5'd24;

    reg [4:0] state;
    reg input_bank;
    reg [1:0] layer_index;
    reg [9:0] load_count;
    reg [4:0] output_channel;
    reg [4:0] input_channel;
    reg [4:0] output_x;
    reg [4:0] output_y;
    reg [4:0] conv_row_coordinate;
    reg [4:0] conv_col_coordinate;
    reg [12:0] conv_read_address_register;
    reg [11:0] conv_weight_address_register;
    reg [12:0] pool_read_address_register;
    wire [12:0] conv_feature_read_address;
    wire [12:0] pool_feature_read_address;
    reg [1:0] kernel_x;
    reg [1:0] kernel_y;
    reg tap_in_bounds;
    reg [31:0] accumulator;
    reg signed [31:0] raw_value;
    reg signed [15:0] conv_product_register;
    reg signed [7:0] conv_weight_register;
    reg [7:0] conv_activation_register;
    reg [7:0] activated_value;
    reg conv1_first_sample_seen;

    reg [4:0] pool_channel;
    reg [3:0] pool_x;
    reg [3:0] pool_y;
    reg [1:0] pool_sample;
    reg [7:0] pool_max;

    reg [4:0] gap_channel;
    reg [3:0] gap_position;
    reg [11:0] gap_accumulator;
    reg [11:0] features [0:15];

    reg [3:0] fc_class;
    reg [4:0] fc_channel;
    reg signed [7:0] fc_weight_register;
    reg [11:0] fc_feature_register;
    reg signed [31:0] best_score;
    reg [3:0] best_class;
    reg signed [31:0] class_score;
    reg signed [19:0] fc_product_register;

    reg [12:0] bank0_read_addr;
    reg [12:0] bank1_read_addr;
    reg bank0_read_enable;
    reg bank1_read_enable;
    reg [12:0] bank0_write_addr;
    reg [12:0] bank1_write_addr;
    reg [7:0] bank0_write_data;
    reg [7:0] bank1_write_data;
    reg bank0_write_enable;
    reg bank1_write_enable;

    reg [11:0] weight_address;
    reg weight_read_enable;
    wire [7:0] weight_data;
    reg [11:0] weight_scan_address;
    reg [31:0] weight_crc_accumulator;
    reg [31:0] conv_signature_accumulator;
    reg [31:0] pool_signature_accumulator;
    reg [31:0] conv1_input_accumulator;
    reg [31:0] conv1_weight_accumulator;
    reg [31:0] conv1_product_accumulator;
    reg [31:0] conv1_accumulator_trace;
    reg [31:0] conv1_raw_accumulator;
    wire [7:0] bank0_read_data;
    wire [7:0] bank1_read_data;
    wire [7:0] activation_read_data;
    wire [3:0] conv1_shift;
    wire [3:0] conv2_shift;
    wire [3:0] conv3_shift;
    wire signed [31:0] bias_value;

    reg [1:0] params_layer;
    reg [4:0] params_channel;
    wire [7:0] conv_activation_unsigned;
    wire [7:0] conv_weight_magnitude;
    wire [15:0] conv_product_magnitude;
    wire signed [19:0] fc_weight_extended;
    wire signed [19:0] fc_feature_extended;

    integer input_size;
    integer pool_size;
    reg signed [5:0] address_row;
    reg signed [5:0] address_col;
    reg [12:0] address_value;
    integer weight_value;
    integer output_channel_value;
    integer input_channel_value;
    integer fc_class_value;
    integer fc_channel_value;
    integer kernel_x_value;
    integer kernel_y_value;
    integer gap_channel_value;

    wire signed [15:0] conv_product;
    wire signed [19:0] fc_product;
    wire signed [31:0] conv_accumulator_signed;
    wire signed [31:0] fc_accumulator_signed;

    assign activation_read_data = (state == ST_CONV_OPERAND) ?
                                  (input_bank ? bank1_read_data : bank0_read_data) :
                                  (input_bank ? bank0_read_data : bank1_read_data);
    assign conv_activation_unsigned = tap_in_bounds ? conv_activation_register : 8'd0;
    assign fc_weight_extended = {{12{fc_weight_register[7]}}, fc_weight_register};
    assign fc_feature_extended = {8'd0, fc_feature_register};
    assign conv_weight_magnitude = conv_weight_register[7] ?
                                   (~conv_weight_register + 8'd1) : conv_weight_register;
    assign conv_product_magnitude = conv_weight_magnitude * conv_activation_unsigned;
    assign conv_product = conv_weight_register[7] ?
                          -$signed(conv_product_magnitude) : $signed(conv_product_magnitude);
    assign fc_product = fc_weight_extended * fc_feature_extended;
    assign conv_accumulator_signed = $signed(accumulator) +
                                     {{16{conv_product_register[15]}}, conv_product_register};
    assign fc_accumulator_signed = $signed(accumulator) + {{12{fc_product_register[19]}}, fc_product_register};

    cnn_digit_params u_params (
        .layer(params_layer),
        .channel(params_channel),
        .bias_value(bias_value),
        .conv1_shift(conv1_shift),
        .conv2_shift(conv2_shift),
        .conv3_shift(conv3_shift)
    );

    cnn_digit_weight_rom u_weights (
        .clk(clk),
        .read_enable(weight_read_enable),
        .address(weight_address),
        .data(weight_data)
    );

    cnn_digit_activation_ram u_bank0 (
        .clk(clk),
        .read_enable(bank0_read_enable),
        .read_address(bank0_read_addr),
        .read_data(bank0_read_data),
        .write_enable(bank0_write_enable),
        .write_address(bank0_write_addr),
        .write_data(bank0_write_data)
    );

    cnn_digit_activation_ram u_bank1 (
        .clk(clk),
        .read_enable(bank1_read_enable),
        .read_address(bank1_read_addr),
        .read_data(bank1_read_data),
        .write_enable(bank1_write_enable),
        .write_address(bank1_write_addr),
        .write_data(bank1_write_data)
    );

    function [7:0] requantize_relu;
        input signed [31:0] value;
        input [3:0] shift;
        reg signed [31:0] rounded_value;
        begin
            if (value <= 0) begin
                requantize_relu = 8'd0;
            end else begin
                rounded_value = (value + (32'sd1 <<< (shift - 1'b1))) >>> shift;
                if (rounded_value > 255)
                    requantize_relu = 8'd255;
                else
                    requantize_relu = rounded_value[7:0];
            end
        end
    endfunction

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

    function [31:0] signature_step;
        input [31:0] previous;
        input [7:0] sample;
        begin
            signature_step = {previous[26:0], previous[31:27]} ^ {24'd0, sample};
        end
    endfunction

    function [31:0] signature_word_step;
        input [31:0] previous;
        input [31:0] sample;
        begin
            signature_word_step = {previous[26:0], previous[31:27]} ^ sample;
        end
    endfunction

    function [12:0] feature_address;
        input [4:0] channel;
        input [4:0] row;
        input [4:0] col;
        input [1:0] layer;
        reg [12:0] channel_offset;
        reg [12:0] row_offset;
        reg [12:0] channel_value;
        reg [12:0] row_value;
        begin
            channel_value = {8'd0, channel};
            row_value = {8'd0, row};
            case (layer)
                2'd0: begin
                    channel_offset = (channel_value << 9) + (channel_value << 8) + (channel_value << 4);
                    row_offset = (row_value << 4) + (row_value << 3) + (row_value << 2);
                end
                2'd1: begin
                    channel_offset = (channel_value << 7) + (channel_value << 6) + (channel_value << 2);
                    row_offset = (row_value << 3) + (row_value << 2) + (row_value << 1);
                end
                default: begin
                    channel_offset = (channel_value << 5) + (channel_value << 4) + channel_value;
                    row_offset = (row_value << 2) + (row_value << 1) + row_value;
                end
            endcase
            feature_address = channel_offset + row_offset + {8'd0, col};
        end
    endfunction

    function [12:0] pooled_address;
        input [4:0] channel;
        input [3:0] row;
        input [3:0] col;
        input [1:0] layer;
        reg [12:0] channel_offset;
        reg [12:0] row_offset;
        reg [12:0] channel_value;
        reg [12:0] row_value;
        begin
            channel_value = {8'd0, channel};
            row_value = {9'd0, row};
            case (layer)
                2'd0: begin
                    channel_offset = (channel_value << 7) + (channel_value << 6) +
                                     (channel_value << 2);
                    row_offset = (row_value << 3) + (row_value << 2) + (row_value << 1);
                end
                2'd1: begin
                    channel_offset = (channel_value << 5) + (channel_value << 4) + channel_value;
                    row_offset = (row_value << 2) + (row_value << 1) + row_value;
                end
                default: begin
                    channel_offset = (channel_value << 3) + channel_value;
                    row_offset = (row_value << 1) + row_value;
                end
            endcase
            pooled_address = channel_offset + row_offset + {9'd0, col};
        end
    endfunction

    assign conv_feature_read_address = feature_address(
        input_channel, conv_row_coordinate, conv_col_coordinate, layer_index);
    assign pool_feature_read_address = feature_address(
        pool_channel, {pool_y, 1'b0} + pool_sample[1],
        {pool_x, 1'b0} + pool_sample[0], layer_index);

    always @* begin
        case (layer_index)
            2'd0: begin input_size = 28; pool_size = 14; end
            2'd1: begin input_size = 14; pool_size = 7; end
            default: begin input_size = 7; pool_size = 3; end
        endcase

        if (state == ST_FC_START || state == ST_FC_ISSUE || state == ST_FC_ACCUM ||
            state == ST_FC_SUM || state == ST_FC_COMPARE) begin
            params_layer = 2'd3;
            params_channel = {1'b0, fc_class};
        end else begin
            params_layer = layer_index;
            params_channel = output_channel;
        end

        bank0_read_addr = 13'd0;
        bank1_read_addr = 13'd0;
        bank0_read_enable = 1'b0;
        bank1_read_enable = 1'b0;
        bank0_write_addr = 13'd0;
        bank1_write_addr = 13'd0;
        bank0_write_data = 8'd0;
        bank1_write_data = 8'd0;
        bank0_write_enable = 1'b0;
        bank1_write_enable = 1'b0;
        weight_address = 12'd0;
        weight_read_enable = 1'b0;
        address_row = 6'sd0;
        address_col = 6'sd0;
        address_value = 13'd0;
        weight_value = 0;
        output_channel_value = output_channel;
        input_channel_value = input_channel;
        fc_class_value = fc_class;
        fc_channel_value = fc_channel;
        kernel_x_value = kernel_x;
        kernel_y_value = kernel_y;
        gap_channel_value = gap_channel;

        if (state == ST_LOAD && pixel_valid) begin
            bank0_write_enable = 1'b1;
            bank0_write_addr = {3'b000, load_count};
            bank0_write_data = pixel_data;
        end

        if (state == ST_CONV_COORD) begin
            address_row = output_y + kernel_y - 1;
            address_col = output_x + kernel_x - 1;
            case (layer_index)
                2'd0: weight_value = (output_channel_value << 3) + output_channel_value +
                                     (kernel_y_value << 1) + kernel_y_value + kernel_x_value;
                2'd1: weight_value = 72 + (output_channel_value << 6) +
                                     (output_channel_value << 3) + (input_channel_value << 3) +
                                     input_channel_value + (kernel_y_value << 1) +
                                     kernel_y_value + kernel_x_value;
                default: weight_value = 1224 + (output_channel_value << 7) +
                                        (output_channel_value << 4) + (input_channel_value << 3) +
                                        input_channel_value + (kernel_y_value << 1) +
                                        kernel_y_value + kernel_x_value;
            endcase
            weight_address = weight_value[11:0];
        end

        if (state == ST_CONV_ISSUE) begin
            if (input_bank) begin
                bank1_read_enable = 1'b1;
                bank1_read_addr = conv_read_address_register;
            end else begin
                bank0_read_enable = 1'b1;
                bank0_read_addr = conv_read_address_register;
            end
            weight_address = conv_weight_address_register;
            weight_read_enable = 1'b1;
        end

        if (state == ST_CONV_STORE) begin
            address_value = feature_address(output_channel, output_y, output_x, layer_index);
            if (input_bank) begin
                bank0_write_enable = 1'b1;
                bank0_write_addr = address_value[12:0];
                bank0_write_data = activated_value;
            end else begin
                bank1_write_enable = 1'b1;
                bank1_write_addr = address_value[12:0];
                bank1_write_data = activated_value;
            end
        end

        if (state == ST_POOL_ISSUE) begin
            if (input_bank) begin
                bank0_read_enable = 1'b1;
                bank0_read_addr = pool_read_address_register;
            end else begin
                bank1_read_enable = 1'b1;
                bank1_read_addr = pool_read_address_register;
            end
        end

        if (state == ST_POOL_WRITE) begin
            address_value = pooled_address(pool_channel, pool_y, pool_x, layer_index);
            if (input_bank) begin
                bank0_write_enable = 1'b1;
                bank0_write_addr = address_value[12:0];
                bank0_write_data = pool_max;
            end else begin
                bank1_write_enable = 1'b1;
                bank1_write_addr = address_value[12:0];
                bank1_write_data = pool_max;
            end
        end

        if (state == ST_GAP_ISSUE) begin
            address_value = (gap_channel_value << 3) + gap_channel_value + gap_position;
            if (input_bank) begin
                bank0_read_enable = 1'b1;
                bank0_read_addr = address_value[12:0];
            end else begin
                bank1_read_enable = 1'b1;
                bank1_read_addr = address_value[12:0];
            end
        end

        if (state == ST_FC_ISSUE) begin
            weight_value = 3528 + (fc_class_value << 4) + fc_channel_value;
            weight_address = weight_value[11:0];
            weight_read_enable = 1'b1;
        end

        if (state == ST_WEIGHT_SCAN_ISSUE) begin
            weight_address = weight_scan_address;
            weight_read_enable = 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_WEIGHT_SCAN_ISSUE;
            busy <= 1'b0;
            done <= 1'b0;
            class_id <= 4'd0;
            weight_check_done <= 1'b0;
            weight_crc32 <= 32'd0;
            weight_scan_address <= 12'd0;
            weight_crc_accumulator <= 32'hffffffff;
            conv_signature_accumulator <= 32'd0;
            pool_signature_accumulator <= 32'd0;
            conv1_input_accumulator <= 32'h13579BDF;
            conv1_weight_accumulator <= 32'h2468ACE0;
            conv1_product_accumulator <= 32'd0;
            conv1_accumulator_trace <= 32'd0;
            conv1_raw_accumulator <= 32'd0;
            conv1_signature <= 32'd0;
            pool1_signature <= 32'd0;
            conv2_signature <= 32'd0;
            pool2_signature <= 32'd0;
            conv3_signature <= 32'd0;
            pool3_signature <= 32'd0;
            conv1_input_signature <= 32'd0;
            conv1_weight_signature <= 32'd0;
            conv1_product_signature <= 32'd0;
            conv1_accumulator_signature <= 32'd0;
            conv1_raw_signature <= 32'd0;
            conv1_channel_signatures <= 256'd0;
            pool1_channel_signatures <= 256'd0;
            conv1_first_weight <= 8'd0;
            conv1_first_activation <= 8'd0;
            conv1_first_product <= 16'd0;
            conv1_first_sample_seen <= 1'b0;
            input_bank <= 1'b0;
            layer_index <= 2'd0;
            load_count <= 10'd0;
            output_channel <= 5'd0;
            input_channel <= 5'd0;
            output_x <= 5'd0;
            output_y <= 5'd0;
            conv_row_coordinate <= 5'd0;
            conv_col_coordinate <= 5'd0;
            conv_read_address_register <= 13'd0;
            conv_weight_address_register <= 12'd0;
            pool_read_address_register <= 13'd0;
            kernel_x <= 2'd0;
            kernel_y <= 2'd0;
            tap_in_bounds <= 1'b0;
            accumulator <= 32'd0;
            raw_value <= 32'sd0;
            conv_product_register <= 16'sd0;
            conv_weight_register <= 8'sd0;
            conv_activation_register <= 8'd0;
            activated_value <= 8'd0;
            pool_channel <= 5'd0;
            pool_x <= 4'd0;
            pool_y <= 4'd0;
            pool_sample <= 2'd0;
            pool_max <= 8'd0;
            gap_channel <= 5'd0;
            gap_position <= 4'd0;
            gap_accumulator <= 12'd0;
            fc_class <= 4'd0;
            fc_channel <= 5'd0;
            fc_weight_register <= 8'sd0;
            fc_feature_register <= 12'd0;
            best_score <= 32'sh80000000;
            best_class <= 4'd0;
            class_score <= 32'sd0;
            fc_product_register <= 20'sd0;
        end else begin
            weight_check_done <= 1'b0;
            case (state)
                ST_WEIGHT_SCAN_ISSUE: begin
                    state <= ST_WEIGHT_SCAN_CAPTURE;
                end

                ST_WEIGHT_SCAN_CAPTURE: begin
                    weight_crc_accumulator <= crc32_byte(weight_crc_accumulator, weight_data);
                    if (weight_scan_address == 12'd4095) begin
                        weight_crc32 <= ~crc32_byte(weight_crc_accumulator, weight_data);
                        weight_check_done <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        weight_scan_address <= weight_scan_address + 1'b1;
                        state <= ST_WEIGHT_SCAN_ISSUE;
                    end
                end

                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        done <= 1'b0;
                        input_bank <= 1'b0;
                        layer_index <= 2'd0;
                        load_count <= 10'd0;
                        conv_signature_accumulator <= 32'd0;
                        pool_signature_accumulator <= 32'd0;
                        conv1_input_accumulator <= 32'h13579BDF;
                        conv1_weight_accumulator <= 32'h2468ACE0;
                        conv1_product_accumulator <= 32'd0;
                        conv1_accumulator_trace <= 32'd0;
                        conv1_raw_accumulator <= 32'd0;
                        conv1_signature <= 32'd0;
                        pool1_signature <= 32'd0;
                        conv2_signature <= 32'd0;
                        pool2_signature <= 32'd0;
                        conv3_signature <= 32'd0;
                        pool3_signature <= 32'd0;
                        conv1_input_signature <= 32'd0;
                        conv1_weight_signature <= 32'd0;
                        conv1_product_signature <= 32'd0;
                        conv1_accumulator_signature <= 32'd0;
                        conv1_raw_signature <= 32'd0;
                        conv1_channel_signatures <= 256'd0;
                        pool1_channel_signatures <= 256'd0;
                        conv1_first_weight <= 8'd0;
                        conv1_first_activation <= 8'd0;
                        conv1_first_product <= 16'd0;
                        conv1_first_sample_seen <= 1'b0;
                        state <= ST_LOAD;
                    end
                end

                ST_LOAD: begin
                    if (pixel_valid) begin
                        if (load_count == 10'd783) begin
                            output_channel <= 5'd0;
                            input_channel <= 5'd0;
                            output_x <= 5'd0;
                            output_y <= 5'd0;
                            kernel_x <= 2'd0;
                            kernel_y <= 2'd0;
                            accumulator <= 32'd0;
                            state <= ST_CONV_COORD;
                        end else begin
                            load_count <= load_count + 1'b1;
                        end
                    end
                end

                ST_CONV_COORD: begin
                    conv_row_coordinate <= address_row[4:0];
                    conv_col_coordinate <= address_col[4:0];
                    conv_weight_address_register <= weight_address;
                    tap_in_bounds <= (address_row >= 0 && address_row < input_size &&
                                      address_col >= 0 && address_col < input_size);
                    state <= ST_CONV_ADDRESS;
                end

                ST_CONV_ADDRESS: begin
                    conv_read_address_register <= conv_feature_read_address;
                    state <= ST_CONV_ISSUE;
                end

                ST_CONV_ISSUE: state <= ST_CONV_OPERAND;

                ST_CONV_OPERAND: begin
                    if (layer_index == 2'd0) begin
                        conv1_input_accumulator <= signature_step(
                            conv1_input_accumulator,
                            tap_in_bounds ? activation_read_data : 8'd0);
                        conv1_weight_accumulator <= signature_step(
                            conv1_weight_accumulator, weight_data);
                    end
                    conv_weight_register <= weight_data;
                    conv_activation_register <= activation_read_data;
                    state <= ST_CONV_PRODUCT;
                end

                ST_CONV_PRODUCT: begin
                    if (layer_index == 2'd0) begin
                        conv1_product_accumulator <= signature_word_step(
                            conv1_product_accumulator,
                            {{16{conv_product[15]}}, conv_product});
                        if (!conv1_first_sample_seen && output_channel == 5'd0 &&
                            input_channel == 5'd0 && tap_in_bounds &&
                            conv_weight_register[7] &&
                            conv_activation_register != 8'd0) begin
                            conv1_first_weight <= conv_weight_register;
                            conv1_first_activation <= conv_activation_register;
                            conv1_first_product <= conv_product;
                            conv1_first_sample_seen <= 1'b1;
                        end
                    end
                    conv_product_register <= conv_product;
                    state <= ST_CONV_ACCUM;
                end

                ST_CONV_ACCUM: begin
                    if (layer_index == 2'd0)
                        conv1_accumulator_trace <= signature_word_step(
                            conv1_accumulator_trace, conv_accumulator_signed);
                    accumulator <= conv_accumulator_signed;
                    if (kernel_x == 2'd2 && kernel_y == 2'd2 &&
                        ((layer_index == 2'd0 && input_channel == 5'd0) ||
                         (layer_index == 2'd1 && input_channel == 5'd7) ||
                         (layer_index == 2'd2 && input_channel == 5'd15))) begin
                        state <= ST_CONV_BIAS;
                    end else begin
                        if (kernel_x != 2'd2) begin
                            kernel_x <= kernel_x + 1'b1;
                        end else begin
                            kernel_x <= 2'd0;
                            if (kernel_y != 2'd2) begin
                                kernel_y <= kernel_y + 1'b1;
                            end else begin
                                kernel_y <= 2'd0;
                                input_channel <= input_channel + 1'b1;
                            end
                        end
                        state <= ST_CONV_COORD;
                    end
                end

                ST_CONV_BIAS: begin
                    if (layer_index == 2'd0)
                        conv1_raw_accumulator <= signature_word_step(
                            conv1_raw_accumulator, $signed(accumulator) + bias_value);
                    raw_value <= $signed(accumulator) + bias_value;
                    state <= ST_CONV_REQUANT;
                end

                ST_CONV_REQUANT: begin
                    case (layer_index)
                        2'd0: activated_value <= requantize_relu(raw_value, conv1_shift);
                        2'd1: activated_value <= requantize_relu(raw_value, conv2_shift);
                        default: activated_value <= requantize_relu(raw_value, conv3_shift);
                    endcase
                    state <= ST_CONV_STORE;
                end

                ST_CONV_STORE: begin
                    conv_signature_accumulator <= signature_step(
                        conv_signature_accumulator, activated_value);
                    if (layer_index == 2'd0)
                        conv1_channel_signatures[output_channel * 32 +: 32] <=
                            signature_step(conv1_channel_signatures[output_channel * 32 +: 32], activated_value);
                    if (output_channel == ((layer_index == 2'd0) ? 5'd7 : 5'd15) &&
                        output_y == input_size - 1 && output_x == input_size - 1) begin
                        case (layer_index)
                            2'd0: begin
                                conv1_signature <= signature_step(
                                    conv_signature_accumulator, activated_value);
                                conv1_input_signature <= conv1_input_accumulator;
                                conv1_weight_signature <= conv1_weight_accumulator;
                                conv1_product_signature <= conv1_product_accumulator;
                                conv1_accumulator_signature <= conv1_accumulator_trace;
                                conv1_raw_signature <= conv1_raw_accumulator;
                            end
                            2'd1: conv2_signature <= signature_step(
                                conv_signature_accumulator, activated_value);
                            default: conv3_signature <= signature_step(
                                conv_signature_accumulator, activated_value);
                        endcase
                    end
                    accumulator <= 32'd0;
                    input_channel <= 5'd0;
                    kernel_x <= 2'd0;
                    kernel_y <= 2'd0;
                    if (output_x + 1 < input_size) begin
                        output_x <= output_x + 1'b1;
                        state <= ST_CONV_COORD;
                    end else begin
                        output_x <= 5'd0;
                        if (output_y + 1 < input_size) begin
                            output_y <= output_y + 1'b1;
                            state <= ST_CONV_COORD;
                        end else begin
                            output_y <= 5'd0;
                            if (output_channel != ((layer_index == 2'd0) ? 5'd7 : 5'd15)) begin
                                output_channel <= output_channel + 1'b1;
                                state <= ST_CONV_COORD;
                            end else begin
                                pool_channel <= 5'd0;
                                pool_x <= 4'd0;
                                pool_y <= 4'd0;
                                pool_sample <= 2'd0;
                                state <= ST_POOL_ADDRESS;
                            end
                        end
                    end
                end

                ST_POOL_ADDRESS: begin
                    pool_read_address_register <= pool_feature_read_address;
                    state <= ST_POOL_ISSUE;
                end

                ST_POOL_ISSUE: state <= ST_POOL_CAPTURE;

                ST_POOL_CAPTURE: begin
                    if (pool_sample == 2'd0 || activation_read_data > pool_max)
                        pool_max <= activation_read_data;
                    if (pool_sample == 2'd3) begin
                        state <= ST_POOL_WRITE;
                    end else begin
                        pool_sample <= pool_sample + 1'b1;
                        state <= ST_POOL_ADDRESS;
                    end
                end

                ST_POOL_WRITE: begin
                    pool_signature_accumulator <= signature_step(
                        pool_signature_accumulator, pool_max);
                    if (layer_index == 2'd0)
                        pool1_channel_signatures[pool_channel * 32 +: 32] <=
                            signature_step(pool1_channel_signatures[pool_channel * 32 +: 32], pool_max);
                    if (pool_channel == ((layer_index == 2'd0) ? 5'd7 : 5'd15) &&
                        pool_y == pool_size - 1 && pool_x == pool_size - 1) begin
                        case (layer_index)
                            2'd0: pool1_signature <= signature_step(
                                pool_signature_accumulator, pool_max);
                            2'd1: pool2_signature <= signature_step(
                                pool_signature_accumulator, pool_max);
                            default: pool3_signature <= signature_step(
                                pool_signature_accumulator, pool_max);
                        endcase
                    end
                    pool_sample <= 2'd0;
                    if (pool_x + 1 < pool_size) begin
                        pool_x <= pool_x + 1'b1;
                        state <= ST_POOL_ADDRESS;
                    end else begin
                        pool_x <= 4'd0;
                        if (pool_y + 1 < pool_size) begin
                            pool_y <= pool_y + 1'b1;
                            state <= ST_POOL_ADDRESS;
                        end else begin
                            pool_y <= 4'd0;
                            if (pool_channel != ((layer_index == 2'd0) ? 5'd7 : 5'd15)) begin
                                pool_channel <= pool_channel + 1'b1;
                                state <= ST_POOL_ADDRESS;
                            end else if (layer_index == 2'd2) begin
                                gap_channel <= 5'd0;
                                gap_position <= 4'd0;
                                gap_accumulator <= 12'd0;
                                state <= ST_GAP_ISSUE;
                            end else begin
                            conv_signature_accumulator <= 32'd0;
                            pool_signature_accumulator <= 32'd0;
                                input_bank <= ~input_bank;
                                layer_index <= layer_index + 1'b1;
                                output_channel <= 5'd0;
                                input_channel <= 5'd0;
                                output_x <= 5'd0;
                                output_y <= 5'd0;
                                kernel_x <= 2'd0;
                                kernel_y <= 2'd0;
                                accumulator <= 32'd0;
                                state <= ST_CONV_COORD;
                            end
                        end
                    end
                end

                ST_GAP_ISSUE: state <= ST_GAP_ACCUM;

                ST_GAP_ACCUM: begin
                    if (gap_position == 4'd8) begin
                        features[gap_channel] <= gap_accumulator + activation_read_data;
                        gap_accumulator <= 12'd0;
                        gap_position <= 4'd0;
                        if (gap_channel == 5'd15) begin
                            fc_class <= 4'd0;
                            best_score <= 32'sh80000000;
                            best_class <= 4'd0;
                            state <= ST_FC_START;
                        end else begin
                            gap_channel <= gap_channel + 1'b1;
                            state <= ST_GAP_ISSUE;
                        end
                    end else begin
                        gap_accumulator <= gap_accumulator + activation_read_data;
                        gap_position <= gap_position + 1'b1;
                        state <= ST_GAP_ISSUE;
                    end
                end

                ST_FC_START: begin
                    accumulator <= bias_value;
                    fc_channel <= 5'd0;
                    state <= ST_FC_ISSUE;
                end

                ST_FC_ISSUE: state <= ST_FC_ACCUM;

                ST_FC_ACCUM: begin
                    fc_weight_register <= weight_data;
                    fc_feature_register <= features[fc_channel];
                    state <= ST_FC_PRODUCT;
                end

                ST_FC_PRODUCT: begin
                    fc_product_register <= fc_product;
                    state <= ST_FC_SUM;
                end

                ST_FC_SUM: begin
                    if (fc_channel == 5'd15) begin
                        class_score <= fc_accumulator_signed;
                        state <= ST_FC_COMPARE;
                    end else begin
                        accumulator <= fc_accumulator_signed;
                        fc_channel <= fc_channel + 1'b1;
                        state <= ST_FC_ISSUE;
                    end
                end

                ST_FC_COMPARE: begin
                    if (fc_class == 4'd9) begin
                        if (class_score > best_score)
                            class_id <= fc_class;
                        else
                            class_id <= best_class;
                        done <= 1'b1;
                        busy <= 1'b0;
                        state <= ST_IDLE;
                    end else begin
                        if (class_score > best_score) begin
                            best_score <= class_score;
                            best_class <= fc_class;
                        end
                        fc_class <= fc_class + 1'b1;
                        state <= ST_FC_START;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`define CNN_PH1P35

module cnn_digit_activation_ram (
    input wire clk,
    input wire read_enable,
    input wire [12:0] read_address,
    output wire [7:0] read_data,
    input wire write_enable,
    input wire [12:0] write_address,
    input wire [7:0] write_data
);
`ifdef CNN_SIM
    reg [7:0] memory [0:8191];
    reg [7:0] read_data_sim;
    assign read_data = read_data_sim;

    always @(posedge clk) begin
        if (write_enable)
            memory[write_address] <= write_data;
        if (read_enable)
            read_data_sim <= memory[read_address];
    end
`elsif CNN_PH1P35
    wire [7:0] bank_read_data [0:3];
    reg [1:0] read_bank_q;
    wire [3:0] bank_write_enable;
    wire [3:0] bank_read_enable;
    genvar bank;
    generate for (bank = 0; bank < 4; bank = bank + 1) begin : g_activation_bank
        assign bank_write_enable[bank] = write_enable && (write_address[12:11] == bank);
        assign bank_read_enable[bank] = read_enable && (read_address[12:11] == bank);
        PH1P_LOGIC_ERAM #(
            .DATA_WIDTH_A(8), .ADDR_WIDTH_A(11), .DATA_DEPTH_A(2048),
            .DATA_WIDTH_B(8), .ADDR_WIDTH_B(11), .DATA_DEPTH_B(2048),
            .MODE("PDPW"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
            .IMPLEMENT("20K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
            .INIT_FILE("NONE"), .FILL_ALL("NONE")
        ) u_ram (
            .dia(write_data), .dib(8'd0),
            .addra(write_address[10:0]), .addrb(read_address[10:0]),
            .cea(bank_write_enable[bank]), .ceb(bank_read_enable[bank]),
            .ocea(1'b0), .oceb(1'b0), .clka(clk), .clkb(clk),
            .wea(bank_write_enable[bank]), .web(1'b0), .bea(8'hFF), .beb(8'h00),
            .rsta(1'b0), .rstb(1'b0), .doa(), .dob(bank_read_data[bank])
        );
    end endgenerate

    always @(posedge clk)
        if (read_enable)
            read_bank_q <= read_address[12:11];
    assign read_data = bank_read_data[read_bank_q];
`else
    reg [7:0] memory [0:8191];
    reg [7:0] read_data_generic;
    assign read_data = read_data_generic;
    always @(posedge clk) begin
        if (write_enable)
            memory[write_address] <= write_data;
        if (read_enable)
            read_data_generic <= memory[read_address];
    end
`endif
endmodule

module cnn_digit_weight_rom (
    input wire clk,
    input wire read_enable,
    input wire [11:0] address,
    output wire [7:0] data
);
`ifdef CNN_SIM
    reg [7:0] memory [0:4095];
    reg [7:0] data_reg;
    initial $readmemh("uisrc/01_rtl/cnn/weights/cnn_digit_weights.hex", memory);
    always @(posedge clk) begin
        if (read_enable)
            data_reg <= memory[address];
    end
    assign data = data_reg;
`elsif CNN_PH1P35
    wire [7:0] bank0_data;
    wire [7:0] bank1_data;
    reg bank_q;
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(8), .ADDR_WIDTH_A(11), .DATA_DEPTH_A(2048),
        .DATA_WIDTH_B(8), .ADDR_WIDTH_B(11), .DATA_DEPTH_B(2048),
        .MODE("SP"), .REGMODE_A("NOREG"),
        .IMPLEMENT("20K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("uisrc/01_rtl/cnn/weights/cnn_digit_weights_0.mif"),
        .FILL_ALL("NONE")
    ) u_weight_bank0 (
        .dia(8'd0), .dib(8'd0), .addra(address[10:0]), .addrb(11'd0),
        .cea(read_enable && !address[11]), .ceb(1'b0),
        .ocea(1'b0), .oceb(1'b0), .clka(clk), .clkb(1'b0),
        .wea(1'b0), .web(1'b0), .bea(8'h00), .beb(8'h00),
        .rsta(1'b0), .rstb(1'b0), .doa(bank0_data), .dob()
    );
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(8), .ADDR_WIDTH_A(11), .DATA_DEPTH_A(2048),
        .DATA_WIDTH_B(8), .ADDR_WIDTH_B(11), .DATA_DEPTH_B(2048),
        .MODE("SP"), .REGMODE_A("NOREG"),
        .IMPLEMENT("20K"), .DEBUGGABLE("NO"), .PACKABLE("NO"),
        .INIT_FILE("uisrc/01_rtl/cnn/weights/cnn_digit_weights_1.mif"),
        .FILL_ALL("NONE")
    ) u_weight_bank1 (
        .dia(8'd0), .dib(8'd0), .addra(address[10:0]), .addrb(11'd0),
        .cea(read_enable && address[11]), .ceb(1'b0),
        .ocea(1'b0), .oceb(1'b0), .clka(clk), .clkb(1'b0),
        .wea(1'b0), .web(1'b0), .bea(8'h00), .beb(8'h00),
        .rsta(1'b0), .rstb(1'b0), .doa(bank1_data), .dob()
    );
    always @(posedge clk)
        if (read_enable)
            bank_q <= address[11];
    assign data = bank_q ? bank1_data : bank0_data;
`else
    wire [7:0] data_wire;
    EG_LOGIC_BRAM #(
        .DATA_WIDTH_A(8),
        .ADDR_WIDTH_A(12),
        .DATA_DEPTH_A(4096),
        .DATA_WIDTH_B(8),
        .ADDR_WIDTH_B(12),
        .DATA_DEPTH_B(4096),
        .MODE("SP"),
        .REGMODE_A("NOREG"),
        .IMPLEMENT("32K"),
        .DEBUGGABLE("NO"),
        .PACKABLE("NO"),
        .INIT_FILE("uisrc/01_rtl/cnn/weights/cnn_digit_weights.mif"),
        .FILL_ALL("NONE")
    ) u_weight_bram (
        .dia(8'd0),
        .dib(8'd0),
        .addra(address),
        .addrb(12'd0),
        .cea(read_enable),
        .ceb(1'b0),
        .ocea(1'b0),
        .oceb(1'b0),
        .clka(clk),
        .clkb(1'b0),
        .wea(1'b0),
        .web(1'b0),
        .bea(1'b0),
        .beb(1'b0),
        .rsta(1'b0),
        .rstb(1'b0),
        .doa(data_wire),
        .dob()
    );
    assign data = data_wire;
`endif
endmodule
