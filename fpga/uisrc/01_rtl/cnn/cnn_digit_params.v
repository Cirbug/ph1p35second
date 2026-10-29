module cnn_digit_params(
    input wire [1:0] layer,
    input wire [4:0] channel,
    output reg signed [31:0] bias_value,
    output wire [3:0] conv1_shift,
    output wire [3:0] conv2_shift,
    output wire [3:0] conv3_shift
);
    assign conv1_shift = 4'd10;
    assign conv2_shift = 4'd9;
    assign conv3_shift = 4'd9;
    always @* begin
        case ({layer, channel})
            7'd0: bias_value = 32'h000030F0;
            7'd1: bias_value = 32'h00000981;
            7'd2: bias_value = 32'h00000002;
            7'd3: bias_value = 32'h00003672;
            7'd4: bias_value = 32'h000027EA;
            7'd5: bias_value = 32'h00002CAB;
            7'd6: bias_value = 32'h00000271;
            7'd7: bias_value = 32'h0000029A;
            7'd32: bias_value = 32'hFFFFFE76;
            7'd33: bias_value = 32'hFFFFFE84;
            7'd34: bias_value = 32'hFFFFFF8D;
            7'd35: bias_value = 32'h000000D8;
            7'd36: bias_value = 32'h0000028E;
            7'd37: bias_value = 32'h00000160;
            7'd38: bias_value = 32'h00000208;
            7'd39: bias_value = 32'hFFFFFF8B;
            7'd40: bias_value = 32'h0000013E;
            7'd41: bias_value = 32'h0000010E;
            7'd42: bias_value = 32'h000001D5;
            7'd43: bias_value = 32'h000000AF;
            7'd44: bias_value = 32'h000005E3;
            7'd45: bias_value = 32'h00000038;
            7'd46: bias_value = 32'h00000212;
            7'd47: bias_value = 32'h0000014F;
            7'd64: bias_value = 32'h00000006;
            7'd65: bias_value = 32'hFFFFFFFF;
            7'd66: bias_value = 32'h00000047;
            7'd67: bias_value = 32'hFFFFFFE5;
            7'd68: bias_value = 32'hFFFFFFCD;
            7'd69: bias_value = 32'hFFFFFFB3;
            7'd70: bias_value = 32'h0000004E;
            7'd71: bias_value = 32'hFFFFFFDA;
            7'd72: bias_value = 32'h00000049;
            7'd73: bias_value = 32'h0000005A;
            7'd74: bias_value = 32'h0000004B;
            7'd75: bias_value = 32'hFFFFFFCC;
            7'd76: bias_value = 32'hFFFFFFE2;
            7'd77: bias_value = 32'hFFFFFFA8;
            7'd78: bias_value = 32'hFFFFFFFB;
            7'd79: bias_value = 32'h0000001B;
            7'd96: bias_value = 32'h000003B3;
            7'd97: bias_value = 32'h00000013;
            7'd98: bias_value = 32'hFFFFFD7A;
            7'd99: bias_value = 32'hFFFFFC5D;
            7'd100: bias_value = 32'hFFFFFE90;
            7'd101: bias_value = 32'hFFFFFFC7;
            7'd102: bias_value = 32'hFFFFFD4A;
            7'd103: bias_value = 32'hFFFFFE36;
            7'd104: bias_value = 32'h000000AA;
            7'd105: bias_value = 32'hFFFFFF02;
            default: bias_value = 32'sd0;
        endcase
    end
endmodule
