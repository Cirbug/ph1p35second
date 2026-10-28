//==========================================================================
// crc16_csi.v
//   CSI-2 / DSI 包尾 CRC-16
//   多项式 x^16+x^12+x^5+1 (0x1021)，初值 0xFFFF，不反射，无输出异或
//   即 CRC-16/CCITT-FALSE
//
//   用法：每个包开始前 I_init 打一拍，然后逐字节 I_valid，
//         最后一个载荷字节之后输出的 O_crc 即为包尾 CRC。
//   校验向量："123456789" -> 0x29B1（tb 已验证）
//==========================================================================
module crc16_csi (
    input  wire        I_clk,
    input  wire        I_rst,
    input  wire        I_init,     // 同步复位到 0xFFFF
    input  wire        I_valid,
    input  wire [7:0]  I_data,
    output reg  [15:0] O_crc
);

    integer    i;
    reg [15:0] c;
    reg        b;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            O_crc <= 16'hFFFF;
        end else if (I_init) begin
            O_crc <= 16'hFFFF;
        end else if (I_valid) begin
            c = O_crc;
            for (i = 7; i >= 0; i = i - 1) begin
                b = c[15] ^ I_data[i];
                c = {c[14:0], 1'b0};
                if (b)
                    c = c ^ 16'h1021;
            end
            O_crc <= c;
        end
    end

endmodule
