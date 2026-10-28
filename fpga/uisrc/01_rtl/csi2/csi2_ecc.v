//==========================================================================
// csi2_ecc.v
//   CSI-2 包头 6 位 ECC（Hamming）。
//
//   输入 I_hdr 的字节顺序必须是 {WC_MSB, WC_LSB, DI}：
//     - 摄像头实测 WC=0x0CA8、DI=0x2B 时 ECC=0x3B，
//       正好等于 I_hdr=24'h0CA82B 按下面掩码算出的结果。
//   这与 A 板工程的 csi_unpacket_2lane 完全一致。
//
//   说明：早期版本按另一种位序实现，能保证单比特纠错唯一性，
//   但算出的值和标准不同（0x9D vs 0x3B），导致对端校验失败。
//==========================================================================
module csi2_ecc (
    input  wire [23:0] I_hdr,
    output wire [7:0]  O_ecc
);

    assign O_ecc[0]   = ^(I_hdr & 24'hf12cb7);
    assign O_ecc[1]   = ^(I_hdr & 24'hf2555b);
    assign O_ecc[2]   = ^(I_hdr & 24'h749a6d);
    assign O_ecc[3]   = ^(I_hdr & 24'hb8e38e);
    assign O_ecc[4]   = ^(I_hdr & 24'hdf03f0);
    assign O_ecc[5]   = ^(I_hdr & 24'heffc00);
    assign O_ecc[7:6] = 2'b00;

endmodule