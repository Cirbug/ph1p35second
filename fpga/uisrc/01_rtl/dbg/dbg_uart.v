//==========================================================================
// dbg_uart.v  （精简诊断版）
//   每 ~100ms 打印一行，115200 8N1：
//     C=1 W=0960 D=2B M=04C7 H0=2B09 H1=6A3E F=01D2 V=9AB3 X=0
//
//   W  = 解析出的 WC          D  = 解析出的 DT
//   M  = 最近一次 HS 突发的拍数
//   H0 = valid 上升沿那一拍的数据（应为 {DI, WC_LSB}）
//   H1 = 紧接着那一拍的数据（应为 {WC_MSB, ECC}）
//   F  = 发送侧帧计数         V = RX 有效字节计数
//   FS = RX 解析到的帧起数    FE = RX 解析到的帧尾数
//   RD = 原始 RX 上检测到的 FE 头数
//   DDR/relay/TX 诊断字段均为饱和计数；用于确认数据经过的最后一级。
//==========================================================================
module dbg_uart #(
    parameter CLK_HZ = 24000000,
    parameter BAUD   = 115200,
    parameter GAP_MS = 100
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_st_cfg,
    input  wire [15:0] I_st_wc,
    input  wire [7:0]  I_st_dt,
    input  wire [15:0] I_st_burst,
    input  wire [15:0] I_st_h0,
    input  wire [15:0] I_st_h1,
    input  wire [15:0] I_st_frame,
    input  wire [15:0] I_st_rxval,
    input  wire        I_st_werr,
    input  wire [15:0] I_st_gapmax,
    input  wire [15:0] I_st_pktcnt,
    input  wire [15:0] I_st_rx_fs,
    input  wire [15:0] I_st_rx_fe,
    input  wire [15:0] I_st_raw_fe,
    input  wire [15:0] I_st_rx_rows,
    input  wire [15:0] I_st_rx_wc,
    input  wire [15:0] I_st_rx_bad_rows,
    input  wire        I_st_rx_parse_err,
    input  wire [15:0] I_st_scale_rows,
    input  wire [15:0] I_st_scale_wc,
    input  wire        I_st_scale_err,
    input  wire [15:0] I_st_tx_packets,
    input  wire [15:0] I_st_tx_lines,
    input  wire [31:0] I_st_tx_bytes,
    input  wire [15:0] I_st_tx_wc,
    input  wire        I_st_tx_err,
    input  wire        I_st_ddr_init,
    input  wire [15:0] I_st_ddr_commit,
    input  wire [15:0] I_st_ddr_drop,
    input  wire [1:0]  I_st_ddr_ready_count,
    input  wire [15:0] I_st_ddr_rd_req,
    input  wire [15:0] I_st_ddr_rd_rsp,
    input  wire [15:0] I_st_cache_words,
    input  wire [15:0] I_st_cache_lines,
    input  wire [15:0] I_st_cache_frames,
    input  wire [15:0] I_st_relay_writes,
    input  wire [15:0] I_st_relay_reads,
    input  wire        I_st_relay_full,
    input  wire [15:0] I_st_tx_words,
    input  wire [15:0] I_st_tx_long_lines,
    input  wire        I_st_tx_dpll_locked,
    input  wire        I_st_tx_ready,
    input  wire        I_st_ddr_busy,
    input  wire        I_st_ddr_error,
    input  wire        I_st_tx_error_live,
    input  wire [15:0] I_st_ae_packets,
    input  wire [15:0] I_st_ae_errors,
    input  wire        I_st_ae_remote,

    output reg         O_uart_tx
);

    localparam DIV     = CLK_HZ / BAUD;
    localparam MS_CNT  = CLK_HZ / 1000;
    localparam GAP_CNT = MS_CNT * GAP_MS;
    localparam NBYTE   = 341;

    function [7:0] hx;
        input [3:0] n;
        begin
            hx = (n < 4'd10) ? (8'd48 + {4'd0, n}) : (8'd55 + {4'd0, n});
        end
    endfunction

    reg [8:0] idx;
    reg [7:0] msg_byte;

    always @(*) begin
        case (idx)
        8'd0 : msg_byte = 8'h43;                       // C
        8'd1 : msg_byte = 8'h3D;
        8'd2 : msg_byte = 8'd48 + { 7'd0, I_st_cfg };
        8'd3 : msg_byte = 8'h20;
        8'd4 : msg_byte = 8'h57;                       // W
        8'd5 : msg_byte = 8'h3D;
        8'd6 : msg_byte = hx(I_st_wc[15:12]);
        8'd7 : msg_byte = hx(I_st_wc[11:8]);
        8'd8 : msg_byte = hx(I_st_wc[7:4]);
        8'd9 : msg_byte = hx(I_st_wc[3:0]);
        8'd10: msg_byte = 8'h20;
        8'd11: msg_byte = 8'h44;                       // D
        8'd12: msg_byte = 8'h3D;
        8'd13: msg_byte = hx(I_st_dt[7:4]);
        8'd14: msg_byte = hx(I_st_dt[3:0]);
        8'd15: msg_byte = 8'h20;
        8'd16: msg_byte = 8'h4D;                       // M
        8'd17: msg_byte = 8'h3D;
        8'd18: msg_byte = hx(I_st_burst[15:12]);
        8'd19: msg_byte = hx(I_st_burst[11:8]);
        8'd20: msg_byte = hx(I_st_burst[7:4]);
        8'd21: msg_byte = hx(I_st_burst[3:0]);
        8'd22: msg_byte = 8'h20;
        8'd23: msg_byte = 8'h48;                       // H0
        8'd24: msg_byte = 8'h30;
        8'd25: msg_byte = 8'h3D;
        8'd26: msg_byte = hx(I_st_h0[15:12]);
        8'd27: msg_byte = hx(I_st_h0[11:8]);
        8'd28: msg_byte = hx(I_st_h0[7:4]);
        8'd29: msg_byte = hx(I_st_h0[3:0]);
        8'd30: msg_byte = 8'h20;
        8'd31: msg_byte = 8'h48;                       // H1
        8'd32: msg_byte = 8'h31;
        8'd33: msg_byte = 8'h3D;
        8'd34: msg_byte = hx(I_st_h1[15:12]);
        8'd35: msg_byte = hx(I_st_h1[11:8]);
        8'd36: msg_byte = hx(I_st_h1[7:4]);
        8'd37: msg_byte = hx(I_st_h1[3:0]);
        8'd38: msg_byte = 8'h20;
        8'd39: msg_byte = 8'h46;                       // F
        8'd40: msg_byte = 8'h3D;
        8'd41: msg_byte = hx(I_st_frame[15:12]);
        8'd42: msg_byte = hx(I_st_frame[11:8]);
        8'd43: msg_byte = hx(I_st_frame[7:4]);
        8'd44: msg_byte = hx(I_st_frame[3:0]);
        8'd45: msg_byte = 8'h20;
        8'd46: msg_byte = 8'h56;                       // V
        8'd47: msg_byte = 8'h3D;
        8'd48: msg_byte = hx(I_st_rxval[15:12]);
        8'd49: msg_byte = hx(I_st_rxval[11:8]);
        8'd50: msg_byte = hx(I_st_rxval[7:4]);
        8'd51: msg_byte = hx(I_st_rxval[3:0]);
        8'd52: msg_byte = 8'h20;
        8'd53: msg_byte = 8'h58;                       // X
        8'd54: msg_byte = 8'h3D;
        8'd55: msg_byte = 8'd48 + { 7'd0, I_st_werr };
        8'd56: msg_byte = 8'h20;
        8'd57: msg_byte = 8'h41;   // A = DT==0x00 的包计数
        8'd58: msg_byte = 8'h3D;
        8'd59: msg_byte = hx(I_st_gapmax[15:12]);
        8'd60: msg_byte = hx(I_st_gapmax[11:8]);
        8'd61: msg_byte = hx(I_st_gapmax[7:4]);
        8'd62: msg_byte = hx(I_st_gapmax[3:0]);
        8'd63: msg_byte = 8'h20;
        8'd64: msg_byte = 8'h54;   // T = TX 实际发出的拍数
        8'd65: msg_byte = 8'h3D;
        8'd66: msg_byte = hx(I_st_pktcnt[15:12]);
        8'd67: msg_byte = hx(I_st_pktcnt[11:8]);
        8'd68: msg_byte = hx(I_st_pktcnt[7:4]);
        8'd69: msg_byte = hx(I_st_pktcnt[3:0]);
        8'd70: msg_byte = 8'h20;
        8'd71: msg_byte = 8'h46;                       // FS
        8'd72: msg_byte = 8'h53;
        8'd73: msg_byte = 8'h3D;
        8'd74: msg_byte = hx(I_st_rx_fs[15:12]);
        8'd75: msg_byte = hx(I_st_rx_fs[11:8]);
        8'd76: msg_byte = hx(I_st_rx_fs[7:4]);
        8'd77: msg_byte = hx(I_st_rx_fs[3:0]);
        8'd78: msg_byte = 8'h20;
        8'd79: msg_byte = 8'h46;                       // FE
        8'd80: msg_byte = 8'h45;
        8'd81: msg_byte = 8'h3D;
        8'd82: msg_byte = hx(I_st_rx_fe[15:12]);
        8'd83: msg_byte = hx(I_st_rx_fe[11:8]);
        8'd84: msg_byte = hx(I_st_rx_fe[7:4]);
        8'd85: msg_byte = hx(I_st_rx_fe[3:0]);
        8'd86: msg_byte = 8'h20;
        8'd87: msg_byte = 8'h52;                       // RD
        8'd88: msg_byte = 8'h44;
        8'd89: msg_byte = 8'h3D;
        8'd90: msg_byte = hx(I_st_raw_fe[15:12]);
        8'd91: msg_byte = hx(I_st_raw_fe[11:8]);
        8'd92: msg_byte = hx(I_st_raw_fe[7:4]);
        8'd93: msg_byte = hx(I_st_raw_fe[3:0]);
        8'd94: msg_byte = 8'h20;
        8'd95: msg_byte = 8'h52;                       // RI
        8'd96: msg_byte = 8'h49;
        8'd97: msg_byte = 8'h3D;
        8'd98: msg_byte = hx(I_st_rx_rows[15:12]);
        8'd99: msg_byte = hx(I_st_rx_rows[11:8]);
        8'd100: msg_byte = hx(I_st_rx_rows[7:4]);
        8'd101: msg_byte = hx(I_st_rx_rows[3:0]);
        8'd102: msg_byte = 8'h20;
        8'd103: msg_byte = 8'h49;                      // IW
        8'd104: msg_byte = 8'h57;
        8'd105: msg_byte = 8'h3D;
        8'd106: msg_byte = hx(I_st_rx_wc[15:12]);
        8'd107: msg_byte = hx(I_st_rx_wc[11:8]);
        8'd108: msg_byte = hx(I_st_rx_wc[7:4]);
        8'd109: msg_byte = hx(I_st_rx_wc[3:0]);
        8'd110: msg_byte = 8'h20;
        8'd111: msg_byte = 8'h52;                      // RE
        8'd112: msg_byte = 8'h45;
        8'd113: msg_byte = 8'h3D;
        8'd114: msg_byte = hx(I_st_rx_bad_rows[15:12]);
        8'd115: msg_byte = hx(I_st_rx_bad_rows[11:8]);
        8'd116: msg_byte = hx(I_st_rx_bad_rows[7:4]);
        8'd117: msg_byte = hx(I_st_rx_bad_rows[3:0]);
        8'd118: msg_byte = 8'h20;
        8'd119: msg_byte = 8'h52;                      // RXE
        8'd120: msg_byte = 8'h58;
        8'd121: msg_byte = 8'h45;
        8'd122: msg_byte = 8'h3D;
        8'd123: msg_byte = 8'd48 + { 7'd0, I_st_rx_parse_err };
        8'd124: msg_byte = 8'h20;
        8'd125: msg_byte = 8'h53;                      // SO
        8'd126: msg_byte = 8'h4F;
        8'd127: msg_byte = 8'h3D;
        8'd128: msg_byte = hx(I_st_scale_rows[15:12]);
        8'd129: msg_byte = hx(I_st_scale_rows[11:8]);
        8'd130: msg_byte = hx(I_st_scale_rows[7:4]);
        8'd131: msg_byte = hx(I_st_scale_rows[3:0]);
        8'd132: msg_byte = 8'h20;
        8'd133: msg_byte = 8'h4F;                      // OW
        8'd134: msg_byte = 8'h57;
        8'd135: msg_byte = 8'h3D;
        8'd136: msg_byte = hx(I_st_scale_wc[15:12]);
        8'd137: msg_byte = hx(I_st_scale_wc[11:8]);
        8'd138: msg_byte = hx(I_st_scale_wc[7:4]);
        8'd139: msg_byte = hx(I_st_scale_wc[3:0]);
        8'd140: msg_byte = 8'h20;
        8'd141: msg_byte = 8'h53;                      // SE
        8'd142: msg_byte = 8'h45;
        8'd143: msg_byte = 8'h3D;
        8'd144: msg_byte = 8'd48 + { 7'd0, I_st_scale_err };
        8'd145: msg_byte = 8'h20;
        8'd146: msg_byte = 8'h54;                      // TP
        8'd147: msg_byte = 8'h50;
        8'd148: msg_byte = 8'h3D;
        8'd149: msg_byte = hx(I_st_tx_packets[15:12]);
        8'd150: msg_byte = hx(I_st_tx_packets[11:8]);
        8'd151: msg_byte = hx(I_st_tx_packets[7:4]);
        8'd152: msg_byte = hx(I_st_tx_packets[3:0]);
        8'd153: msg_byte = 8'h20;
        8'd154: msg_byte = 8'h54;                      // TL
        8'd155: msg_byte = 8'h4C;
        8'd156: msg_byte = 8'h3D;
        8'd157: msg_byte = hx(I_st_tx_lines[15:12]);
        8'd158: msg_byte = hx(I_st_tx_lines[11:8]);
        8'd159: msg_byte = hx(I_st_tx_lines[7:4]);
        8'd160: msg_byte = hx(I_st_tx_lines[3:0]);
        8'd161: msg_byte = 8'h20;
        8'd162: msg_byte = 8'h54;                      // TB
        8'd163: msg_byte = 8'h42;
        8'd164: msg_byte = 8'h3D;
        8'd165: msg_byte = hx(I_st_tx_bytes[31:28]);
        8'd166: msg_byte = hx(I_st_tx_bytes[27:24]);
        8'd167: msg_byte = hx(I_st_tx_bytes[23:20]);
        8'd168: msg_byte = hx(I_st_tx_bytes[19:16]);
        8'd169: msg_byte = hx(I_st_tx_bytes[15:12]);
        8'd170: msg_byte = hx(I_st_tx_bytes[11:8]);
        8'd171: msg_byte = hx(I_st_tx_bytes[7:4]);
        8'd172: msg_byte = hx(I_st_tx_bytes[3:0]);
        8'd173: msg_byte = 8'h20;
        8'd174: msg_byte = 8'h54;                      // TW
        8'd175: msg_byte = 8'h57;
        8'd176: msg_byte = 8'h3D;
        8'd177: msg_byte = hx(I_st_tx_wc[15:12]);
        8'd178: msg_byte = hx(I_st_tx_wc[11:8]);
        8'd179: msg_byte = hx(I_st_tx_wc[7:4]);
        8'd180: msg_byte = hx(I_st_tx_wc[3:0]);
        8'd181: msg_byte = 8'h20;
        8'd182: msg_byte = 8'h54;                      // TE
        8'd183: msg_byte = 8'h45;
        8'd184: msg_byte = 8'h3D;
        8'd185: msg_byte = 8'd48 + { 7'd0, I_st_tx_err };
        8'd186: msg_byte = 8'h20;                       // DDR and relay progress
        8'd187: msg_byte = 8'h44;                       // DI
        8'd188: msg_byte = 8'h49;
        8'd189: msg_byte = 8'h3D;
        8'd190: msg_byte = 8'd48 + {7'd0, I_st_ddr_init};
        8'd191: msg_byte = 8'h20;
        8'd192: msg_byte = 8'h44;                       // DC
        8'd193: msg_byte = 8'h43;
        8'd194: msg_byte = 8'h3D;
        8'd195: msg_byte = hx(I_st_ddr_commit[15:12]);
        8'd196: msg_byte = hx(I_st_ddr_commit[11:8]);
        8'd197: msg_byte = hx(I_st_ddr_commit[7:4]);
        8'd198: msg_byte = hx(I_st_ddr_commit[3:0]);
        8'd199: msg_byte = 8'h20;
        8'd200: msg_byte = 8'h44;                       // DP
        8'd201: msg_byte = 8'h50;
        8'd202: msg_byte = 8'h3D;
        8'd203: msg_byte = hx(I_st_ddr_drop[15:12]);
        8'd204: msg_byte = hx(I_st_ddr_drop[11:8]);
        8'd205: msg_byte = hx(I_st_ddr_drop[7:4]);
        8'd206: msg_byte = hx(I_st_ddr_drop[3:0]);
        8'd207: msg_byte = 8'h20;
        8'd208: msg_byte = 8'h42;                       // BR
        8'd209: msg_byte = 8'h52;
        8'd210: msg_byte = 8'h3D;
        8'd211: msg_byte = hx({2'b00, I_st_ddr_ready_count});
        8'd212: msg_byte = 8'h20;
        8'd213: msg_byte = 8'h52;                       // RQ
        8'd214: msg_byte = 8'h51;
        8'd215: msg_byte = 8'h3D;
        8'd216: msg_byte = hx(I_st_ddr_rd_req[15:12]);
        8'd217: msg_byte = hx(I_st_ddr_rd_req[11:8]);
        8'd218: msg_byte = hx(I_st_ddr_rd_req[7:4]);
        8'd219: msg_byte = hx(I_st_ddr_rd_req[3:0]);
        8'd220: msg_byte = 8'h20;
        8'd221: msg_byte = 8'h52;                       // RR
        8'd222: msg_byte = 8'h52;
        8'd223: msg_byte = 8'h3D;
        8'd224: msg_byte = hx(I_st_ddr_rd_rsp[15:12]);
        8'd225: msg_byte = hx(I_st_ddr_rd_rsp[11:8]);
        8'd226: msg_byte = hx(I_st_ddr_rd_rsp[7:4]);
        8'd227: msg_byte = hx(I_st_ddr_rd_rsp[3:0]);
        8'd228: msg_byte = 8'h20;
        8'd229: msg_byte = 8'h43;                       // CW
        8'd230: msg_byte = 8'h57;
        8'd231: msg_byte = 8'h3D;
        8'd232: msg_byte = hx(I_st_cache_words[15:12]);
        8'd233: msg_byte = hx(I_st_cache_words[11:8]);
        8'd234: msg_byte = hx(I_st_cache_words[7:4]);
        8'd235: msg_byte = hx(I_st_cache_words[3:0]);
        8'd236: msg_byte = 8'h20;
        8'd237: msg_byte = 8'h43;                       // CL
        8'd238: msg_byte = 8'h4C;
        8'd239: msg_byte = 8'h3D;
        8'd240: msg_byte = hx(I_st_cache_lines[15:12]);
        8'd241: msg_byte = hx(I_st_cache_lines[11:8]);
        8'd242: msg_byte = hx(I_st_cache_lines[7:4]);
        8'd243: msg_byte = hx(I_st_cache_lines[3:0]);
        8'd244: msg_byte = 8'h20;
        8'd245: msg_byte = 8'h43;                       // CF
        8'd246: msg_byte = 8'h46;
        8'd247: msg_byte = 8'h3D;
        8'd248: msg_byte = hx(I_st_cache_frames[15:12]);
        8'd249: msg_byte = hx(I_st_cache_frames[11:8]);
        8'd250: msg_byte = hx(I_st_cache_frames[7:4]);
        8'd251: msg_byte = hx(I_st_cache_frames[3:0]);
        8'd252: msg_byte = 8'h20;
        8'd253: msg_byte = 8'h52;                       // RW
        8'd254: msg_byte = 8'h57;
        8'd255: msg_byte = 8'h3D;
        9'd256: msg_byte = hx(I_st_relay_writes[15:12]);
        9'd257: msg_byte = hx(I_st_relay_writes[11:8]);
        9'd258: msg_byte = hx(I_st_relay_writes[7:4]);
        9'd259: msg_byte = hx(I_st_relay_writes[3:0]);
        9'd260: msg_byte = 8'h20;
        9'd261: msg_byte = 8'h52;                       // RC
        9'd262: msg_byte = 8'h43;
        9'd263: msg_byte = 8'h3D;
        9'd264: msg_byte = hx(I_st_relay_reads[15:12]);
        9'd265: msg_byte = hx(I_st_relay_reads[11:8]);
        9'd266: msg_byte = hx(I_st_relay_reads[7:4]);
        9'd267: msg_byte = hx(I_st_relay_reads[3:0]);
        9'd268: msg_byte = 8'h20;
        9'd269: msg_byte = 8'h52;                       // RF
        9'd270: msg_byte = 8'h46;
        9'd271: msg_byte = 8'h3D;
        9'd272: msg_byte = 8'd48 + {7'd0, I_st_relay_full};
        9'd273: msg_byte = 8'h20;
        9'd274: msg_byte = 8'h54;                       // TXW
        9'd275: msg_byte = 8'h58;
        9'd276: msg_byte = 8'h57;
        9'd277: msg_byte = 8'h3D;
        9'd278: msg_byte = hx(I_st_tx_words[15:12]);
        9'd279: msg_byte = hx(I_st_tx_words[11:8]);
        9'd280: msg_byte = hx(I_st_tx_words[7:4]);
        9'd281: msg_byte = hx(I_st_tx_words[3:0]);
        9'd282: msg_byte = 8'h20;
        9'd283: msg_byte = 8'h54;                       // TXL
        9'd284: msg_byte = 8'h58;
        9'd285: msg_byte = 8'h4C;
        9'd286: msg_byte = 8'h3D;
        9'd287: msg_byte = hx(I_st_tx_long_lines[15:12]);
        9'd288: msg_byte = hx(I_st_tx_long_lines[11:8]);
        9'd289: msg_byte = hx(I_st_tx_long_lines[7:4]);
        9'd290: msg_byte = hx(I_st_tx_long_lines[3:0]);
        9'd291: msg_byte = 8'h20;
        9'd292: msg_byte = 8'h44;                       // DL
        9'd293: msg_byte = 8'h4C;
        9'd294: msg_byte = 8'h3D;
        9'd295: msg_byte = 8'd48 + {7'd0, I_st_tx_dpll_locked};
        9'd296: msg_byte = 8'h20;
        9'd297: msg_byte = 8'h52;                       // RDY
        9'd298: msg_byte = 8'h44;
        9'd299: msg_byte = 8'h59;
        9'd300: msg_byte = 8'h3D;
        9'd301: msg_byte = 8'd48 + {7'd0, I_st_tx_ready};
        9'd302: msg_byte = 8'h20;
        9'd303: msg_byte = 8'h44;                       // DB
        9'd304: msg_byte = 8'h42;
        9'd305: msg_byte = 8'h3D;
        9'd306: msg_byte = 8'd48 + {7'd0, I_st_ddr_busy};
        9'd307: msg_byte = 8'h20;
        9'd308: msg_byte = 8'h44;                       // DE
        9'd309: msg_byte = 8'h45;
        9'd310: msg_byte = 8'h3D;
        9'd311: msg_byte = 8'd48 + {7'd0, I_st_ddr_error};
        9'd312: msg_byte = 8'h20;
        9'd313: msg_byte = 8'h54;                       // TXE
        9'd314: msg_byte = 8'h58;
        9'd315: msg_byte = 8'h45;
        9'd316: msg_byte = 8'h3D;
        9'd317: msg_byte = 8'd48 + {7'd0, I_st_tx_error_live};
        9'd318: msg_byte = 8'h20;
        9'd319: msg_byte = 8'h41;
        9'd320: msg_byte = 8'h50;
        9'd321: msg_byte = 8'h3D;
        9'd322: msg_byte = hx(I_st_ae_packets[15:12]);
        9'd323: msg_byte = hx(I_st_ae_packets[11:8]);
        9'd324: msg_byte = hx(I_st_ae_packets[7:4]);
        9'd325: msg_byte = hx(I_st_ae_packets[3:0]);
        9'd326: msg_byte = 8'h20;
        9'd327: msg_byte = 8'h45;
        9'd328: msg_byte = 8'h52;
        9'd329: msg_byte = 8'h3D;
        9'd330: msg_byte = hx(I_st_ae_errors[15:12]);
        9'd331: msg_byte = hx(I_st_ae_errors[11:8]);
        9'd332: msg_byte = hx(I_st_ae_errors[7:4]);
        9'd333: msg_byte = hx(I_st_ae_errors[3:0]);
        9'd334: msg_byte = 8'h20;
        9'd335: msg_byte = 8'h52;
        9'd336: msg_byte = 8'h41;
        9'd337: msg_byte = 8'h3D;
        9'd338: msg_byte = 8'd48 + {7'd0, I_st_ae_remote};
        9'd339: msg_byte = 8'h0D;
        9'd340: msg_byte = 8'h0A;
        default: msg_byte = 8'h20;
        endcase
    end

    localparam S_GAP   = 2'd0;
    localparam S_LOAD  = 2'd1;
    localparam S_SHIFT = 2'd2;
    localparam S_NEXT  = 2'd3;

    reg [1:0]  s;
    reg [23:0] gap_cnt;
    reg [15:0] div_cnt;
    reg [3:0]  bitn;
    reg [9:0]  shft;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s <= S_GAP; gap_cnt <= 24'd0; div_cnt <= 16'd0;
            bitn <= 4'd0; shft <= 10'h3FF; idx <= 9'd0; O_uart_tx <= 1'b1;
        end else begin
            case (s)
            S_GAP: begin
                O_uart_tx <= 1'b1;
                if (gap_cnt >= GAP_CNT[23:0]) begin
                    gap_cnt <= 24'd0; idx <= 9'd0; s <= S_LOAD;
                end else gap_cnt <= gap_cnt + 24'd1;
            end
            S_LOAD: begin
                shft <= { 1'b1, msg_byte, 1'b0 };
                bitn <= 4'd0; div_cnt <= 16'd0;
                O_uart_tx <= 1'b0; s <= S_SHIFT;
            end
            S_SHIFT: begin
                if (div_cnt >= (DIV - 1)) begin
                    div_cnt <= 16'd0;
                    if (bitn == 4'd9) begin O_uart_tx <= 1'b1; s <= S_NEXT; end
                    else begin
                        bitn <= bitn + 4'd1;
                        shft <= { 1'b1, shft[9:1] };
                        O_uart_tx <= shft[1];
                    end
                end else div_cnt <= div_cnt + 16'd1;
            end
            S_NEXT: begin
                if (idx >= (NBYTE - 1)) s <= S_GAP;
                else begin idx <= idx + 8'd1; s <= S_LOAD; end
            end
            default: s <= S_GAP;
            endcase
        end
    end

endmodule
