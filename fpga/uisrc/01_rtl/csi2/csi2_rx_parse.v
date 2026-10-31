// Two-lane CSI-2 parser. Wire byte order is {earlier byte, later byte}.
// Packets can share one HS interval: advance by WC and consume the CRC beat.
// Header ECC detects/rejects errors; payload CRC is consumed, not checked.
// SC500 HS trail may be valid 00/FF fill. Do not scan through this fill or
// accept FS after another packet in the same burst (zero fill aliases FS=0).
module csi2_rx_parse #(
    parameter LANE_NUM = 2
)(
    input wire I_clk,I_rst,I_hs_valid,
    input wire [LANE_NUM*8-1:0] I_hs_data,
    output reg O_wr_en,
    output reg [LANE_NUM*8-1:0] O_wr_data,
    output reg O_pkt_end,
    output reg [15:0] O_pkt_wc,
    output reg [7:0] O_pkt_dt,
    output reg O_frame_start,O_frame_end,O_wc_err
);
    localparam HEADER0=3'd0,HEADER1=3'd1,PAYLOAD=3'd2,FOOTER=3'd3,DROP=3'd4;
    reg [2:0] state;
    reg [15:0] header0,remaining;
    reg first_header,raw_packet;
    wire [23:0] header_bits={I_hs_data[15:8],header0[7:0],header0[15:8]};
    wire [15:0] byte_count={I_hs_data[15:8],header0[7:0]};
    wire vc0=header0[15:14]==0;
    wire is_raw=vc0 && header0[13:8]==6'h2b;
    wire [7:0] ecc;
    csi2_ecc u_header_ecc(.I_hdr(header_bits),.O_ecc(ecc));
    wire ecc_ok=I_hs_data[7:0]==ecc;
    wire tail_fill=(I_hs_data[15:8]==0 || I_hs_data[15:8]==8'hff) &&
                   (I_hs_data[7:0]==0 || I_hs_data[7:0]==8'hff);

    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            state<=HEADER0;header0<=0;remaining<=0;first_header<=1;raw_packet<=0;
            O_wr_en<=0;O_wr_data<=0;O_pkt_end<=0;O_pkt_wc<=0;O_pkt_dt<=0;
            O_frame_start<=0;O_frame_end<=0;O_wc_err<=0;
        end else begin
            O_wr_en<=0;O_pkt_end<=0;O_frame_start<=0;O_frame_end<=0;
            if(!I_hs_valid) begin
                // LP aborts an incomplete header/payload/CRC. The next burst
                // starts clean; do not resume a truncated packet as pixels.
                if(state==HEADER1 || state==PAYLOAD || state==FOOTER)
                    O_wc_err<=1;
                if(raw_packet && (state==PAYLOAD || state==FOOTER))
                    O_pkt_end<=1; // let the cropper detect/reset a short line
                state<=HEADER0;first_header<=1;raw_packet<=0;remaining<=0;
            end else case(state)
                HEADER0: begin
                    if(!first_header && (tail_fill || I_hs_data[15:8]==0))
                        state<=DROP;
                    else begin header0<=I_hs_data;state<=HEADER1;end
                end
                HEADER1: begin
                    first_header<=0;raw_packet<=0;
                    if(!ecc_ok) begin O_wc_err<=1;state<=DROP;end
                    else begin
                        O_pkt_dt<=header0[15:8];O_pkt_wc<=byte_count;
                        if(header0[13:8]<6'h10) begin
                            O_frame_start<=first_header && vc0 && header0[13:8]==0;
                            O_frame_end<=vc0 && header0[13:8]==1;
                            state<=(vc0 && header0[13:8]==1) ? DROP:HEADER0;
                        end else if(byte_count[0] || (is_raw && byte_count==0)) begin
                            O_wc_err<=1;state<=DROP;
                        end else begin
                            raw_packet<=is_raw;remaining<=byte_count>>1;
                            state<=byte_count==0 ? FOOTER:PAYLOAD;
                        end
                    end
                end
                PAYLOAD: begin
                    if(raw_packet) begin O_wr_en<=1;O_wr_data<=I_hs_data;end
                    remaining<=remaining-1'b1;
                    if(remaining==1) state<=FOOTER;
                end
                FOOTER: begin
                    O_pkt_end<=raw_packet;raw_packet<=0;state<=HEADER0;
                end
                default: state<=DROP;
            endcase
        end
    end
endmodule

// Source frame boundary for the cropper. An actual FS always resets its row
// phase, including after interrupted reception. Close an old partial crop
// before accepting the next source frame. Keep height fallback for SC500
// streams lacking FE, but never let that fallback override a genuine FS.
module csi2_raw_frame_sync #(
    parameter integer FRAME_LINES=1944
)(
    input wire clk,rst_n,frame_start,frame_end,word_valid,line_end,
    output wire frame_boundary
);
    localparam integer CW=(FRAME_LINES>1) ? $clog2(FRAME_LINES):1;
    reg active;
    reg [CW-1:0] rows;
    wire inferred_end=active && line_end && rows==FRAME_LINES-1;
    assign frame_boundary=frame_start || frame_end || inferred_end;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin active<=0;rows<=0;end
        else if(frame_start) begin active<=1;rows<=0;end
        else if(frame_end || inferred_end) begin active<=0;rows<=0;end
        else begin
            if(!active && word_valid) begin active<=1;rows<=0;end
            if(active && line_end) rows<=rows+1'b1;
        end
    end
endmodule
