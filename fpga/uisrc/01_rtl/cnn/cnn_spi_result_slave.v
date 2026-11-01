`ifndef CNN_SPI_RESULT_SLAVE_V
`define CNN_SPI_RESULT_SLAVE_V
// Mode 0, MSB first. All logic uses clk; SPI pads enter synchronizers.
// A transaction reads an immutable 160-byte packet. A5 is the first MOSI byte.
// Header DG/01/flags/seqBE/count/00, 16 * (xBE,yBE,wBE,hBE,class),
// weight CRC32 BE, CRC16-CCITT(init FFFF) over bytes 0..155, CR LF.
module cnn_spi_result_slave (
    input wire clk,rst_n,spi_clk,spi_cs_n,spi_mosi,
    output wire spi_miso,
    input wire digit_done,frame_done,
    input wire [9:0] digit_x,digit_w,
    input wire [8:0] digit_y,digit_h,
    input wire [3:0] digit_class,
    input wire [4:0] frame_count,
    input wire [31:0] weight_crc
);
    (* async_reg="true" *) reg [1:0] sclk_s,cs_s,mosi_s;
    reg sclk_d,cs_d;
    wire rising=sclk_s[1]&&!sclk_d, falling=!sclk_s[1]&&sclk_d;
    wire select=!cs_s[1]&&cs_d;
    reg [41:0] records[0:31];
    reg [7:0] packet[0:511];
    reg capture_bank,source_bank,build_bank,published_bank,read_bank;
    reg published_valid,read_valid;
    reg [4:0] collected,saved_count;
    reg overflow,saved_overflow;
    reg [15:0] frame_sequence,saved_sequence,crc16;
    reg [31:0] saved_weight;
    reg [41:0] record_data;
    reg [7:0] build_index;
    reg [3:0] record_index,field_index;
    reg [1:0] state;
    localparam IDLE=0,BUILD=1,LOAD=2,PUBLISH=3;
    reg [7:0] tx_index,rx_shift;
    reg [7:0] read_data;
    reg [2:0] bit_index;
    reg miso_bit,command_ok;
    wire accept=digit_done && collected<16;
    wire [4:0] completed_count=collected+accept;
    wire [7:0] tx_byte=(read_valid && (tx_index==0 || command_ok)) ? read_data : 8'd0;
    // Synchronous read permits a dual-port ERAM; the SPI half-period is much
    // longer than this one-cycle latency. Header byte 0 has its MSB clear.
    always @(posedge clk) read_data<=packet[{read_bank,tx_index}];
    assign spi_miso=spi_cs_n ? 1'bz : miso_bit;
    function [15:0] crc_byte;
        input [15:0] crc;
        input [7:0] data;
        reg [15:0] c; integer k;
        begin
            c=crc^{data,8'd0};
            for(k=0;k<8;k=k+1)c=c[15]?(c<<1)^16'h1021:c<<1;
            crc_byte=c;
        end
    endfunction
    reg [7:0] build_byte;
    always @* begin
        build_byte=0;
        case(build_index)
        0:build_byte=8'h44; 1:build_byte=8'h47; 2:build_byte=1;
        3:build_byte={6'd0,saved_overflow,1'b1};
        4:build_byte=saved_sequence[15:8]; 5:build_byte=saved_sequence[7:0];
        6:build_byte={3'd0,saved_count}; 7:build_byte=0;
        152:build_byte=saved_weight[31:24];153:build_byte=saved_weight[23:16];
        154:build_byte=saved_weight[15:8];155:build_byte=saved_weight[7:0];
        156:build_byte=crc16[15:8];157:build_byte=crc16[7:0];
        158:build_byte=13;159:build_byte=10;
        default: if({1'b0,record_index}<saved_count) begin
            case(field_index)
            0:build_byte={6'd0,record_data[41:40]};1:build_byte=record_data[39:32];
            2:build_byte={7'd0,record_data[31]};3:build_byte=record_data[30:23];
            4:build_byte={6'd0,record_data[22:21]};5:build_byte=record_data[20:13];
            6:build_byte={7'd0,record_data[12]};7:build_byte=record_data[11:4];
            8:build_byte={4'd0,record_data[3:0]};
            default:build_byte=0;
            endcase
        end
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sclk_s<=0;cs_s<=3;mosi_s<=0;sclk_d<=0;cs_d<=1;
            tx_index<=0;bit_index<=7;rx_shift<=0;miso_bit<=0;
            read_bank<=0;read_valid<=0;command_ok<=0;
        end else begin
            sclk_s<={sclk_s[0],spi_clk};cs_s<={cs_s[0],spi_cs_n};mosi_s<={mosi_s[0],spi_mosi};
            sclk_d<=sclk_s[1];cs_d<=cs_s[1];
            if(select) begin
                read_bank<=published_bank;read_valid<=published_valid;
                tx_index<=0;bit_index<=7;command_ok<=0;rx_shift<=0;
                miso_bit<=0;
            end else if(!cs_s[1]) begin
                if(rising) begin
                    rx_shift<={rx_shift[6:0],mosi_s[1]};
                    if(bit_index==0) begin
                        if(tx_index==0)command_ok<=({rx_shift[6:0],mosi_s[1]}==8'ha5);
                        if(tx_index<159)tx_index<=tx_index+1'b1;
                        bit_index<=7;
                    end else bit_index<=bit_index-1'b1;
                end
                if(falling)miso_bit<=tx_byte[bit_index];
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            capture_bank<=0;source_bank<=0;build_bank<=1;published_bank<=0;
            published_valid<=0;collected<=0;overflow<=0;saved_count<=0;saved_overflow<=0;
            frame_sequence<=0;saved_sequence<=0;saved_weight<=0;crc16<=16'hffff;
            record_data<=0;build_index<=0;record_index<=0;field_index<=0;state<=IDLE;
        end else begin
            if(accept)begin
                records[{capture_bank,collected[3:0]}]<={digit_x,digit_y,digit_w,digit_h,digit_class};
                collected<=collected+1'b1;
            end
            if(digit_done && collected==16)overflow<=1;
            if(frame_done)begin
                frame_sequence<=frame_sequence+1'b1;collected<=0;overflow<=0;
                if(state==IDLE)begin
                    source_bank<=capture_bank;capture_bank<=~capture_bank;
                    build_bank<=~published_bank;saved_sequence<=frame_sequence+1'b1;
                    saved_count<=completed_count;saved_weight<=weight_crc;
                    saved_overflow<=overflow || (digit_done && collected==16) || frame_count!=completed_count;
                    build_index<=0;record_index<=0;field_index<=0;crc16<=16'hffff;state<=BUILD;
                end
                // If packet preparation/publication is busy, drop this result
                // group rather than stall the camera or overwrite its source bank.
            end
            case(state)
            BUILD:begin
                packet[{build_bank,build_index}]<=build_byte;
                if(build_index<156)crc16<=crc_byte(crc16,build_byte);
                if(build_index==159)state<=PUBLISH;
                else begin
                    build_index<=build_index+1'b1;
                    if(build_index==7)state<=LOAD;
                    else if(build_index>=8 && build_index<=151)begin
                        if(field_index==8)begin
                            field_index<=0;record_index<=record_index+1'b1;
                            if(build_index!=151)state<=LOAD;
                        end else field_index<=field_index+1'b1;
                    end
                end
            end
            LOAD:begin record_data<=records[{source_bank,record_index}];state<=BUILD;end
            PUBLISH:if(cs_s[1] && spi_cs_n)begin published_bank<=build_bank;published_valid<=1;state<=IDLE;end
            default:begin end
            endcase
        end
    end
endmodule
`endif
