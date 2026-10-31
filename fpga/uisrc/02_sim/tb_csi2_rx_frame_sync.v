`timescale 1ns/1ps
// Spatial test: reception starts halfway through a sensor frame; genuine FS
// must realign the crop. Check every output byte in successive good frames.
module tb_csi2_rx_frame_sync #(parameter FULL=0);
    localparam SW=FULL ? 2592:40, SH=FULL ? 1944:16, CX=FULL ? 8:4;
    localparam DW=FULL ? 1920:24, DH=FULL ? 1440:12;
    localparam OH=FULL ? 1080:8, CY=FULL ? 180:2;
    localparam WORDS_PER_LINE=DW*10/16;
    reg clk=0,rst=1,valid=0;
    reg [15:0] data=0;
    always #5 clk=~clk;
    wire pv,fs,fe,le,error,boundary;
    wire [15:0] payload,wc;
    wire [7:0] dt;
    csi2_rx_parse parser(.I_clk(clk),.I_rst(rst),.I_hs_valid(valid),.I_hs_data(data),
        .O_wr_en(pv),.O_wr_data(payload),.O_pkt_end(le),.O_pkt_wc(wc),
        .O_pkt_dt(dt),.O_frame_start(fs),.O_frame_end(fe),.O_wc_err(error));
    csi2_raw_frame_sync #(.FRAME_LINES(SH)) syncer(.clk(clk),.rst_n(!rst),
        .frame_start(fs),.frame_end(fe),.word_valid(pv),.line_end(le),.frame_boundary(boundary));
    wire ov,ofs,ofe,ole,scale_error;
    wire [15:0] out_data;
    csi2_raw10_scale_crop #(.SRC_WIDTH(SW),.SRC_HEIGHT(SH),.SRC_CROP_X(CX),
        .SCALE_WIDTH(DW),.SCALE_HEIGHT(DH),.OUT_HEIGHT(OH),.CROP_Y(CY)) cropper(
        .clk(clk),.rst(rst),.in_valid(pv),.in_data(payload),.in_line_end(le),
        .in_word_count(wc),.in_data_type(dt),.in_frame_end(boundary),.out_valid(ov),
        .out_data(out_data),.out_frame_start(ofs),.out_line_end(ole),.out_word_count(),
        .out_frame_end(ofe),.format_error(scale_error));
    reg check_enable=0;
    reg crop_open=0;
    integer tag=0,words=0,lines=0,starts=0,ends=0;
    integer row_index;
    reg [7:0] value;
    reg [15:0] expected;
    function integer source_row(input integer row);
        // Map output Bayer row pairs directly to source coordinates.
        source_row=2*(((CY/2+row/2)*(SH/2)+DH/2-1)/(DH/2))+row%2;
    endfunction
    // Input P0..P3 low bits are 0,1,2,3. Independently map output pairs
    // back to source pixel coordinates, then form the standard RAW10 byte.
    function [7:0] low_byte(input integer group_index);
        integer x0,x2;
        begin
            x0=CX+2*((group_index*2*((SW-2*CX)/2)+DW/2-1)/(DW/2));
            x2=CX+2*(((group_index*2+1)*((SW-2*CX)/2)+DW/2-1)/(DW/2));
            low_byte=(x0%4)|(((x0+1)%4)<<2)|((x2%4)<<4)|(((x2+1)%4)<<6);
        end
    endfunction
    always @(posedge clk) begin
        if(rst) crop_open=0;
        else begin
            if(ofs) begin
                if(crop_open) $fatal(1,"crop opened twice without FE");
                crop_open=1;
            end
            if(ofe) begin
                if(!crop_open) $fatal(1,"orphan/duplicate crop FE");
                crop_open=0;
            end
        end
    end
    always @(posedge clk) if(!rst && check_enable) begin
        if(ofs) starts=starts+1;
        if(ofe) ends=ends+1;
        if(ole) lines=lines+1;
        if(ov) begin
            row_index=words/WORDS_PER_LINE;
            if(row_index>=OH) $fatal(1,"too many crop pixels");
            value=tag*64+source_row(row_index);
            case(words%5)
                0,1,3:expected={value,value};
                2:expected={low_byte(((words%WORDS_PER_LINE)/5)*2),value};
                4:expected={value,low_byte(((words%WORDS_PER_LINE)/5)*2+1)};
            endcase
            if(out_data!==expected)
                $fatal(1,"crop row phase tag=%d row=%d word=%d got=%h expected=%h",
                    tag,row_index,words,out_data,expected);
            words=words+1;
        end
    end
    function [7:0] ecc(input [23:0] h);
        begin
            ecc[0]=^(h&24'hf12cb7);ecc[1]=^(h&24'hf2555b);
            ecc[2]=^(h&24'h749a6d);ecc[3]=^(h&24'hb8e38e);
            ecc[4]=^(h&24'hdf03f0);ecc[5]=^(h&24'heffc00);ecc[7:6]=0;
        end
    endfunction
    task beat(input [15:0] v);
        begin @(negedge clk);valid=1;data=v;end
    endtask
    task gap;
        begin @(negedge clk);valid=0;repeat(30) @(negedge clk);end
    endtask
    task header(input [7:0] dtype,input [15:0] count);
        begin beat({dtype,count[7:0]});beat({count[15:8],ecc({count,dtype})});end
    endtask
    task raw_line(input [7:0] v);
        begin
            header(8'h2b,SW*10/8);
            for(integer j=0;j<SW/8;j=j+1) begin
                beat({v,v});beat({v,v});beat({8'he4,v});beat({v,v});beat({v,8'he4});
            end
            beat(16'hdead);
        end
    endtask
    task good_frame(input integer frame_tag,input bit explicit_fe);
        begin
            check_enable=0;words=0;lines=0;starts=0;ends=0;tag=frame_tag;
            header(0,frame_tag);gap; // boundary may close an old partial crop
            words=0;lines=0;starts=0;ends=0;check_enable=1;
            for(integer row=0;row<SH;row=row+1) begin raw_line(frame_tag*64+row);gap;end
            if(explicit_fe) begin header(1,frame_tag);gap;end
            if(words!=WORDS_PER_LINE*OH || lines!=OH || starts!=1 || ends!=1 || error)
                $fatal(1,"frame counts words=%d lines=%d FS=%d FE=%d parse_error=%b",
                    words,lines,starts,ends,error);
            $display("PASS source FS realigns crop tag=%0d bytes=%0d lines=%0d explicit_FE=%b",
                frame_tag,words*2,lines,explicit_fe);
            check_enable=0;
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);rst=0;gap;
        // No initial FS: simulate powering up in the middle of a source frame.
        for(integer row=0;row<5;row=row+1) begin raw_line(8'hf0+row);gap;end
        good_frame(1,1);
        // An incomplete previous frame must not shift the following crop.
        for(integer row=0;row<3;row=row+1) begin raw_line(8'he0+row);gap;end
        good_frame(2,0);
        if(scale_error) $fatal(1,"scaler format error");
        $display("PASS parser/source boundary/crop spatial regression");$finish;
    end
    initial begin #200000000;$fatal(1,"timeout");end
endmodule
