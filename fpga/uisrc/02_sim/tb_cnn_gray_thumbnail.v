`timescale 1ns/1ps
module tb_cnn_gray_thumbnail;
    reg wr_clk=0,rd_clk=0,rst=1,frame_start=0,accept=0,rd_en=0;
    reg [9:0] x=0;
    reg [8:0] y=0;
    reg [7:0] gray=0;
    reg [11:0] addr=0;
    wire [7:0] value;
    integer xx,yy,i;
    always #5 wr_clk=~wr_clk;
    always #7 rd_clk=~rd_clk;
    cnn_gray_thumbnail dut(.wr_clk(wr_clk),.rd_clk(rd_clk),.rst(rst),
        .frame_start(frame_start),.pixel_accept(accept),.pixel_x(x),.pixel_y(y),
        .gray_data(gray),.rd_en(rd_en),.rd_addr(addr),.rd_data(value));
    initial begin
        repeat(4) @(negedge wr_clk); rst=0;
        for(yy=0;yy<180;yy=yy+1) for(xx=0;xx<320;xx=xx+1) begin
            x=xx; y=yy; gray=(xx+3*yy)&255; accept=1;
            frame_start=(xx==0 && yy==0); @(negedge wr_clk);
        end
        accept=0; frame_start=0;
        if(dut.wr_addr!=3600) $fatal(1,"thumbnail count %0d",dut.wr_addr);
        // An unaccepted next frame must not replace the held snapshot.
        frame_start=1; @(negedge wr_clk); frame_start=0;
        gray=0; x=3; y=3; repeat(10) @(negedge wr_clk);
        for(i=0;i<3600;i=i+1) begin
            @(negedge rd_clk); addr=i; rd_en=1;
            @(negedge rd_clk); rd_en=0;
            if(value !== (((4*(i%80)+3)+3*(4*(i/80)+3))&255))
                $fatal(1,"thumbnail address=%0d got=%0d",i,value);
        end
        $display("PASS gray thumbnail: 3600 samples, accepted frame only, both RAM banks, asynchronous read");
        $finish;
    end
    initial begin #3000000; $fatal(1,"timeout"); end
endmodule
