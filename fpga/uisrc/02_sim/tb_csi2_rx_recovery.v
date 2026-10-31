`timescale 1ns/1ps
module tb_csi2_rx_recovery;
    reg clk=0,rst=1,valid=0;
    reg [15:0] data=0;
    always #5 clk=~clk;
    wire wr,fs,fe,ending,error;
    wire [15:0] payload;
    integer words=0,starts=0,ends=0,lines=0;
    csi2_rx_parse dut(.I_clk(clk),.I_rst(rst),.I_hs_valid(valid),.I_hs_data(data),
        .O_wr_en(wr),.O_wr_data(payload),.O_pkt_end(ending),.O_pkt_wc(),
        .O_pkt_dt(),.O_frame_start(fs),.O_frame_end(fe),.O_wc_err(error));
    always @(posedge clk) begin
        if(rst) begin words=0;starts=0;ends=0;lines=0;end
        else begin
            if(wr) begin
                if(payload<16'h1200 || payload>16'h1204) $fatal(1,"header/CRC/other VC leaked into pixels");
                words=words+1;
            end
            if(fs)starts=starts+1;
            if(fe)ends=ends+1;
            if(ending)lines=lines+1;
        end
    end
    task beat(input [15:0] v);
        begin @(negedge clk);valid=1;data=v;end
    endtask
    task gap;
        begin @(negedge clk);valid=0;repeat(8) @(negedge clk);end
    endtask
    task reset_case;
        begin @(negedge clk);rst=1;valid=0;repeat(4) @(negedge clk);rst=0;gap;end
    endtask
    function [7:0] ecc(input [23:0] h);
        reg [7:0] e;
        begin
            e=0;
            for(integer i=0;i<24;i=i+1) if(h[i]) begin
                case(i)
                    0:e=e^8'h07;1:e=e^8'h0b;2:e=e^8'h0d;3:e=e^8'h0e;
                    4:e=e^8'h13;5:e=e^8'h15;6:e=e^8'h16;7:e=e^8'h19;
                    8:e=e^8'h1a;9:e=e^8'h1c;10:e=e^8'h23;11:e=e^8'h25;
                    12:e=e^8'h26;13:e=e^8'h29;14:e=e^8'h2a;15:e=e^8'h2c;
                    16:e=e^8'h31;17:e=e^8'h32;18:e=e^8'h34;19:e=e^8'h38;
                    20:e=e^8'h1f;21:e=e^8'h2f;22:e=e^8'h37;23:e=e^8'h3b;
                endcase
            end
            ecc=e;
        end
    endfunction
    task header(input [7:0] dt,input [15:0] wc,input bit corrupt);
        begin beat({dt,wc[7:0]});beat({wc[15:8],ecc({wc,dt})^{7'd0,corrupt}});end
    endtask
    task raw_line;
        begin
            header(8'h2b,10,0);
            for(integer j=0;j<5;j=j+1)beat(16'h1200+j);
            beat(16'hdead);
        end
    endtask
    task check(input integer nw,input integer ns,input integer ne,input integer nl,input bit err);
        begin
            if(words!=nw || starts!=ns || ends!=ne || lines!=nl || error!==err)
                $fatal(1,"counts words=%d FS=%d FE=%d lines=%d error=%b expected=%d/%d/%d/%d/%b",
                    words,starts,ends,lines,error,nw,ns,ne,nl,err);
        end
    endtask
    initial begin
        reset_case;
        header(0,0,0);raw_line;raw_line;header(1,16'h4321,0);
        repeat(12)beat(0);repeat(12)beat(16'hffff);gap;
        check(10,1,1,2,0);
        $display("PASS shared FS/RAW/RAW/FE, nonzero short data, FE trail");

        reset_case;raw_line;
        repeat(12)beat(0);repeat(12)beat(16'hffff);gap;
        raw_line;repeat(12)beat(16'h00ff);gap;
        check(10,0,0,2,0);
        $display("PASS zero/FF trail creates no extra FS, FE or pixels");

        reset_case;header(8'h2b,10,1);
        for(integer j=0;j<5;j=j+1)beat(16'h1200+j);
        beat(16'hdead);gap;raw_line;gap;
        check(5,0,0,1,1);
        $display("PASS bad ECC rejected and next HS recovers");

        reset_case;beat(16'h2b0a);gap;raw_line;gap;
        check(5,0,0,1,1);
        $display("PASS interrupted header recovers");

        reset_case;header(8'h2b,10,0);beat(16'h1200);beat(16'h1201);gap;
        raw_line;gap;check(7,0,0,2,1);
        $display("PASS interrupted payload closes short line and does not swallow next header");

        reset_case;header(8'h2b,10,0);
        for(integer j=0;j<5;j=j+1)beat(16'h1200+j);
        gap;raw_line;gap;check(10,0,0,2,1);
        $display("PASS missing CRC recovered at next HS");

        reset_case;header(8'h12,4,0);beat(16'hbeef);beat(16'hcafe);beat(0);
        header(8'h6b,4,0);beat(16'hbeef);beat(16'hcafe);beat(0);
        header(2,16'h12,0);header(3,16'h12,0);raw_line;gap;
        check(5,0,0,1,0);
        $display("PASS embedded packets, other VC and LS/LE do not become RAW pixels");

        reset_case;header(8'h2b,9,0);beat(16'h1200);gap;
        raw_line;gap;check(5,0,0,1,1);
        $display("PASS odd WC rejected, parser recovers");
        $display("PASS CSI RX recovery regression");$finish;
    end
    initial begin #1000000;$fatal(1,"timeout");end
endmodule
