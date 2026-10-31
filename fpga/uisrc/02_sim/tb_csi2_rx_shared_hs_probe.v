`timescale 1ns/1ps
// Regression for separate-HS and shared-HS packet framing.
module tb_csi2_rx_shared_hs_probe;
    reg clk=0,rst=1,valid=0;
    reg [15:0] data=0;
    always #5 clk=~clk;
    wire wr,fs,fe,ending;
    integer words=0,starts=0,ends=0,lines=0;
    csi2_rx_parse dut(.I_clk(clk),.I_rst(rst),.I_hs_valid(valid),.I_hs_data(data),
        .O_wr_en(wr),.O_wr_data(),.O_pkt_end(ending),.O_pkt_wc(),
        .O_pkt_dt(),.O_frame_start(fs),.O_frame_end(fe),.O_wc_err());
    always @(posedge clk) if(!rst) begin
        if(wr)words=words+1;
        if(fs)starts=starts+1;
        if(fe)ends=ends+1;
        if(ending)lines=lines+1;
    end
    task beat(input [15:0] value);
        begin @(negedge clk);valid=1;data=value;end
    endtask
    task gap;
        begin @(negedge clk);valid=0;repeat(8) @(negedge clk);end
    endtask
    task start_packet;
        begin beat(16'h0000);beat(16'h0000);end
    endtask
    task end_packet;
        // FE DI=01, data=0, ECC=07 (valid CSI-2 header).
        begin beat(16'h0100);beat(16'h0007);end
    endtask
    task raw_packet;
        begin
            // RAW10 WC=10, ECC=2E.
            beat(16'h2b0a);beat(16'h002e);
            for(integer i=0;i<5;i=i+1)beat(16'h1234+i);
            beat(16'h0000); // CRC beat, value unused by current parser.
        end
    endtask
    initial begin
        repeat(4) @(negedge clk);rst=0;gap;
        start_packet;gap;raw_packet;gap;end_packet;gap;
        if(starts!=1 || ends!=1 || words!=5 || lines!=1)
            $fatal(1,"baseline failed FS=%0d FE=%0d words=%0d lines=%0d",starts,ends,words,lines);
        $display("PASS separate HS: FS=%0d FE=%0d words=%0d lines=%0d",starts,ends,words,lines);
        starts=0;ends=0;words=0;lines=0;
        start_packet;raw_packet;end_packet;gap;
        $display("OBSERVED shared HS: FS=%0d FE=%0d words=%0d lines=%0d",starts,ends,words,lines);
        if(starts!=1 || ends!=1 || words!=5 || lines!=1)
            $fatal(1,"shared-HS packet loss");
        $display("PASS shared HS");$finish;
    end
endmodule
