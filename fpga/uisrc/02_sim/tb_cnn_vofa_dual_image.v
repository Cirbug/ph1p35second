`timescale 1ns/1ps
module tb_cnn_vofa_dual_image;
    reg clk=0,rst_n=0,frame_done=0,mask_data=0;
    reg [7:0] thumb_data=0;
    wire mask_en,thumb_en,active,tx;
    wire [15:0] mask_addr;
    wire [11:0] thumb_addr;
    reg [199:0] header0="image:0,57600,320,180,24\n";
    reg [175:0] header1="image:1,3600,80,45,24\n";
    integer i,j,mask_count=0,thumb_count=0;
    reg [7:0] received;
    always #50 clk=~clk;
    always @(posedge clk) begin
        if(mask_en) begin
            if(mask_addr!==mask_count) $fatal(1,"mask address order");
            mask_data<=mask_addr[0]; mask_count=mask_count+1;
        end
        if(thumb_en) begin
            if(thumb_addr!==thumb_count) $fatal(1,"thumb address order");
            thumb_data<=thumb_addr[7:0]; thumb_count=thumb_count+1;
        end
    end
    cnn_vofa_image_uart #(.CLK_HZ(10000000),.BAUD(5000000),.THUMB_ENABLE(1)) dut (
        .clk(clk),.rst_n(rst_n),.frame_done(frame_done),.mask_data(mask_data),
        .mask_read_enable(mask_en),.mask_read_address(mask_addr),
        .thumb_data(thumb_data),.thumb_read_enable(thumb_en),.thumb_read_address(thumb_addr),
        .dump_active(active),.uart_tx(tx));
    task expect_byte;
        input [7:0] expected;
        begin
            @(negedge tx); #100;
            if(tx!==0) $fatal(1,"start bit");
            for(j=0;j<8;j=j+1) begin #200; received[j]=tx; end
            #200;
            if(tx!==1 || received!==expected) $fatal(1,"UART byte got=%02x expected=%02x",received,expected);
        end
    endtask
    initial begin
        repeat(5) @(negedge clk); rst_n=1;
        repeat(5) @(negedge clk); frame_done=1;
        @(negedge clk); frame_done=0;
        for(i=0;i<25;i=i+1) expect_byte(header0>>((24-i)*8));
        for(i=0;i<57600;i=i+1) expect_byte((i&1)?0:255);
        if(!active) $fatal(1,"mask released before thumbnail");
        for(i=0;i<22;i=i+1) expect_byte(header1>>((21-i)*8));
        for(i=0;i<3600;i=i+1) expect_byte(i&255);
        repeat(20) @(posedge clk);
        if(active || !tx || mask_count!=57600 || thumb_count!=3600) $fatal(1,"incomplete dump");
        $display("PASS dual UART images: 320x180 mask + 80x45 gray, exact headers/payloads/addresses, shared snapshot hold");
        $finish;
    end
    initial begin #200000000; $fatal(1,"timeout"); end
endmodule
