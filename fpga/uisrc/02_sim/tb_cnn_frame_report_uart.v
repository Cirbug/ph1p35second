`timescale 1ns/1ps
module tb_cnn_frame_report_uart;
    reg clk=0, rst_n=0, frame_done=0, digit_done=0, mask_data=0;
    reg [9:0] x=0,w=0;
    reg [8:0] y=0,h=0;
    reg [3:0] digit=0;
    reg [31:0] crc=0;
    reg [4:0] count=0;
    reg [7:0] thumb_data=0;
    wire mask_en,thumb_en,active,tx;
    wire [15:0] mask_addr;
    wire [11:0] thumb_addr;
    integer j,i,scenario,mask_count=0,thumb_count=0,fd,bytes=0;
    reg [7:0] received;
    always #50 clk=~clk;
    always @(posedge clk) begin
        if(mask_en) begin
            if(mask_addr !== mask_count%57600) $fatal(1,"mask address order");
            mask_data<=mask_addr[0]; mask_count=mask_count+1;
        end
        if(thumb_en) begin
            if(thumb_addr !== thumb_count%3600) $fatal(1,"thumbnail address order");
            thumb_data<=thumb_addr[7:0]; thumb_count=thumb_count+1;
        end
    end
    cnn_vofa_image_uart #(.CLK_HZ(10000000),.BAUD(5000000),
        .THUMB_ENABLE(1),.REPORT_ENABLE(1)) dut (
        .clk(clk),.rst_n(rst_n),.frame_done(frame_done),.digit_done(digit_done),
        .digit_x(x),.digit_y(y),.digit_w(w),.digit_h(h),.digit_class(digit),
        .digit_patch_crc(crc),.model_weight_crc(32'heec8e020),.frame_digit_count(count),
        .mask_data(mask_data),.mask_read_enable(mask_en),.mask_read_address(mask_addr),
        .thumb_data(thumb_data),.thumb_read_enable(thumb_en),.thumb_read_address(thumb_addr),
        .dump_active(active),.uart_tx(tx));
    task expect_byte(input [7:0] expected);
        begin
            @(negedge tx); #100;
            if(!active || tx!==0) $fatal(1,"snapshot hold/start bit");
            for(j=0;j<8;j=j+1) begin #200; received[j]=tx; end
            #200;
            if(tx!==1 || received!==expected)
                $fatal(1,"byte %0d got=%02x expected=%02x",bytes,received,expected);
            $fwrite(fd,"%c",received); bytes=bytes+1;
        end
    endtask
    task expect_text(input string message);
        integer k;
        begin
            for(k=0;k<message.len();k=k+1) begin
                if((message[0]=="F" || message[0]=="D" || message[0]=="E") &&
                   message[k]>="a" && message[k]<="f") expect_byte(message[k]-32);
                else expect_byte(message[k]);
            end
        end
    endtask
    task set_digit(input integer n);
        begin
            x=10+n; y=20+n; w=30+n; h=40+n; digit=n%10; crc=32'h12345600+n;
        end
    endtask
    task send_digit(input integer n,input integer last);
        begin
            @(negedge clk); set_digit(n); digit_done=1; frame_done=last;
            @(negedge clk); digit_done=0; frame_done=0;
        end
    endtask
    task expect_group(input [15:0] frame_id,input integer detected,input integer stored,input integer overflow);
        integer n;
        begin
            expect_text($sformatf("F,%04X,%02d,%02d,%01d,EEC8E020\n",frame_id,detected,stored,overflow));
            // Events during the held image must not create records or change frame ID.
            @(negedge clk); digit_done=1; frame_done=1;
            @(negedge clk); digit_done=0; frame_done=0;
            for(n=0;n<stored;n=n+1)
                expect_text($sformatf("D,%04X,%02d,%03d,%03d,%03d,%03d,%01d,%08X\n",
                    frame_id,n,10+n,20+n,30+n,40+n,n%10,32'h12345600+n));
            expect_text("image:0,57600,320,180,24\n");
            for(n=0;n<57600;n=n+1) expect_byte((n&1)?0:255);
            expect_text("image:1,3600,80,45,24\n");
            for(n=0;n<3600;n=n+1) expect_byte(n&255);
            expect_text($sformatf("E,%04X\n",frame_id));
            repeat(30) @(negedge clk);
            if(active || !tx) $fatal(1,"hold not released");
        end
    endtask
    initial begin
        fd=$fopen("r51_uart_sim.bin","wb");
        if(!fd) $fatal(1,"cannot open UART fixture");
        repeat(5) @(negedge clk); rst_n=1;
        count=3;
        send_digit(0,0); send_digit(1,0); send_digit(2,1);
        expect_group(1,3,3,0);
        count=0;
        @(negedge clk); frame_done=1;
        @(negedge clk); frame_done=0;
        expect_group(2,0,0,0);
        count=16;
        for(i=0;i<16;i=i+1) send_digit(i,i==15);
        expect_group(3,16,16,0);
        count=17;
        for(i=0;i<17;i=i+1) send_digit(i,i==16);
        expect_group(4,17,16,1);
        if(mask_count!=4*57600 || thumb_count!=4*3600) $fatal(1,"read count");
        $fclose(fd);
        $display("PASS R51: F/D/E bytes, 0/3/16/overflow, simultaneous digit/frame, busy events, both payloads and snapshot hold; bytes=%0d",bytes);
        $finish;
    end
    initial begin #800000000; $fatal(1,"timeout"); end
endmodule
