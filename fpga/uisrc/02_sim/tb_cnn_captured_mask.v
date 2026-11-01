`timescale 1ns/1ps
// Offline replay of a UART-captured mask into the current detector/CNN RTL.
// This verifies RTL with the captured input; it is not a hardware class trace.
module tb_cnn_captured_mask;
    reg capture_clk=0,infer_clk=0,rst=1;
    reg frame_start=0,pixel_valid=0,frame_complete=0;
    reg [7:0] pixel_data=255;
    reg [7:0] pixels [0:57599];
    reg [2047:0] image_path;
    wire done,frame_done,busy,weight_ready;
    wire [3:0] class_id;
    wire [9:0] x,w;
    wire [8:0] y,h;
    wire [4:0] count;
    wire [31:0] crc,patch_crc;
    integer i,reported=0;
    always #3.75 capture_clk=~capture_clk;
    always #5 infer_clk=~infer_clk;
    cnn_frame_digit_detector #(.MAX_BANDS(8),.MAX_DIGITS(16)) dut (
        .capture_clk(capture_clk),.infer_clk(infer_clk),.rst(rst),
        .frame_start(frame_start),.pixel_valid(pixel_valid),.pixel_data(pixel_data),
        .frame_complete(frame_complete),.image_dump_active(1'b0),
        .image_dump_read_enable(1'b0),.image_dump_read_address(16'd0),
        .digit_done(done),.digit_class(class_id),.digit_x(x),.digit_y(y),
        .digit_w(w),.digit_h(h),.frame_done(frame_done),.frame_digit_count(count),
        .busy(busy),.model_check_done(weight_ready),.model_weight_crc32(crc),
        .digit_patch_crc32(patch_crc)
    );
    always @(posedge infer_clk) begin
        if(done) begin
            reported=reported+1;
            $display("RTL_DIGIT x=%0d y=%0d w=%0d h=%0d class=%0d patch_crc=%08x",x,y,w,h,class_id,patch_crc);
        end
        if(frame_done) begin
            $display("RTL_FRAME_DONE count=%0d reported=%0d weight_crc=%08x",count,reported,crc);
            $finish;
        end
    end
    initial begin
        if(!$value$plusargs("IMAGE=%s",image_path)) $fatal(1,"Use +IMAGE=<57600-byte hex file>");
        $readmemh(image_path,pixels);
        repeat(20) @(negedge infer_clk); rst=0;
        wait(weight_ready && !busy);
        repeat(20) @(negedge capture_clk);
        for(i=0;i<57600;i=i+1) begin
            pixel_valid=1; frame_start=(i==0); pixel_data=pixels[i];
            @(negedge capture_clk);
        end
        pixel_valid=0; frame_start=0;
        repeat(10) @(negedge capture_clk);
        frame_complete=1; @(negedge capture_clk); frame_complete=0;
    end
    initial begin #500000000; $fatal(1,"RTL replay timeout"); end
endmodule
