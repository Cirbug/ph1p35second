`timescale 1ns/1ps
module tb_cnn_adaptive_binarize;
    reg clk=0, rst=1, valid=0, line_end=0, frame_end=0;
    reg [15:0] data=0;
    wire pixel_valid, frame_start, complete, error;
    wire [7:0] binary_data, gray_data;
    wire [9:0] x;
    wire [8:0] y;
    integer emitted=0, black=0, starts=0, completes=0;
    integer row, group_id, k, frame_id=0;
    reg [7:0] background=80;
    reg [7:0] bytes [0:49];
    always #5 clk=~clk;
    cnn_raw10_resize_bin #(.AUTO_THRESHOLD(1), .SRC_WIDTH(40),
        .SRC_HEIGHT(20), .DST_WIDTH(5), .DST_HEIGHT(4)) dut (
        .clk(clk), .rst(rst), .in_valid(valid), .in_data(data),
        .in_line_end(line_end), .in_frame_end(frame_end),
        .frame_start(frame_start), .pixel_valid(pixel_valid),
        .pixel_data(binary_data), .gray_pixel_data(gray_data),
        .frame_complete(complete), .pixel_x(x), .pixel_y(y), .format_error(error)
    );
    always @(posedge clk) if(!rst) begin
        if(frame_start) starts=starts+1;
        if(complete) completes=completes+1;
        if(pixel_valid) begin
            if(x !== emitted%5 || y !== emitted/5) $fatal(1,"coordinate mismatch");
            if(gray_data !== ((x==0) ? 8'd20 : background)) $fatal(1,"gray sample mismatch");
            if(binary_data==0) black=black+1;
            else if(binary_data!==255) $fatal(1,"non-binary output");
            emitted=emitted+1;
        end
    end
    task send_frame;
        input [7:0] bg;
        input integer expected_black;
        input [7:0] expected_next_threshold;
        begin
            background=bg; emitted=0; black=0;
            for(row=0;row<20;row=row+1) begin
                for(group_id=0;group_id<10;group_id=group_id+1) begin
                    for(k=0;k<4;k=k+1)
                        bytes[group_id*5+k]=(group_id<2) ? 8'd20 : bg;
                    bytes[group_id*5+4]=0;
                end
                for(k=0;k<25;k=k+1) begin
                    valid=1; data={bytes[2*k],bytes[2*k+1]}; @(negedge clk);
                end
                valid=0; line_end=1; @(negedge clk); line_end=0;
            end
            repeat(4) @(negedge clk);
            frame_end=1; @(negedge clk); frame_end=0;
            repeat(150) @(negedge clk);
            frame_id=frame_id+1;
            if(emitted!=20 || black!=expected_black || error ||
               starts!=frame_id || completes!=frame_id)
                $fatal(1,"frame %0d pixels=%0d black=%0d expected=%0d",frame_id,emitted,black,expected_black);
            if(dut.adaptive_threshold!==expected_next_threshold)
                $fatal(1,"threshold %0d expected %0d",dut.adaptive_threshold,expected_next_threshold);
        end
    endtask
    initial begin
        repeat(5) @(negedge clk); rst=0;
        send_frame(80,20,63); // initial 128 is too high for this exposure
        send_frame(80,4,63);  // adaptation recovers dark ink on white paper
        send_frame(32,20,27); // sudden darker exposure, previous-frame latency
        send_frame(32,4,27);
        send_frame(200,4,153);
        send_frame(200,4,153);
        $display("PASS adaptive binarize: fixed frame threshold, RAW gray preserved, bright/dark exposure recovery");
        $finish;
    end
    initial begin #2000000; $fatal(1,"timeout"); end
endmodule
