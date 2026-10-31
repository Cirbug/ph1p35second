`timescale 1ns / 1ps

module tb_ae_i2c_link;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg [15:0] exposure = 16'd3990;
    reg [15:0] gain = 16'd80;
    reg [15:0] vts = 16'd2000;
    wire [15:0] sent_count;
    wire [15:0] nack_count;
    wire [15:0] packet_count;
    wire [15:0] error_count;
    wire [15:0] cfg_ae;
    wire [15:0] cfg_ag;
    wire [15:0] cfg_vts;
    wire cfg_req;
    wire remote_active;
    tri1 scl;
    tri1 sda;

    always #5 clk = ~clk;

    ae_control_i2c_master #(
        .CLK_HZ(1000000), .I2C_HZ(100000), .RETRY_TICKS(1000), .SLAVE_ADDRESS(7'h2A)
    ) master (
        .clk(clk), .rst_n(rst_n),
        .exposure(exposure), .gain(gain), .vts(vts),
        .scl(scl), .sda(sda),
        .sent_count(sent_count), .nack_count(nack_count)
    );

    ae_i2c_control #(
        .SLAVE_ADDRESS(7'h2A), .DEFAULT_VTS(16'd2000)
    ) slave (
        .clk(clk), .rst_n(rst_n), .scl(scl), .sda(sda),
        .local_req(1'b0), .local_ae(16'd3000), .local_ag(16'd60),
        .cam_cfg_done(1'b1), .ae_cfg_done(1'b1),
        .cfg_req(cfg_req), .cfg_ae(cfg_ae), .cfg_ag(cfg_ag), .cfg_vts(cfg_vts),
        .remote_active(remote_active), .packet_count(packet_count), .error_count(error_count)
    );

    initial begin
        repeat(8) @(posedge clk);
        rst_n = 1'b1;

        wait(packet_count == 1);
        wait(sent_count == 1);
        repeat(8) @(posedge clk);
        if(!remote_active || cfg_ae !== 16'd3990 || cfg_ag !== 16'd80 || cfg_vts !== 16'd2000)
            $fatal(1, "initial control frame mismatch ae=%0d ag=%0d vts=%0d", cfg_ae, cfg_ag, cfg_vts);

        exposure = 16'd3512;
        gain = 16'd91;
        vts = 16'd2200;
        wait(packet_count == 2);
        wait(sent_count == 2);
        repeat(8) @(posedge clk);
        if(cfg_ae !== 16'd3512 || cfg_ag !== 16'd91 || cfg_vts !== 16'd2200)
            $fatal(1, "updated control frame mismatch ae=%0d ag=%0d vts=%0d", cfg_ae, cfg_ag, cfg_vts);

        gain = 16'd150;
        wait(error_count != 0);
        if(packet_count != 2)
            $fatal(1, "invalid gain was accepted, packets=%0d", packet_count);

        gain = 16'd85;
        wait(packet_count == 3);
        wait(sent_count == 3);
        repeat(8) @(posedge clk);
        if(cfg_ag !== 16'd85 || nack_count == 0)
            $fatal(1, "retry after NACK failed ag=%0d nacks=%0d", cfg_ag, nack_count);

        $display("PASS A-to-B I2C sideband frame, CRC/range NACK, and retry");
        $finish;
    end

    initial begin
        #10000000;
        $fatal(1, "test timeout");
    end
endmodule
