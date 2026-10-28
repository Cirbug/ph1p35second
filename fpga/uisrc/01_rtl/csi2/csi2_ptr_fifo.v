//==========================================================================
// csi2_ptr_fifo.v
//   异步 FIFO（格雷码指针），用于跨 CSI-2 接收时钟域与发送时钟域。
//
//   宽度 18bit 的用途：{type[1:0], data[15:0]}
//     type 0 = 载荷 word
//     type 1 = frame_start
//     type 2 = pkt_end（data 字段带 WC）
//     type 3 = frame_end
//
//   ⚠ 关键实现细节（曾经踩过的坑）：
//     full / empty 必须"打一拍寄存"。若直接用组合的 wgray_next 去算 full，
//     而 wbin_next 又依赖 full，就会形成组合逻辑环（TD 报 SYN-5033），
//     并进一步导致 SYN-8613 "Couldn't find D_NET" 综合失败。
//
//   读侧是组合读（first-word-fall-through），小深度下综合成 LUTRAM。
//==========================================================================
module csi2_ptr_fifo #(
    parameter DW = 18,
    parameter AW = 10
)(
    // 写侧
    input  wire          I_wclk,
    input  wire          I_wrstn,
    input  wire          I_we,
    input  wire [DW-1:0] I_wdata,
    output reg           O_wfull,
    // 读侧
    input  wire          I_rclk,
    input  wire          I_rrstn,
    input  wire          I_re,
    output wire [DW-1:0] O_rdata,
    output reg           O_rempty
);

    reg [DW-1:0] mem [0:(1<<AW)-1];

    reg [AW:0] wbin, wgray;
    reg [AW:0] rbin, rgray;

    reg [AW:0] wq1_rgray, wq2_rgray;   // 读指针同步到写域
    reg [AW:0] rq1_wgray, rq2_wgray;   // 写指针同步到读域

    //------------------------------------------------------------------
    // 写侧
    //------------------------------------------------------------------
    wire [AW:0] wbin_next  = wbin + { {AW{1'b0}}, (I_we & ~O_wfull) };
    wire [AW:0] wgray_next = (wbin_next >> 1) ^ wbin_next;

    // 与经典的 Cummings 实现一致：用"下一个"写指针判断满
    wire        wfull_val  = (wgray_next == { ~wq2_rgray[AW:AW-1], wq2_rgray[AW-2:0] });

    always @(posedge I_wclk) begin
        if (I_we && !O_wfull)
            mem[wbin[AW-1:0]] <= I_wdata;
    end

    always @(posedge I_wclk or negedge I_wrstn) begin
        if (!I_wrstn) begin
            wbin   <= {(AW+1){1'b0}};
            wgray  <= {(AW+1){1'b0}};
            O_wfull<= 1'b0;
        end else begin
            wbin   <= wbin_next;
            wgray  <= wgray_next;
            O_wfull<= wfull_val;
        end
    end

    always @(posedge I_wclk or negedge I_wrstn) begin
        if (!I_wrstn) begin
            wq1_rgray <= {(AW+1){1'b0}};
            wq2_rgray <= {(AW+1){1'b0}};
        end else begin
            wq1_rgray <= rgray;
            wq2_rgray <= wq1_rgray;
        end
    end

    //------------------------------------------------------------------
    // 读侧
    //------------------------------------------------------------------
    wire [AW:0] rbin_next  = rbin + { {AW{1'b0}}, (I_re & ~O_rempty) };
    wire [AW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;

    wire        rempty_val = (rgray_next == rq2_wgray);

    always @(posedge I_rclk or negedge I_rrstn) begin
        if (!I_rrstn) begin
            rbin    <= {(AW+1){1'b0}};
            rgray   <= {(AW+1){1'b0}};
            O_rempty<= 1'b1;
        end else begin
            rbin    <= rbin_next;
            rgray   <= rgray_next;
            O_rempty<= rempty_val;
        end
    end

    always @(posedge I_rclk or negedge I_rrstn) begin
        if (!I_rrstn) begin
            rq1_wgray <= {(AW+1){1'b0}};
            rq2_wgray <= {(AW+1){1'b0}};
        end else begin
            rq1_wgray <= wgray;
            rq2_wgray <= rq1_wgray;
        end
    end

    assign O_rdata = mem[rbin[AW-1:0]];

endmodule
