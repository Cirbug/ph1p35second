



module ph1p_mipiio_tx_wrapper #(
    parameter LANE_NUM              = 4,
    parameter BYTE_NUM              = 1,
    parameter DPHY_TX_LOCATION      = "DPHY0",
    parameter HS_OUTPUT_VOD         = "250mV",
    parameter HS_OUTPUT_IMPEDANCE   = "50ohm",
    parameter HS_OUTPUT_DE_EMPHASIS = "0dB",
    parameter CK_OUTPUT_DELAY       = 0, //0-7 
    parameter L0_OUTPUT_DELAY       = 0, //0-7 
    parameter L1_OUTPUT_DELAY       = 0, //0-7 
    parameter L2_OUTPUT_DELAY       = 0, //0-7 
    parameter L3_OUTPUT_DELAY       = 0  //0-7
)(
    input wire                          I_mipiio_pll_ref_clk,
    output wire                         O_hs_tx_clk,
    input wire                          I_rst,
    
    input wire[6:0]                     I_mipiio_pll_multi_ratio,
    input wire[5:0]                     I_mipiio_pll_div_ratio,
    output reg                          O_mipiio_pll_locked,

    input wire                          I_clk_hs_en,
    input wire[BYTE_NUM*8-1:0]          I_clk_hs_data,
    input wire                          I_clk_lp_p,
    input wire                          I_clk_lp_n,

    input wire                          I_data_hs_en,
    input wire[LANE_NUM*BYTE_NUM*8-1:0] I_data_hs_data,
    input wire[LANE_NUM-1:0]            I_data_lp_p,
    input wire[LANE_NUM-1:0]            I_data_lp_n,

    input wire                          I_lp_l0_rx_en,
    output wire                         O_lp_l0_rx_p,   
    output wire                         O_lp_l0_rx_n,

    inout wire                          IO_tx_clk_pad_n, 
    inout wire                          IO_tx_clk_pad_p, 
    inout wire[3:0]                     IO_tx_data_pad_n,
    inout wire[3:0]                     IO_tx_data_pad_p                 
);
    
	wire       S_hs_tx_clk;

    wire       S_mipiio_ck_hs_tx_en; //synthesis keep;
    wire[15:0] S_mipiio_ck_hs_tx_data;//synthesis keep;
    wire       S_mipiio_ck_lp_tx_en; //synthesis keep;
    wire       S_mipiio_ck_lp_tx_p;
    wire       S_mipiio_ck_lp_tx_n;

    wire       S_mipiio_l0_hs_tx_en; //synthesis keep;
    wire[15:0] S_mipiio_l0_hs_tx_data;//synthesis keep;
    wire       S_mipiio_l0_lp_tx_en; //synthesis keep;
    wire       S_mipiio_l0_lp_tx_p;
    wire       S_mipiio_l0_lp_tx_n;

    wire       S_mipiio_l1_hs_tx_en; //synthesis keep;
    wire[15:0] S_mipiio_l1_hs_tx_data;//synthesis keep;
    wire       S_mipiio_l1_lp_tx_en; //synthesis keep;
    wire       S_mipiio_l1_lp_tx_p;
    wire       S_mipiio_l1_lp_tx_n;

    wire       S_mipiio_l2_hs_tx_en; //synthesis keep;
    wire[15:0] S_mipiio_l2_hs_tx_data;//synthesis keep;
    wire       S_mipiio_l2_lp_tx_en; //synthesis keep;
    wire       S_mipiio_l2_lp_tx_p;
    wire       S_mipiio_l2_lp_tx_n;

    wire       S_mipiio_l3_hs_tx_en; //synthesis keep;
    wire[15:0] S_mipiio_l3_hs_tx_data;//synthesis keep;
    wire       S_mipiio_l3_lp_tx_en; //synthesis keep;
    wire       S_mipiio_l3_lp_tx_p;
    wire       S_mipiio_l3_lp_tx_n;

    wire       S_mipiio_pll_locked;

    reg[15:0]  S_mipiio_l0_data;
    reg[15:0]  S_mipiio_l1_data;
    reg[15:0]  S_mipiio_l2_data;
    reg[15:0]  S_mipiio_l3_data;

    reg[15:0]  S_mipiio_ck_data_negedge;
    reg[15:0]  S_mipiio_l0_data_negedge;
    reg[15:0]  S_mipiio_l1_data_negedge;
    reg[15:0]  S_mipiio_l2_data_negedge;
    reg[15:0]  S_mipiio_l3_data_negedge;
    


    localparam HS_P2S_RATIO = BYTE_NUM == 1 ? "8:1" : "16:1";


	PH1P_LOGIC_BUFG U_PH1P_LOGIC_BUFG (
 		.i(S_hs_tx_clk), 
 		.o(O_hs_tx_clk) 
 	); 


    generate 
        begin
            if(BYTE_NUM == 1)
                begin
                    if(LANE_NUM == 1)
                        begin
                            assign S_mipiio_ck_hs_tx_data = {8'd0,I_clk_hs_data};
                            assign S_mipiio_l0_hs_tx_data = {8'd0,I_data_hs_data[7:0]};
                            assign S_mipiio_l1_hs_tx_data = 16'd0;
                            assign S_mipiio_l2_hs_tx_data = 16'd0;
                            assign S_mipiio_l3_hs_tx_data = 16'd0;

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = 1'b0;
                            assign S_mipiio_l2_hs_tx_en   = 1'b0;
                            assign S_mipiio_l3_hs_tx_en   = 1'b0;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = 1'b1;
                            assign S_mipiio_l2_lp_tx_en = 1'b1;
                            assign S_mipiio_l3_lp_tx_en = 1'b1;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p;
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n;
                            assign S_mipiio_l1_lp_tx_p = 1'b0;
                            assign S_mipiio_l1_lp_tx_n = 1'b0; 
                            assign S_mipiio_l2_lp_tx_p = 1'b0;
                            assign S_mipiio_l2_lp_tx_n = 1'b0;
                            assign S_mipiio_l3_lp_tx_p = 1'b0;
                            assign S_mipiio_l3_lp_tx_n = 1'b0;
                        end
                    else if(LANE_NUM == 2)
                        begin
                            assign S_mipiio_ck_hs_tx_data = {8'd0,I_clk_hs_data};
                            assign S_mipiio_l0_hs_tx_data = {8'd0,I_data_hs_data[7:0]};
                            assign S_mipiio_l1_hs_tx_data = {8'd0,I_data_hs_data[15:8]};
                            assign S_mipiio_l2_hs_tx_data = 16'd0;
                            assign S_mipiio_l3_hs_tx_data = 16'd0;

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l2_hs_tx_en   = 1'b0;
                            assign S_mipiio_l3_hs_tx_en   = 1'b0;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l2_lp_tx_en = 1'b1;
                            assign S_mipiio_l3_lp_tx_en = 1'b1;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p[0];
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n[0];
                            assign S_mipiio_l1_lp_tx_p = I_data_lp_p[1];
                            assign S_mipiio_l1_lp_tx_n = I_data_lp_n[1]; 
                            assign S_mipiio_l2_lp_tx_p = 1'b0;
                            assign S_mipiio_l2_lp_tx_n = 1'b0;
                            assign S_mipiio_l3_lp_tx_p = 1'b0;
                            assign S_mipiio_l3_lp_tx_n = 1'b0;
                        end
                    else if(LANE_NUM == 3)
                        begin
                            assign S_mipiio_ck_hs_tx_data = {8'd0,I_clk_hs_data};
                            assign S_mipiio_l0_hs_tx_data = {8'd0,I_data_hs_data[7:0]};
                            assign S_mipiio_l1_hs_tx_data = {8'd0,I_data_hs_data[15:8]};
                            assign S_mipiio_l2_hs_tx_data = {8'd0,I_data_hs_data[23:16]};
                            assign S_mipiio_l3_hs_tx_data = 16'd0;

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l2_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l3_hs_tx_en   = 1'b0;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l2_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l3_lp_tx_en = 1'b1;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p[0];
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n[0];
                            assign S_mipiio_l1_lp_tx_p = I_data_lp_p[1];
                            assign S_mipiio_l1_lp_tx_n = I_data_lp_n[1]; 
                            assign S_mipiio_l2_lp_tx_p = I_data_lp_p[2];
                            assign S_mipiio_l2_lp_tx_n = I_data_lp_n[2];
                            assign S_mipiio_l3_lp_tx_p = 1'b0;
                            assign S_mipiio_l3_lp_tx_n = 1'b0;
                        end
                    else if(LANE_NUM == 4)
                        begin
                            assign S_mipiio_ck_hs_tx_data = {8'd0,I_clk_hs_data};
                            assign S_mipiio_l0_hs_tx_data = {8'd0,I_data_hs_data[7:0]};
                            assign S_mipiio_l1_hs_tx_data = {8'd0,I_data_hs_data[15:8]};
                            assign S_mipiio_l2_hs_tx_data = {8'd0,I_data_hs_data[23:16]};
                            assign S_mipiio_l3_hs_tx_data = {8'd0,I_data_hs_data[31:24]};

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l2_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l3_hs_tx_en   = I_data_hs_en;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l2_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l3_lp_tx_en = ~I_data_hs_en;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p[0];
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n[0];
                            assign S_mipiio_l1_lp_tx_p = I_data_lp_p[1];
                            assign S_mipiio_l1_lp_tx_n = I_data_lp_n[1]; 
                            assign S_mipiio_l2_lp_tx_p = I_data_lp_p[2];
                            assign S_mipiio_l2_lp_tx_n = I_data_lp_n[2];
                            assign S_mipiio_l3_lp_tx_p = I_data_lp_p[3];
                            assign S_mipiio_l3_lp_tx_n = I_data_lp_n[3];
                        end
                end
            else
                begin
                    if(LANE_NUM == 1)
                        begin
                            assign S_mipiio_ck_hs_tx_data = I_clk_hs_data;
                            assign S_mipiio_l0_hs_tx_data = I_data_hs_data[15:0];
                            assign S_mipiio_l1_hs_tx_data = 16'd0;
                            assign S_mipiio_l2_hs_tx_data = 16'd0;
                            assign S_mipiio_l3_hs_tx_data = 16'd0;

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = 1'b0;
                            assign S_mipiio_l2_hs_tx_en   = 1'b0;
                            assign S_mipiio_l3_hs_tx_en   = 1'b0;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = 1'b1;
                            assign S_mipiio_l2_lp_tx_en = 1'b1;
                            assign S_mipiio_l3_lp_tx_en = 1'b1;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p;
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n;
                            assign S_mipiio_l1_lp_tx_p = 1'b0;
                            assign S_mipiio_l1_lp_tx_n = 1'b0; 
                            assign S_mipiio_l2_lp_tx_p = 1'b0;
                            assign S_mipiio_l2_lp_tx_n = 1'b0;
                            assign S_mipiio_l3_lp_tx_p = 1'b0;
                            assign S_mipiio_l3_lp_tx_n = 1'b0;
                        end
                    else if(LANE_NUM == 2)
                        begin
                            assign S_mipiio_ck_hs_tx_data = I_clk_hs_data;
                            assign S_mipiio_l0_hs_tx_data = I_data_hs_data[15:0];
                            assign S_mipiio_l1_hs_tx_data = I_data_hs_data[31:16];
                            assign S_mipiio_l2_hs_tx_data = 16'd0;
                            assign S_mipiio_l3_hs_tx_data = 16'd0;

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l2_hs_tx_en   = 1'b0;
                            assign S_mipiio_l3_hs_tx_en   = 1'b0;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l2_lp_tx_en = 1'b1;
                            assign S_mipiio_l3_lp_tx_en = 1'b1;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p[0];
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n[0];
                            assign S_mipiio_l1_lp_tx_p = I_data_lp_p[1];
                            assign S_mipiio_l1_lp_tx_n = I_data_lp_n[1]; 
                            assign S_mipiio_l2_lp_tx_p = 1'b0;
                            assign S_mipiio_l2_lp_tx_n = 1'b0;
                            assign S_mipiio_l3_lp_tx_p = 1'b0;
                            assign S_mipiio_l3_lp_tx_n = 1'b0;
                        end
                    else if(LANE_NUM == 3)
                        begin
                            assign S_mipiio_ck_hs_tx_data = I_clk_hs_data;
                            assign S_mipiio_l0_hs_tx_data = I_data_hs_data[15:0];
                            assign S_mipiio_l1_hs_tx_data = I_data_hs_data[31:16];
                            assign S_mipiio_l2_hs_tx_data = I_data_hs_data[47:32];
                            assign S_mipiio_l3_hs_tx_data = 16'd0;

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l2_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l3_hs_tx_en   = 1'b0;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l2_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l3_lp_tx_en = 1'b1;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p[0];
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n[0];
                            assign S_mipiio_l1_lp_tx_p = I_data_lp_p[1];
                            assign S_mipiio_l1_lp_tx_n = I_data_lp_n[1]; 
                            assign S_mipiio_l2_lp_tx_p = I_data_lp_p[2];
                            assign S_mipiio_l2_lp_tx_n = I_data_lp_n[2];
                            assign S_mipiio_l3_lp_tx_p = 1'b0;
                            assign S_mipiio_l3_lp_tx_n = 1'b0;
                        end
                    else if(LANE_NUM == 4)
                        begin
                            assign S_mipiio_ck_hs_tx_data = I_clk_hs_data;
                            assign S_mipiio_l0_hs_tx_data = I_data_hs_data[15:0];
                            assign S_mipiio_l1_hs_tx_data = I_data_hs_data[31:16];
                            assign S_mipiio_l2_hs_tx_data = I_data_hs_data[47:32];
                            assign S_mipiio_l3_hs_tx_data = I_data_hs_data[63:48];

                            assign S_mipiio_ck_hs_tx_en   = I_clk_hs_en;
                            assign S_mipiio_l0_hs_tx_en   = I_lp_l0_rx_en ? 1'b0 : I_data_hs_en;
                            assign S_mipiio_l1_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l2_hs_tx_en   = I_data_hs_en;
                            assign S_mipiio_l3_hs_tx_en   = I_data_hs_en;

                            assign S_mipiio_ck_lp_tx_en = ~I_clk_hs_en;
                            assign S_mipiio_l0_lp_tx_en = I_lp_l0_rx_en ? 1'b0 : (~I_data_hs_en);
                            assign S_mipiio_l1_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l2_lp_tx_en = ~I_data_hs_en;
                            assign S_mipiio_l3_lp_tx_en = ~I_data_hs_en;

                            assign S_mipiio_ck_lp_tx_p = I_clk_lp_p;
                            assign S_mipiio_ck_lp_tx_n = I_clk_lp_n;
                            assign S_mipiio_l0_lp_tx_p = I_data_lp_p[0];
                            assign S_mipiio_l0_lp_tx_n = I_data_lp_n[0];
                            assign S_mipiio_l1_lp_tx_p = I_data_lp_p[1];
                            assign S_mipiio_l1_lp_tx_n = I_data_lp_n[1]; 
                            assign S_mipiio_l2_lp_tx_p = I_data_lp_p[2];
                            assign S_mipiio_l2_lp_tx_n = I_data_lp_n[2];
                            assign S_mipiio_l3_lp_tx_p = I_data_lp_p[3];
                            assign S_mipiio_l3_lp_tx_n = I_data_lp_n[3];
                        end
                end
        end
    endgenerate


    always @(posedge I_mipiio_pll_ref_clk or posedge I_rst) begin
        if(I_rst)
            O_mipiio_pll_locked <= 1'b0;
        else
            if(S_mipiio_pll_locked)
                O_mipiio_pll_locked <= 1'b1;
            else
                O_mipiio_pll_locked <= O_mipiio_pll_locked;
    end

//    always @(posedge I_mipiio_pll_ref_clk) begin
//    	O_mipiio_pll_locked <= S_mipiio_pll_locked;
//    end


//    always @(negedge O_hs_tx_clk) begin
//        S_mipiio_ck_data_negedge <= S_mipiio_ck_hs_tx_data;
//        S_mipiio_l0_data_negedge <= S_mipiio_l0_hs_tx_data;
//        S_mipiio_l1_data_negedge <= S_mipiio_l1_hs_tx_data; 
//        S_mipiio_l2_data_negedge <= S_mipiio_l2_hs_tx_data; 
//        S_mipiio_l3_data_negedge <= S_mipiio_l3_hs_tx_data; 
//    end

    always @(posedge O_hs_tx_clk) begin
        S_mipiio_ck_data_negedge <= S_mipiio_ck_hs_tx_data;
        S_mipiio_l0_data_negedge <= S_mipiio_l0_hs_tx_data;
        S_mipiio_l1_data_negedge <= S_mipiio_l1_hs_tx_data; 
        S_mipiio_l2_data_negedge <= S_mipiio_l2_hs_tx_data; 
        S_mipiio_l3_data_negedge <= S_mipiio_l3_hs_tx_data; 
    end

    PH1P_LOGIC_DPHY_MIPI_TX #(
        .DPHY_TX_LOCATION       ( DPHY_TX_LOCATION    ),   

        .HS_OUTPUT_IMPEDANCE    ( HS_OUTPUT_IMPEDANCE ),    
        .HS_P2S_RATIO           ( HS_P2S_RATIO        ),      
        .HS_OUTPUT_VOD          ( HS_OUTPUT_VOD       ),    

        .CK_LP_OUTPUT_IMPEDANCE ( "110ohm"              ),
        .CK_LP_OUTPUT_SLEW_RATE ( "FAST"                ),  
        .CK_HS_DE_EMPHASIS      ( HS_OUTPUT_DE_EMPHASIS ), 

        .L0_LP_OUTPUT_IMPEDANCE ( "110ohm"              ),
        .L0_LP_OUTPUT_SLEW_RATE ( "FAST"                ),  
        .L0_HS_DE_EMPHASIS      ( HS_OUTPUT_DE_EMPHASIS ), 

        .L1_LP_OUTPUT_IMPEDANCE ( "110ohm"              ),
        .L1_LP_OUTPUT_SLEW_RATE ( "FAST"                ),  
        .L1_HS_DE_EMPHASIS      ( HS_OUTPUT_DE_EMPHASIS ), 

        .L2_LP_OUTPUT_IMPEDANCE ( "110ohm"              ),
        .L2_LP_OUTPUT_SLEW_RATE ( "FAST"                ),  
        .L2_HS_DE_EMPHASIS      ( HS_OUTPUT_DE_EMPHASIS ), 
          
        .L3_LP_OUTPUT_IMPEDANCE ( "110ohm"              ),
        .L3_LP_OUTPUT_SLEW_RATE ( "FAST"                ),  
        .L3_HS_DE_EMPHASIS      ( HS_OUTPUT_DE_EMPHASIS )
    )u_PH1P_LOGIC_DPHY_MIPI_TX(
        .io_clk_pad_n            ( IO_tx_clk_pad_n          ),
        .io_clk_pad_p            ( IO_tx_clk_pad_p          ),
        .io_data_pad_n           ( IO_tx_data_pad_n         ),
        .io_data_pad_p           ( IO_tx_data_pad_p         ),
 
        .i_phy_dpll_ref_clk      ( I_mipiio_pll_ref_clk     ),
        .o_fabric_div4_8_clk     ( S_hs_tx_clk              ),
        .i_phy_dpll_rst_n        ( ~I_rst                   ),
        .i_phy_dpll_pwdn_n       ( 1'b1                     ),
        .i_phy_dpll_vco_bypass_n ( 1'b1                     ),
        .o_phy_dpll_locked       ( S_mipiio_pll_locked      ),
        .i_phy_dpll_multi_ratio  ( I_mipiio_pll_multi_ratio ),
        .i_phy_dpll_div_ratio    ( I_mipiio_pll_div_ratio   ),
        .i_phy_rst_n             ( O_mipiio_pll_locked      ),

        .i_ck_lp_tx_en           ( S_mipiio_ck_lp_tx_en     ),
        .i_ck_lp_tx_p            ( S_mipiio_ck_lp_tx_p      ),
        .i_ck_lp_tx_n            ( S_mipiio_ck_lp_tx_n      ),
        .i_ck_hs_tx_en           ( S_mipiio_ck_hs_tx_en     ),
        .i_ck_hs_odelay_value    ( CK_OUTPUT_DELAY          ),
        .i_ck_hs_tx_data         ( S_mipiio_ck_data_negedge ),
  
        .i_l0_lp_tx_en           ( S_mipiio_l0_lp_tx_en     ),
        .i_l0_lp_tx_p            ( S_mipiio_l0_lp_tx_p      ),
        .i_l0_lp_tx_n            ( S_mipiio_l0_lp_tx_n      ),
        .o_l0_lp_rx_p            ( O_lp_l0_rx_p             ),
        .o_l0_lp_rx_n            ( O_lp_l0_rx_n             ),
        .i_l0_hs_tx_en           ( S_mipiio_l0_hs_tx_en     ),
        .i_l0_hs_odelay_value    ( L0_OUTPUT_DELAY          ),
        .i_l0_hs_tx_data         ( S_mipiio_l0_data_negedge ),

        .i_l1_lp_tx_en           ( S_mipiio_l1_lp_tx_en     ),
        .i_l1_lp_tx_p            ( S_mipiio_l1_lp_tx_p      ),
        .i_l1_lp_tx_n            ( S_mipiio_l1_lp_tx_n      ),
        .o_l1_lp_rx_p            (  ),  
        .o_l1_lp_rx_n            (  ),  
        .i_l1_hs_tx_en           ( S_mipiio_l1_hs_tx_en     ),
        .i_l1_hs_odelay_value    ( L1_OUTPUT_DELAY          ),
        .i_l1_hs_tx_data         ( S_mipiio_l1_data_negedge ),
  
        .i_l2_lp_tx_en           ( S_mipiio_l2_lp_tx_en     ),
        .i_l2_lp_tx_p            ( S_mipiio_l2_lp_tx_p      ),
        .i_l2_lp_tx_n            ( S_mipiio_l2_lp_tx_n      ),
        .o_l2_lp_rx_p            (  ),  
        .o_l2_lp_rx_n            (  ),  
        .i_l2_hs_tx_en           ( S_mipiio_l2_hs_tx_en     ),
        .i_l2_hs_odelay_value    ( L2_OUTPUT_DELAY          ),
        .i_l2_hs_tx_data         ( S_mipiio_l2_data_negedge ),
  
        .i_l3_lp_tx_en           ( S_mipiio_l3_lp_tx_en     ),
        .i_l3_lp_tx_p            ( S_mipiio_l3_lp_tx_p      ),
        .i_l3_lp_tx_n            ( S_mipiio_l3_lp_tx_n      ),
        .o_l3_lp_rx_p            (  ),  
        .o_l3_lp_rx_n            (  ),  
        .i_l3_hs_tx_en           ( S_mipiio_l3_hs_tx_en     ),
        .i_l3_hs_odelay_value    ( L3_OUTPUT_DELAY          ),
        .i_l3_hs_tx_data         ( S_mipiio_l3_data_negedge )
    );


endmodule