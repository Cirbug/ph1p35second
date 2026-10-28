
module mipi_dphy_tx_over_ph1p_mipiio_wrapper #(
    parameter DPHY_TX_LOCATION      = "DPHY0",
    
    parameter BYTE_NUM              = 1,
    parameter LANE_NUM              = 4,
	parameter CLK_LANE_MODE         = "CONTINUE", ///"CONTINUE","INTERRUPT"
  
    parameter HS_OUTPUT_VOD         = "250mV",
    parameter HS_OUTPUT_IMPEDANCE   = "50ohm",
    parameter HS_OUTPUT_DE_EMPHASIS = "0dB",
    parameter CK_OUTPUT_DELAY       = 0,
    parameter L0_OUTPUT_DELAY       = 0,
    parameter L1_OUTPUT_DELAY       = 0,
    parameter L2_OUTPUT_DELAY       = 0,
    parameter L3_OUTPUT_DELAY       = 0,
  
    parameter DPLL_MULTI_RATIO      = 19,
    parameter DPLL_DIV_RATIO        = 1,
  
    parameter T_DATA_LPX            = 4,
    parameter T_DATA_HS_PREPARE     = 4,
    parameter T_DATA_HS_ZERO        = 10,
    parameter T_DATA_HS_TRAIL       = 10,
    parameter T_CLK_LPX             = 4,
    parameter T_CLK_PREPARE         = 4,
    parameter T_CLK_ZERO            = 10,
    parameter T_CLK_PRE             = 10,
    parameter T_CLK_POST            = 10,
    parameter T_CLK_TRAIL           = 10
    )(
    input wire                          I_dphy_pll_ref_clk,
    input wire                          I_rst,

    output wire                         O_dpll_locked,
           
    output wire                         O_hs_tx_clk,
    input wire                          I_hs_tx_valid,
    input wire[LANE_NUM*BYTE_NUM*8-1:0] I_hs_tx_data, 
    input wire                          I_hs_tx_last,
    output wire                         O_hs_tx_ready,  

    input wire                          I_lp_tx_p,
    input wire                          I_lp_tx_n,

    input wire                          I_lp_rx_en,
    output wire                         O_lp_rx_p,
    output wire                         O_lp_rx_n,

    inout wire                          IO_tx_clk_pad_n, 
    inout wire                          IO_tx_clk_pad_p, 
    inout wire[3:0]                     IO_tx_data_pad_n,
    inout wire[3:0]                     IO_tx_data_pad_p        
);

    wire                          S_clk_hs_en;
    wire[BYTE_NUM*8-1:0]          S_clk_hs_data;
    wire                          S_clk_lp_p;
    wire                          S_clk_lp_n;

    wire                          S_data_hs_en;
    wire[LANE_NUM*BYTE_NUM*8-1:0] S_data_hs_data;
    wire[LANE_NUM-1:0]            S_data_lp_p;
    wire[LANE_NUM-1:0]            S_data_lp_n;
    wire[LANE_NUM-1:0]            S_mipiio_data_lp_p;
    wire[LANE_NUM-1:0]            S_mipiio_data_lp_n;


    hs_tx_wrapper#(
        .LANE_NUM          ( LANE_NUM          ),
        .BYTE_NUM          ( BYTE_NUM          ),
        .CLK_LANE_MODE     ( CLK_LANE_MODE     ),

        .T_DATA_LPX        ( T_DATA_LPX        ),
        .T_DATA_HS_PREPARE ( T_DATA_HS_PREPARE ),
        .T_DATA_HS_ZERO    ( T_DATA_HS_ZERO    ),
        .T_DATA_HS_TRAIL   ( T_DATA_HS_TRAIL   ),
        .T_CLK_LPX         ( T_CLK_LPX         ),
        .T_CLK_PREPARE     ( T_CLK_PREPARE     ),
        .T_CLK_ZERO        ( T_CLK_ZERO        ),
        .T_CLK_PRE         ( T_CLK_PRE         ),
        .T_CLK_POST        ( T_CLK_POST        ),
        .T_CLK_TRAIL       ( T_CLK_TRAIL       )
    )u_hs_tx_wrapper(
        .I_clk             ( O_hs_tx_clk       ),
        .I_rst             ( I_rst             ),

        .I_tx_valid        ( I_hs_tx_valid     ),
        .I_tx_data         ( I_hs_tx_data      ),
        .I_tx_last         ( I_hs_tx_last      ),
        .O_tx_ready        ( O_hs_tx_ready     ),

        .O_clk_hs_en       ( S_clk_hs_en       ),
        .O_clk_hs_data     ( S_clk_hs_data     ),
        .O_clk_lp_p        ( S_clk_lp_p        ),
        .O_clk_lp_n        ( S_clk_lp_n        ),

        .O_data_hs_en      ( S_data_hs_en      ),
        .O_data_hs_data    ( S_data_hs_data    ),
        .O_data_lp_p       ( S_data_lp_p       ),
        .O_data_lp_n       ( S_data_lp_n       )
    );


    genvar i;
    generate begin
        for(i = 0; i < LANE_NUM; i = i + 1) begin : MIPI_LANES
            if(i == 0)
                begin
                    assign S_mipiio_data_lp_p[i] = S_data_lp_p[i] & I_lp_tx_p;
                    assign S_mipiio_data_lp_n[i] = S_data_lp_n[i] & I_lp_tx_n;
                end 
            else
                begin
                    assign S_mipiio_data_lp_p[i] = S_data_lp_p[i];
                    assign S_mipiio_data_lp_n[i] = S_data_lp_n[i];
                end 
        end
    end
	endgenerate


    ph1p_mipiio_tx_wrapper#(
        .LANE_NUM                 ( LANE_NUM              ),
        .BYTE_NUM                 ( BYTE_NUM              ),
        .DPHY_TX_LOCATION         ( DPHY_TX_LOCATION      ),
        .HS_OUTPUT_VOD            ( HS_OUTPUT_VOD         ),
        .HS_OUTPUT_IMPEDANCE      ( HS_OUTPUT_IMPEDANCE   ),
        .HS_OUTPUT_DE_EMPHASIS    ( HS_OUTPUT_DE_EMPHASIS ),
        .CK_OUTPUT_DELAY          ( CK_OUTPUT_DELAY       ),
        .L0_OUTPUT_DELAY          ( L0_OUTPUT_DELAY       ),
        .L1_OUTPUT_DELAY          ( L1_OUTPUT_DELAY       ),
        .L2_OUTPUT_DELAY          ( L2_OUTPUT_DELAY       ),
        .L3_OUTPUT_DELAY          ( L3_OUTPUT_DELAY       )
    )u_ph1p_mipiio_tx_wrapper(
        .I_mipiio_pll_ref_clk     ( I_dphy_pll_ref_clk ),
        .O_hs_tx_clk              ( O_hs_tx_clk        ),
        .I_rst                    ( I_rst              ),

        .I_mipiio_pll_multi_ratio ( DPLL_MULTI_RATIO   ),
        .I_mipiio_pll_div_ratio   ( DPLL_DIV_RATIO     ),
        .O_mipiio_pll_locked      ( O_dpll_locked      ),

        .I_clk_hs_en              ( S_clk_hs_en        ),
        .I_clk_hs_data            ( S_clk_hs_data      ),
        .I_clk_lp_p               ( S_clk_lp_p         ),
        .I_clk_lp_n               ( S_clk_lp_n         ),

        .I_data_hs_en             ( S_data_hs_en       ),
        .I_data_hs_data           ( S_data_hs_data     ),
        .I_data_lp_p              ( S_mipiio_data_lp_p ),
        .I_data_lp_n              ( S_mipiio_data_lp_n ),

        .I_lp_l0_rx_en            ( I_lp_rx_en         ),
        .O_lp_l0_rx_p             ( O_lp_rx_p          ),
        .O_lp_l0_rx_n             ( O_lp_rx_n          ),

        .IO_tx_clk_pad_n          ( IO_tx_clk_pad_n    ),
        .IO_tx_clk_pad_p          ( IO_tx_clk_pad_p    ),
        .IO_tx_data_pad_n         ( IO_tx_data_pad_n   ),
        .IO_tx_data_pad_p         ( IO_tx_data_pad_p   )
    );




endmodule
