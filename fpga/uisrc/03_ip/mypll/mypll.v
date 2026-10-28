/************************************************************\
**	Copyright (c) 2012-2024 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Jun 21 2024 15:49:35
**	TD version	:	5.9.112797
///////////////////////////////////////////////////////////////////////////////
//	Input frequency:                100.0 MHZ
//	Clock multiplication factor: 2
//	Clock division factor:       1
//	Clock information:
//		Clock name	| Frequency 	| Phase shift 
//		C0        	| 200.000000   MHZ	    |  0 DEG 
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ns / 100 fs 

\************************************************************/
module mypll
(
  input                         refclk,
  output                        clk0_out,
  output                        lock,
  input                         pllreset
);
  wire							  clk0_buf;
DR1_LOGIC_BUFG bufg_feedback (
 .i(clk0_buf), 
 .o(clk0_out) 
 ); 








  DR1_PHY_PLL_WRAPERR_83aa86f42154
  #(
      .FBKCLK("CLKC0_EXT"),
      .REFCLK_DIV(1),
      .FBCLK_DIV(2),
      .CLKC0_FPHASE(0),
      .CLKC0_CPHASE(4),
      .CLKC0_DIV(5),
      .CLKC0_DUTY_INT(3),
      .CLKC0_ENABLE("ENABLE"),
      .FIN("100.0"),
      .FEEDBK_MODE("NORMAL"),
      .PLL_USR_RST("ENABLE"),
      .PLL_FEED_TYPE("EXTERNAL"),
      .LPF_RES(1),
      .LPF_CAP(2),
      .ICP_CUR(13),
      .GMC_GAIN(3),
      .FRAC_ENABLE("DISABLE"),
      .DITHER_ENABLE("DISABLE"),
      .SDM_FRAC(0),
      .SSC_AMP(0.0000),
      .MPHASE_ENABLE("DISABLE"),
      .DYN_PHASE_PATH_SEL("DISABLE"),
      .DYN_FPHASE_EN("DISABLE"),
      .CLKC0_FPHASE_RSTSEL(0),
      .CLKC0_DUTY50("ENABLE"),
      .INTPI(0),
      .SSC_ENABLE("DISABLE"),
      .SSC_MODE("DOWN"),
      .SSC_FREQ_DIV(0),
      .SSC_RNGE(0),
      .HIGH_SPEED_EN("DISABLE")
  )DR1_PHY_PLL_WRAPERR_83aa86f42154_Inst
  (
      .clk1_en(1'b0),
      .clk1_out(),
      .clkb1_out(),
      .clk2_en(1'b0),
      .clk2_out(),
      .clkb2_out(),
      .clk3_en(1'b0),
      .clk3_out(),
      .clkb3_out(),
      .clk4_en(1'b0),
      .clk4_out(),
      .clkb4_out(),
      .clk5_en(1'b0),
      .clk5_out(),
      .clkb5_out(),
      .clk6_en(1'b0),
      .clk6_out(),
      .clkb6_out(),
      .refclk(refclk),
      .ssc_en(1'b0),
      .ext_freq_mod_val(17'b00000000000000000),
      .ext_freq_mod_en(1'b0),
      .ext_freq_mod_clk(1'b0),
	  .clkc_rst(2'b00),
      .fbclk(clk0_out),
      .drp_rdata(),
      .drp_rdy(),
      .drp_err(),
      .drp_wdata(8'b00000000),
      .drp_addr(8'b00000000),
      .drp_wr(1'b0),
      .drp_rd(1'b0),
      .drp_sel(1'b0),
      .drp_rstn(1'b1),
      .drp_clk(1'b0),
      .cps_step(2'b00),
      .psclksel(3'b000),
      .psdone(),
      .psstep(1'b0),
      .psdown(1'b0),
      .psclk(1'b0),
      .pllpd(1'b0),
      .wakeup(1'b0),
      .refclk_rst(1'b0),
      .clk0_en(1'b1),
      .clkb0_out(),
      .clk0_out(clk0_buf),
      .lock(lock),
      .pllreset(pllreset)
  );
endmodule
