`timescale 1ns / 1ps
/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2019/12/17
*Module Name:HDMI_IN_Test
*File Name:HDMI_IN_Test.v
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/

/*******************************ui7611reg***************************
--1.ADV7611–æ∆¨≈‰÷√ºƒ¥Ê∆˜≈‰÷√±Ì
*********************************************************************/

module uics500reg
(
input      [8 :0]  REG_INDEX,
output reg [31:0]  REG_DATA,
output     [8 :0]  REG_SIZE  
);

assign	REG_SIZE = 115 + 1;
//-----------------------------------------------------------------
/////////////////////   Config Data LUT   //////////////////////////    
always@(*)
begin
    case(REG_INDEX)
		0:	REG_DATA = {16'h0103, 8'h01}; 
		1:	REG_DATA = {16'h0100, 8'h00}; 
		2:	REG_DATA = {16'h36e9, 8'h80}; 
		3:	REG_DATA = {16'h36f9, 8'h80}; 
		4:	REG_DATA = {16'h36ea, 8'h3c}; 
		5:	REG_DATA = {16'h36eb, 8'h0c}; 
		6:	REG_DATA = {16'h36ec, 8'h0b}; 
		7:	REG_DATA = {16'h36ed, 8'h26}; 
		8:	REG_DATA = {16'h36fa, 8'h3b}; 
		9:	REG_DATA = {16'h36fb, 8'ha6}; 
		10:	REG_DATA = {16'h36fc, 8'h10}; 
		11:	REG_DATA = {16'h36fd, 8'h24}; 
		12:	REG_DATA = {16'h36e9, 8'h04}; 
		13:	REG_DATA = {16'h36f9, 8'h04}; 
		14:	REG_DATA = {16'h301f, 8'h50}; 
		15:	REG_DATA = {16'h302d, 8'h20}; 
		16:	REG_DATA = {16'h3106, 8'h01}; 
		17:	REG_DATA = {16'h3200, 8'h01}; 
		18:	REG_DATA = {16'h3201, 8'h54}; 
		19:	REG_DATA = {16'h3202, 8'h01}; 
		20:	REG_DATA = {16'h3203, 8'hb4}; 
		21:	REG_DATA = {16'h3204, 8'h08}; 
		22:	REG_DATA = {16'h3205, 8'hdb}; 
		23:	REG_DATA = {16'h3206, 8'h05}; 
		24:	REG_DATA = {16'h3207, 8'hf3}; 
		25:	REG_DATA = {16'h3208, 8'h07}; 
		26:	REG_DATA = {16'h3209, 8'h80}; 
		27:	REG_DATA = {16'h320a, 8'h04}; 
		28:	REG_DATA = {16'h320b, 8'h38}; 
		29:	REG_DATA = {16'h320c, 8'h0b}; 
		30:	REG_DATA = {16'h320d, 8'h24}; 
		31:	REG_DATA = {16'h320e, 8'h04}; 
		32:	REG_DATA = {16'h320f, 8'h67}; 
		33:	REG_DATA = {16'h3210, 8'h00}; 
		34:	REG_DATA = {16'h3211, 8'h04}; 
		35:	REG_DATA = {16'h3212, 8'h00}; 
		36:	REG_DATA = {16'h3213, 8'h04}; 
		37:	REG_DATA = {16'h3249, 8'h0f}; 
		38:	REG_DATA = {16'h3253, 8'h06}; 
		39:	REG_DATA = {16'h3271, 8'h13}; 
		40:	REG_DATA = {16'h3273, 8'h13}; 
		41:	REG_DATA = {16'h3301, 8'h0a}; 
		42:	REG_DATA = {16'h3306, 8'h40}; 
		43:	REG_DATA = {16'h3309, 8'h60}; 
		44:	REG_DATA = {16'h330a, 8'h00}; 
		45:	REG_DATA = {16'h330b, 8'hd8}; 
		46:	REG_DATA = {16'h331e, 8'h41}; 
		47:	REG_DATA = {16'h331f, 8'h51}; 
		48:	REG_DATA = {16'h3320, 8'h04}; 
		49:	REG_DATA = {16'h3333, 8'h10}; 
		50:	REG_DATA = {16'h335d, 8'h60}; 
		51:	REG_DATA = {16'h3364, 8'h56}; 
		52:	REG_DATA = {16'h336b, 8'h08}; 
		53:	REG_DATA = {16'h3390, 8'h08}; 
		54:	REG_DATA = {16'h3391, 8'h18}; 
		55:	REG_DATA = {16'h3392, 8'h38}; 
		56:	REG_DATA = {16'h3393, 8'h0a}; 
		57:	REG_DATA = {16'h3394, 8'h24}; 
		58:	REG_DATA = {16'h3395, 8'h30}; 
		59:	REG_DATA = {16'h33ad, 8'h29}; 
		60:	REG_DATA = {16'h341c, 8'h04}; 
		61:	REG_DATA = {16'h341d, 8'h04}; 
		62:	REG_DATA = {16'h341e, 8'h03}; 
		63:	REG_DATA = {16'h3425, 8'h00}; 
		64:	REG_DATA = {16'h3426, 8'h00}; 
		65:	REG_DATA = {16'h3622, 8'hc7}; 
		66:	REG_DATA = {16'h3632, 8'h44}; 
		67:	REG_DATA = {16'h3636, 8'h6e}; 
		68:	REG_DATA = {16'h3637, 8'h08}; 
		69:	REG_DATA = {16'h3638, 8'h0a}; 
		70:	REG_DATA = {16'h3651, 8'h9d}; 
		71:	REG_DATA = {16'h3670, 8'h4b}; 
		72:	REG_DATA = {16'h3674, 8'hc0}; 
		73:	REG_DATA = {16'h3675, 8'h58}; 
		74:	REG_DATA = {16'h3676, 8'h5a}; 
		75:	REG_DATA = {16'h367c, 8'h18}; 
		76:	REG_DATA = {16'h367d, 8'h38}; 
		77:	REG_DATA = {16'h3690, 8'h43}; 
		78:	REG_DATA = {16'h3691, 8'h53}; 
		79:	REG_DATA = {16'h3692, 8'h53}; 
		80:	REG_DATA = {16'h3699, 8'h08}; 
		81:	REG_DATA = {16'h369a, 8'h10}; 
		82:	REG_DATA = {16'h369b, 8'h1f}; 
		83:	REG_DATA = {16'h369c, 8'h18}; 
		84:	REG_DATA = {16'h369d, 8'h38}; 
		85:	REG_DATA = {16'h36a2, 8'h08}; 
		86:	REG_DATA = {16'h36a3, 8'h18}; 
		87:	REG_DATA = {16'h36a6, 8'h08}; 
		88:	REG_DATA = {16'h36a7, 8'h18}; 
		89:	REG_DATA = {16'h36ab, 8'h40}; 
		90:	REG_DATA = {16'h36ac, 8'h40}; 
		91:	REG_DATA = {16'h36ad, 8'h40}; 
		92:	REG_DATA = {16'h3901, 8'h00}; 
		93:	REG_DATA = {16'h3904, 8'h0c}; 
		94:	REG_DATA = {16'h3906, 8'h3a}; 
		95:	REG_DATA = {16'h391d, 8'h14}; 
		96:	REG_DATA = {16'h3e01, 8'h8c}; 
		97:	REG_DATA = {16'h3e02, 8'h60}; 
		98:	REG_DATA = {16'h4000, 8'h00}; 
		99:	REG_DATA = {16'h4001, 8'h04}; 
		100:	REG_DATA = {16'h4002, 8'hb0}; 
		101:	REG_DATA = {16'h4003, 8'h00}; 
		102:	REG_DATA = {16'h4004, 8'h00}; 
		103:	REG_DATA = {16'h4005, 8'h00}; 
		104:	REG_DATA = {16'h4006, 8'h07}; 
		105:	REG_DATA = {16'h4007, 8'hae}; 
		106:	REG_DATA = {16'h4008, 8'h00}; 
		107:	REG_DATA = {16'h4009, 8'hc8}; 
		108:	REG_DATA = {16'h440e, 8'h02}; 
		109:	REG_DATA = {16'h4509, 8'h28}; 
		110:	REG_DATA = {16'h4837, 8'h21}; 
		111:	REG_DATA = {16'h5000, 8'h0e}; 
		112:	REG_DATA = {16'h302d, 8'h00}; 
		113:	REG_DATA = {16'h3e08, 8'h07}; 
		114:	REG_DATA = {16'h3e09, 8'h3f}; 
		115:	REG_DATA = {16'h0100, 8'h01}; 

		default:REG_DATA    =   {16'h0000, 8'h00};
    endcase
end

endmodule

