/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2019/12/17
*Module Name:run_led
*File Name:run_led.v
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
*3) _n activ low
*4) _dg debug signal 
*5) _r delay or register
*6) S_ state mechine
*********************************************************************/

`timescale 1ns / 1ps

module pll_test
(
input 		I_sysclk,//系统时钟输入
output [1:0]O_up_led//LED输出
);
    
reg [25:0]  cnt;

//例化PLL模块
pll pll_inst(
.refclk     (I_sysclk),//PLL输入时钟
.reset 		( 1'b0),//Pll复位
.lock    	(pll_lock),//PLL锁住
.clk0_out 	(clk0)//PLL输出时钟
);

assign O_up_led = {cnt[25:24]};//输出计数器的高位用于驱动LED

//计数器
always @ (posedge clk0 or negedge pll_lock)begin
	if(pll_lock==1'b0)
	  cnt <= 26'd0;
	else
   	  cnt <= cnt + 1'b1; //cnt power on initial value is all 1
end

endmodule

