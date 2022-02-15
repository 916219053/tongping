//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com 
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           hdmi_top
// Last modified Date:  2019/7/1 9:30:00
// Last Version:        V1.1
// Descriptions:        HDMI顶层模块
//----------------------------------------------------------------------------------------
// Created by:          正点原子
// Created date:        2019/7/1 9:30:00
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module hdmi_top(
	input 		   hdmi_clk     ,
	input 		   hdmi_clk_5   ,
	input 		   rst_n        ,
	
	input  [15:0]  rd_data      ,
	output 		   rd_en        ,
	
	output [10:0 ] pixel_xpos   ,  //像素点横坐标
	output [10:0 ] pixel_ypos   ,  //像素点纵坐标 
	output [10:0]  h_disp       ,
	output [10:0]  v_disp       ,
	output         video_vs     ,
	
	//HDMI接口
   output         tmds_clk_p   ,  // TMDS 时钟通道
   output         tmds_clk_n   ,
   output [2:0]   tmds_data_p  ,  // TMDS 数据通道
   output [2:0]   tmds_data_n

);

//wire define

wire [23:0 ] video_rgb     ;
wire [23:0 ] rd_data888    ;
wire         video_hs      ;
wire         video_de      ;

assign rd_data888 = {rd_data[15:11],3'b000,rd_data[10:5],2'b00,
                    rd_data[4:0],3'b000};  

//例化视频显示驱动模块
video_driver u_video_driver(
    .pixel_clk      (hdmi_clk    ),
    .sys_rst_n      (rst_n       ),

    .video_hs       (video_hs    ),
    .video_vs       (video_vs    ),
    .video_de       (video_de    ),
    .video_rgb      (   ),
    .data_req       (rd_en       ),
    .pixel_xpos     (pixel_xpos  ),
    .pixel_ypos     (pixel_ypos  ),
	 .h_disp         (h_disp      ),
	 .v_disp         (v_disp      ),
    .video_rgb_565  (rd_data     )
    );

lcd_disply u_lcd_disply(

	.lcd_clk    (hdmi_clk),
	.sys_rst_n  (rst_n),
	
	.pixel_xpos (pixel_xpos),
	.pixel_ypos (pixel_ypos),
	.ID_lcd     (),
	.rd_data    (rd_data888),
	.rd_h_pixel (h_disp),
	.pixel_data (video_rgb)
);

//例化HDMI驱动模块
dvi_transmitter_top u_rgb2dvi_0(
    .pclk           (hdmi_clk   ),
    .pclk_x5        (hdmi_clk_5 ),
    .reset_n        (rst_n      ),
                
    .video_din      (video_rgb  ),
    .video_hsync    (video_hs   ), 
    .video_vsync    (video_vs   ),
    .video_de       (video_de   ),
                
    .tmds_clk_p     (tmds_clk_p ),
    .tmds_clk_n     (tmds_clk_n ),
    .tmds_data_p    (tmds_data_p),
    .tmds_data_n    (tmds_data_n)
    );
	 
endmodule