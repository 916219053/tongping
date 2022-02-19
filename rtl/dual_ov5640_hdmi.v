module dual_ov5640_hdmi(    
    input         sys_clk    ,  //系统时钟
    input         sys_rst_n  ,  //系统复位，低电平有效
    
    input         key0,
    input         key1,
    //摄像头0  
    input         cam0_pclk  ,  //cmos 数据像素时钟
    input         cam0_vsync ,  //cmos 场同步信号
    input         cam0_href  ,  //cmos 行同步信号
    input  [7:0]  cam0_data  ,  //cmos 数据   
    output        cam0_rst_n ,  //cmos 复位信号，低电平有效
    output        cam0_pwdn  ,  //cmos 电源休眠模式选择信号 
    output        cam0_scl   ,  //cmos SCCB_SCL线
    inout         cam0_sda   ,  //cmos SCCB_SDA线
    
    //摄像头1 
    input         cam1_pclk  ,  //cmos 数据像素时钟
    input         cam1_vsync ,  //cmos 场同步信号
    input         cam1_href  ,  //cmos 行同步信号
    input  [7:0]  cam1_data  ,  //cmos 数据  
    output        cam1_rst_n ,  //cmos 复位信号，低电平有效
    output        cam1_pwdn  ,  //cmos 电源休眠模式选择信号
    output        cam1_scl   ,  //cmos SCCB_SCL线
    inout         cam1_sda   ,  //cmos SCCB_SDA线    
    //SDRAM 
    output        sdram_clk  ,  //SDRAM 时钟
    output        sdram_cke  ,  //SDRAM 时钟有效
    output        sdram_cs_n ,  //SDRAM 片选
    output        sdram_ras_n,  //SDRAM 行有效
    output        sdram_cas_n,  //SDRAM 列有效
    output        sdram_we_n ,  //SDRAM 写有效
    output [1:0]  sdram_ba   ,  //SDRAM Bank地址
    output [1:0]  sdram_dqm  ,  //SDRAM 数据掩码
    output [12:0] sdram_addr ,  //SDRAM 地址
    inout  [15:0] sdram_data ,  //SDRAM 数据      
    //HDMI接口
    output        tmds_clk_p ,  // TMDS 时钟通道
    output        tmds_clk_n ,
    output [2:0]  tmds_data_p,  // TMDS 数据通道
    output [2:0]  tmds_data_n,
    
    input   wire            sd_miso     ,
    output  wire            sd_clk      ,   //SD卡时钟信号
    output  wire            sd_cs_n     ,   //片选信号
    output  wire            sd_mosi        //主输出从输入信号
    );
	 
parameter V_CMOS_DISP   = 13'd800       ; //CMOS分辨率--行
parameter H_CMOS_DISP   = 13'd1280      ; //CMOS分辨率--列	
parameter TOTAL_H_PIXEL = 13'd2570      ; //CMOS分辨率--行
parameter TOTAL_V_PIXEL = 13'd980       ; 
parameter   DATA_NUM    =   12'd256     ;   //读写数据个数
//wire define
wire        clk_100m       ;  //100mhz时钟,SDRAM操作时钟
wire        clk_100m_shift ;  //100mhz时钟,SDRAM相位偏移时钟
wire        clk_50m        ;  //100mhz时钟,LCD顶层模块时钟 
wire        clk_50m_shift  ;
wire        locked         ;
wire        rst_n          ;
wire        sys_init_done  ;  //系统初始化完成(sdram初始化+摄像头初始化)
                    
wire        wr0_en         ;  //sdram_ctrl模块写使能
wire [15:0] wr0_data       ;  //sdram_ctrl模块写数据
wire        wr1_en         ;  //sdram_ctrl模块写使能
wire [15:0] wr1_data       ;  //sdram_ctrl模块写数据
wire        rd_en          ;  //sdram_ctrl模块读使能
wire [15:0] rd_data        ;  //sdram_ctrl模块读数据
wire [12:0] rd_h_pixel     ;  //图像水平像素
wire        sdram_init_done;  //SDRAM初始化完成
wire        cam_init_done0 ;
wire        cam_init_done1 ;

wire [12:0] cmos_h_pixel   ;  //CMOS水平方向像素个数  
wire [12:0] cmos_v_pixel   ;  //CMOS垂直方向像素个数
wire [12:0] total_h_pixel  ;  //水平总像素大小
wire [12:0] total_v_pixel  ;  //垂直总像素大小
wire [23:0] sdram_max_addr ;  //sdram读写的最大地址
wire        hdmi_clk       ;  
wire        hdmi_clk_5     ;  
wire        locked_hdmi    ;
wire        video_vs       ;
wire        cmos0_flag;
wire        cmos1_flag;
reg        cmos0_en;
reg        cmos1_en;
wire        cmos0_open;
wire        cmos1_open;
wire        init_end;
wire        wrsd_en;
wire        wr_addr;
wire        wr_req;
wire        wr_sd_data;
wire    [11:0]  wr_fifo_data_num ;
reg         wr_busy_dly         ;   //sd卡写数据忙信号打一拍

//*****************************************************
//**                    main code
//*****************************************************

assign  rst_n = sys_rst_n & locked & locked_hdmi;
assign  sys_init_done=cam_init_done0 & cam_init_done1 & sdram_init_done;
assign  cmos0_open= cmos0_en? wr0_en:1'b0;
assign  cmos1_open= cmos1_en? wr1_en:1'b0;
assign  wrsd_en = ((wr_fifo_data_num == (DATA_NUM)) && (init_end == 1'b1))
                ? 1'b1 : 1'b0;    


always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n==1'b0) 
        cmos0_en<=1'b1;
    else if(cmos0_flag==1'b1)
        cmos0_en<=~cmos0_en;
    else
        cmos0_en<=cmos0_en;
        
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n==1'b0) 
        cmos1_en<=1'b1;
    else if(cmos1_flag==1'b1)
        cmos1_en<=~cmos1_en;
    else
        cmos1_en<=cmos1_en;
        
pll u_pll(
    .areset             (~sys_rst_n    ),
    .inclk0             (sys_clk       ),
            
    .c0                 (clk_100m      ),
    .c1                 (clk_100m_shift),
    .c2                 (clk_50m       ),
    .c3                 (clk_50m_shift ),
    .locked             (locked        )
    );
	 
pll_hdmi	pll_hdmi_inst (
	.areset              ( ~sys_rst_n  ),
	.inclk0              ( sys_clk     ),
	.c0                  ( hdmi_clk    ),
	.c1                  ( hdmi_clk_5  ),
	.locked              ( locked_hdmi )
	);	 
//按键
key_filter
#(
    .CNT_MAX(20'd999_999) //计数器计数最大值
)key_filter_cmos0_inst
(
    .sys_clk    (sys_clk) ,   //系统时钟50Mhz
    .sys_rst_n  (sys_rst_n) ,   //全局复位
    .key_in     (key0) ,   //按键输入信号

    .key_flag   (cmos0_flag)     //key_flag为1时表示消抖后检测到按键被按下
                                    //key_flag为0时表示没有检测到按键被按下
);

key_filter
#(
    .CNT_MAX(20'd999_999) //计数器计数最大值
)key_filter_cmos1_inst
(
    .sys_clk    (sys_clk) ,   //系统时钟50Mhz
    .sys_rst_n  (sys_rst_n) ,   //全局复位
    .key_in     (key1) ,   //按键输入信号

    .key_flag   (cmos1_flag)     //key_flag为1时表示消抖后检测到按键被按下
                                    //key_flag为0时表示没有检测到按键被按下
);


//OV5640 0摄像头驱动
ov5640_dri u0_ov5640_dri(
    .clk               (clk_50m   ),
    .rst_n             (rst_n     ),

    .cam_pclk          (cam0_pclk ),
    .cam_vsync         (cam0_vsync),
    .cam_href          (cam0_href ),
    .cam_data          (cam0_data ),
    .cam_rst_n         (cam0_rst_n),
    .cam_pwdn          (cam0_pwdn ),
    .cam_scl           (cam0_scl  ),
    .cam_sda           (cam0_sda  ),
    
    .capture_start     (sdram_init_done),
    .cmos_h_pixel      (H_CMOS_DISP[12:1]),
    .cmos_v_pixel      (V_CMOS_DISP),
    .total_h_pixel     (TOTAL_H_PIXEL),
    .total_v_pixel     (TOTAL_V_PIXEL),
	 .cam_init_done     (),   

    .cmos_frame_vsync  (),
    .cmos_frame_href   (),
    .cmos_frame_valid  (wr0_en),
    .cmos_frame_data   (wr0_data)
    );
    
//OV5640 1摄像头驱动
ov5640_dri u1_ov5640_dri(
    .clk               (clk_50m   ),
    .rst_n             (rst_n     ), 

    .cam_pclk          (cam1_pclk ),
    .cam_vsync         (cam1_vsync),
    .cam_href          (cam1_href ),
    .cam_data          (cam1_data ),
    .cam_rst_n         (cam1_rst_n),
    .cam_pwdn          (cam1_pwdn  ),
    .cam_scl           (cam1_scl  ),
    .cam_sda           (cam1_sda  ),
    
    .capture_start     (sdram_init_done),
    .cmos_h_pixel      (H_CMOS_DISP[12:1]),
    .cmos_v_pixel      (V_CMOS_DISP),
    .total_h_pixel     (TOTAL_H_PIXEL),
    .total_v_pixel     (TOTAL_V_PIXEL),
	 .cam_init_done     (),   

    .cmos_frame_vsync  (),
    .cmos_frame_href   (),
    .cmos_frame_valid  (wr1_en),
    .cmos_frame_data   (wr1_data)
    );    
    
//SDRAM 控制器顶层模块,封装成FIFO接口
//SDRAM 控制器地址组成: {bank_addr[1:0],row_addr[12:0],col_addr[8:0]}
sdram_top u_sdram_top(
    .ref_clk            (clk_100m),         //sdram 控制器参考时钟
    .out_clk            (clk_100m_shift),   //用于输出的相位偏移时钟
    .rst_n              (rst_n),            //系统复位
                                       
    //用户写端口    
    .wr_len             (10'd512),          //写SDRAM时的数据突发长度
    .wr_load            (~rst_n),           //写端口复位: 复位写地址,清空写FIFO    
    .wr_min_addr        (24'd0),            //写SDRAM的起始地址
    .wr_max_addr        (V_CMOS_DISP*H_CMOS_DISP-1),   //写SDRAM的结束地址
    
    .wr_clk0            (cam0_pclk),        //写端口FIFO: 写时钟
    .wr_en0             (cmos0_open),           //写端口FIFO: 写使能
    .wr_data0           (wr0_data),         //写端口FIFO: 写数据

    .wr_clk1            (cam1_pclk),        //写端口FIFO: 写时钟
    .wr_en1             (cmos1_open),           //写端口FIFO: 写使能
    .wr_data1           (wr1_data),         //写端口FIFO: 写数据
    
    //用户读端口
    .rd_h_pixel         (H_CMOS_DISP[12:1]),
    .rd_clk             (hdmi_clk),         //读端口FIFO: 读时钟
    .rd_en              (rd_en),            //读端口FIFO: 读使能
    .rd_data            (rd_data),          //读端口FIFO: 读数据
    .rd_min_addr        (24'd0),            //读SDRAM的起始地址
    .rd_max_addr        (V_CMOS_DISP*H_CMOS_DISP-1),   //读SDRAM的结束地址
    .rd_len             (10'd512),          //从SDRAM中读数据时的突发长度
    .rd_load            (~rst_n),           //读端口复位: 复位读地址,清空读FIFO
    
    //用户控制端口                                
    .sdram_read_valid   (1'b1),             //SDRAM 读使能
    .sdram_pingpang_en  (1'b1),             //SDRAM 乒乓操作使能
    .sdram_init_done    (sdram_init_done),  //SDRAM 初始化完成标志
                                            
    //SDRAM 芯片接口                                
    .sdram_clk          (sdram_clk),        //SDRAM 芯片时钟
    .sdram_cke          (sdram_cke),        //SDRAM 时钟有效
    .sdram_cs_n         (sdram_cs_n),       //SDRAM 片选
    .sdram_ras_n        (sdram_ras_n),      //SDRAM 行有效
    .sdram_cas_n        (sdram_cas_n),      //SDRAM 列有效
    .sdram_we_n         (sdram_we_n),       //SDRAM 写有效
    .sdram_ba           (sdram_ba),         //SDRAM Bank地址
    .sdram_addr         (sdram_addr),       //SDRAM 行/列地址
    .sdram_data         (sdram_data),       //SDRAM 数据
    .sdram_dqm          (sdram_dqm),         //SDRAM 数据掩码
    
    .wr_addr          (wr_addr)
    );
	 
//例化HDMI顶层模块
hdmi_top u_hdmi_top(
    .hdmi_clk       (hdmi_clk   ),
    .hdmi_clk_5     (hdmi_clk_5 ),
    .rst_n          (rst_n      ),
                
    .rd_data        (rd_data    ),
    .rd_en          (rd_en      ), 
    .h_disp         (),	 
    .v_disp         (),
    .pixel_xpos     (),
    .pixel_ypos     (),
    .video_vs       (video_vs),	 
    .tmds_clk_p     (tmds_clk_p ),
    .tmds_clk_n     (tmds_clk_n ),
    .tmds_data_p    (tmds_data_p),
    .tmds_data_n    (tmds_data_n)
    );	 

sd_ctrl sd_ctrl_inst
(
    .sys_clk         (clk_50m       ),  //输入工作时钟,频率50MHz
    .sys_clk_shift   (clk_50m_shift ),  //输入工作时钟,频率50MHz,相位偏移180度
    .sys_rst_n       (rst_n         ),  //输入复位信号,低电平有效

    .sd_miso         (sd_miso       ),  //主输入从输出信号
    .sd_clk          (sd_clk        ),  //SD卡时钟信号
    .sd_cs_n         (sd_cs_n       ),  //片选信号
    .sd_mosi         (sd_mosi       ),  //主输出从输入信号

    .wr_en           (wrsd_en         ),  //数据写使能信号
    .wr_addr         (wr_addr       ),  //写数据扇区地址
    .wr_data         (wr_sd_data       ),  //写数据
    .wr_busy         (     ),  //写操作忙信号
    .wr_req          (wr_req        ),  //写数据请求信号

    .rd_en           (         ),  //数据读使能信号
    .rd_addr         (       ),  //读数据扇区地址
    .rd_busy         (       ),  //读操作忙信号
    .rd_data_en      (    ),  //读数据标志信号
    .rd_data         (       ),  //读数据


    .init_end        (init_end      )   //SD卡初始化完成信号
);

sd_wr_fifo	sd_wr_fifo_inst (
	.data ( rd_data ),
	.wrclk ( clk_50m ),
	.wrreq ( rd_en ),
    
    .rdclk ( clk_50m ),
	.rdreq ( wr_req ),
	.q ( wr_sd_data ),
	.rdusedw ( wr_fifo_data_num )
	);


endmodule 