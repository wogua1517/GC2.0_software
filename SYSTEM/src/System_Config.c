#include "System_Config.h"

__GOOD goods[1+SOURCENUM];
__POINT points[1+SOURCENUM+5];
u8 data_commit_flag = USE_UPPC;// 选择是否向上位机上传数据，当上传数据的时候置数为1，为0表示使用无线串口
float final_x ;
float final_y ;

void System_Resource_Init(void)
{
	float zangle_last = 0xff;   							//保存上一次的z轴数据 
	code = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 		// 中断优先级分组
	delay_init(180);
			
	Location_TIM3_init(100 - 1, 9000 - 1); 					// PID的定时器初始化
	Location_TIM7_init(200 - 1, 9000 - 1);
	
//	uart_init(115200);  
	Raspi_uart_Init(115200);								// 串口2初始化	树莓派
	ServoCon_uart_Init(9600);                 				// 串口3初始化	舵控板
	Slide_uart_Init(115200);								// 串口5初始化	滑台
	communicate_uart_init(115200); 							// 串口6初始化 	下位机
	OPS_DMA_Init(115200);                 					// 串口7初始化	ops通信 		  
	qrcode_uart_Init(9600);                   				// 串口8初始化 	二维码识别 
	
	LED_Init();    											// LED初始化
	KEY_Init();    											// 按键初始化，发车
	BEEP_Init();   											// 蜂鸣器初始化
	
	//Flash_ON();
	BUZZER_ON;                   							// 蜂鸣器鸣叫
	delay_ms(200);             								// 控制蜂鸣器鸣叫的时间
	BUZZER_OFF;                  							// 关闭蜂鸣器
	
	send(0, 0, 0, 0, 0, 0, 0);								// 发送小车停止的指令
	
	OPS_Init();
	zangle_last = zangle;  									// 读取opsZ轴数据
	delay_ms(1000);
	
	if( fabs(zangle-zangle_last) < 0.1f)
	{
			BUZZER_ON;                   					// 蜂鸣器鸣叫
			delay_ms(200);             						// 控制蜂鸣器鸣叫的时间
			BUZZER_OFF;         
	}
	else 
	{
		while(1)
		{
			BUZZER_ON;                  					// 蜂鸣器鸣叫
			delay_ms(200);             						// 控制蜂鸣器鸣叫的时间
			BUZZER_OFF;                  					// 关闭蜂鸣器
			delay_ms(1000); 
				
		}
	}
	
	
	AllPoint_Init(points,goods);							// 点位初始化
	task_init();											// 任务初始化
	
	
	delay_ms(2000);
//	Emm_V5_En_Control( 1, true, false);						// 滑台使能
//	Emm_V5_Stop_Now(1,false);
//	Emm_V5_Reset_CurPos_To_Zero(1);
//	Emm_V5_Pos_Control(1, 0, 1000, 0, 0, true, false);		// 滑台复位
	Emm_V5_Origin_Trigger_Return(1, 2, false);
//	Emm_V5_Pos_Control(1, 0, 1000, 0, 1000, true, false);
	while(rxFrameFlag == false); rxFrameFlag = false;
	
	delay_ms(200);             							
	
	
	moveServo(2,POSITION_UP,1000);							// 抬起圆盘
	moveServo(1,POSITION_0,R_Speed);						// 转盘复位
	
	//延时
	getPosition(1);
	receiveHandle();
	while(abs(now_position-POSITION_0)>=5) {
		getPosition(1);	
		receiveHandle();	
	}
	getPosition(2);
	receiveHandle();
	while(abs(now_position-POSITION_UP)>=5) {
		//moveServo(2,POSITION_UP,50);							// 抬起圆盘
		getPosition(2);	
		receiveHandle();	
	}
	
	// 树莓派程序初始化
	USART_SendData(USART2, 0);								
	while ((USART2->SR & 0x40) == 0);						
	USART_SendData(USART2, 3);								
	while ((USART2->SR & 0x40) == 0);	
	final_x = 0;
	final_y = 0;
	u8 cnt = 0;
	float temp_x,temp_y;
	while(cnt<3) {
		u8 xbuf[4];
		xbuf[0] = rxd_buf[0];
		xbuf[1] = rxd_buf[1];
		xbuf[2] = rxd_buf[2];
		xbuf[3] = rxd_buf[3];
				
		final_x = *((float*)&xbuf[0]);
		
		xbuf[0] = rxd_buf[4];
		xbuf[1] = rxd_buf[5];
		xbuf[2] = rxd_buf[6];
		xbuf[3] = rxd_buf[7];
				
		final_y = *((float*)&xbuf[0]);	
		if(temp_x!=final_x||temp_y!=final_y||temp_x==0) {
			temp_x=final_x;
			temp_y=final_y;
			cnt=0;
		}
		else cnt++;

		delay_ms(50);
	}
	USART_SendData(USART2, 0);								
	while ((USART2->SR & 0x40) == 0);	
	
	final_x = 155;
	final_y = 92;
	
	BUZZER_ON;                   							// 初始化完成提示
	delay_ms(200);             								
	BUZZER_OFF;  
}

void System_Tasks_Init(void)
{
	u8 key;
	while (1)      
	{
		key = KEY_Scan(0);
		if (key == KEY0_PRES)
		{
			Run_Task();
		}
	}
}
