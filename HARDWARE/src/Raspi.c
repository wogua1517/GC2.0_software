#include "System_Config.h"

#if USE_RASPI
#include "Raspi.h"


//串口发送缓存区 	
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节	  
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.


/*******************************************************************************
* 函 数 名         : Raspi_uart_Init
* 函数功能		   : UART2初始化函数
* 输    入         : bound:波特率
* 输    出         : 无
*******************************************************************************/ 
void Raspi_uart_Init(u32 bound)     //串口初始化为9600
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	

   //UART8 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	USART_Cmd(USART2, ENABLE);  //使能串口2 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); //开启串口空闲中断，每收到一帧数据进入一次中断

	//UART2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
	
}

void Raspi_printf(char* fmt,...)  
{  
	u16 i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART2_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART2_TX_BUF);//此次发送数据的长度
	for(j=0;j<i;j++)//循环发送数据
	{
	  while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  //等待上次传输完成 
		USART_SendData(USART2,(uint8_t)USART2_TX_BUF[j]); 	 //发送数据到串口3 
	}
	
}

u8 rxd_buf[8];//定义数组接收数据包，定长
u8 rxd_flag = 0;//接收标志
u8 rxd_index = 0;//接收索引

void USART2_IRQHandler(void)	
{
	u8 recv_dat;
	static uint8_t recv_state = 0;//默认从索引0开始
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		recv_dat = USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
		switch(recv_state)
		{ 
		case 0:
				if(recv_dat == 0xFE)//接收到包头
				{
						recv_state = 1;//切换状态
						rxd_index = 0;
				}
				else
				{
						recv_state = 0;//切换状态
				}
				break;
		case 1:
				rxd_buf[rxd_index++] = recv_dat;//接收字符
				if(rxd_index>=10)//判断是否接收数据包完成1
				{
						recv_state =2;//切换状态
				}
				break;
		case 2:
				if(recv_dat == 0xAA)//接收到包尾
				{
						rxd_flag = 1;//标志位置1
						recv_state =0;//并将状态清零，接收下一包数据 
					
						
				}
				break;
		}
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志位	 
	
	} 

}
#endif
