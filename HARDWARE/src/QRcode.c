/*******************************************************************************
* 文件名: qrcode.c
* 作  者: 宋承熠
* 日  期: 2021/8/25
* 二维码数据获取
*******************************************************************************/

#include "System_Config.h"

#if USE_QRCODE
#include "QRcode.h"

u8 code_n=0;
u8 code_mid[2];
u8 code;
/*******************************************************************************
* 函 数 名         : qrcode_uart_Init
* 函数功能		   : UART8初始化函数
* 输    入         : bound:波特率
* 输    出         : 无
*******************************************************************************/ 
void qrcode_uart_Init(u32 bound)     //串口初始化为9600
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);
 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_UART8);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_UART8);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	

   //UART8 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART8, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(UART8, ENABLE);  //使能串口8 
	
	USART_ClearFlag(UART8, USART_FLAG_TC);
		
	USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);//开启相关中断

	//UART8 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;//串口8中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、	
}

/*******************************************************************************
* 函 数 名         : UART8_IRQHandler
* 函数功能		   : USART8中断函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/ 
void UART8_IRQHandler(void)                	//串口8中断服务程序
{
	u8 r,i = 0;
	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(UART8, USART_IT_RXNE);
		r =USART_ReceiveData(UART8);//(UART8->DR);	//读取接收到的数据
		if(r!=13)
		{
		  code_mid[code_n++] = r;
        //用数组去接收扫二维码（二维码是三个数）
        //下面一定要写成4，因为它发过来的数据是3+1个
		}
		else
		{
			code=0;
			for(i=0;i<code_n;i++)
			{
				code_mid[i]=code_mid[i]-48;
				code=code*10+code_mid[i];
			}
			code_mid[0]=code_mid[1]=0;
			code_n=0;
		}
		USART_SendData(UART8,r);
		while(USART_GetFlagStatus(UART8,USART_FLAG_TC) != SET);
	} 
	USART_ClearFlag(UART8,USART_FLAG_TC);
} 	


#endif




