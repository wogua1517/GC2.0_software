#include "System_Config.h"

#if USE_MOTOR
#include "Motor.h"

u8 USART6_RX_BUF[USART6_MAX_RECV_LEN]; 				//接收缓冲,最大USART6_MAX_RECV_LEN个字节.
u8 USART6_TX_BUF[USART6_MAX_SEND_LEN]; 			//发送缓冲,最大USART6_MAX_SEND_LEN字节
u16 USART6_RX_STA=0;   

// 自定义的printf函数
// 重定义printf函数
void u2_printf(char* fmt,...)
{
    u16 i,j; 
	va_list ap; 
    //开始执行数据的插入
	va_start(ap,fmt);
	vsprintf((char*)USART6_TX_BUF,fmt,ap);   //进行数据的插入
	va_end(ap);
    //停止执行数据的插入
	i=strlen((const char*)USART6_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
        while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(USART6,USART6_TX_BUF[j]); 
	} 
}

void communicate_uart_init(u32 bound){
	
	//GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOD时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
 
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOC6复用为USART6
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOC7复用为USART6
	
	//USART6端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6与GPIOC7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC6，PC7

   //USART6 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART6, &USART_InitStructure); //初始化串口6
	
    USART_Cmd(USART6, ENABLE);  //使能串口6
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

	//USART6 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口6中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

}


/********************************************************
char* itoa(int num,char* str,int radix)
将输入的数字转化为字符串 num 数字 str字符串 radix数据长度
示例如itoa(vx,string1,10);
将vx转化为字符串并存入string1中

********************************************************/

char* itoa(int num,char* str,int radix)
{
	  char temp;
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    unsigned unum;
    int i=0,j,k;
 
    
    if(radix==10&&num<0)
    {
        unum=(unsigned)-num;
        str[i++]='-';
    }
    else unum=(unsigned)num;
 
    
    do
    {
        str[i++]=index[unum%(unsigned)radix];
        unum/=radix;
 
    }while(unum);
 
    str[i]='\0';
 
    if(str[0]=='-') k=1;
    else k=0;
 

    for(j=k;j<=(i-1)/2;j++)
    {
        temp=str[j];
        str[j]=str[i-1+k-j];
        str[i-1+k-j]=temp;
    }
 
    return str;
 
}



/********************************************************
void send(int vx,int vy,int vz,int z,int motor1,int motor2,int holder)
发送数据到下位机
vx 竖的速度
vy 横的速度
vz 旋转的速度

以下数据都还没用
z  车子所在的角度 
motor1 小电机1状态
motor2 小电机2状态
holder 云台状态
*******************************************************/

char string1[16] = {0};
char string2[16] = {0};
char string3[16] = {0};
char string4[16] = {0};
char string5[16] = {0};
char string6[16] = {0};
char string7[16] = {0};
int vx_last=0,vy_last=0,vz_last=0;
u8 angle_last=0;
void send(int vx,int vy,int vz,int z,int motor1,int motor2,int holder)
{
	itoa(vy,string1,10);
	itoa(vx,string2,10);
	itoa(vz,string3,10);
	itoa(z,string4,10);
	itoa(motor1,string5,10);
	itoa(motor2,string6,10);
	itoa(holder,string7,10);
	u2_printf("{%s %s %s %s %s %s %s}",string1,string2,string3,string4,string5,string6,string7);
}
#endif
