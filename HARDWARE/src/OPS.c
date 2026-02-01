#include "System_Config.h"

#if USE_OPS
#include "OPS.h"

u8 OPS_DMA_ready = 0;

u8 DMA_OPS_buf[48] = {'*',0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, '#'};
Union_OPS OPS;
	
	
void OPS_Init(void)
{
	#if USE_ABSOLUTE_POS
	Reference_Position.x = 0;
	Reference_Position.y = 0;
	Reference_Position.z = 0;
	#endif
	while(OPS_DMA_ready == 0)
    {
       DMA_OPS_buf[41] = 1;   // 上位机的数据未发生改变
    }                    // 等待ops已经可以正常工作了
    DMA_OPS_buf[41] = 0; // ops的标志位,发送给上位机
    Cali_Ops();    // 进行清零的操作
	#if USE_ABSOLUTE_POS
	Reference_Position.x = 0;
	Reference_Position.y = 0;
	Reference_Position.z = 0;
	#endif
    delay_ms(10);
}
void OPS_DMA_Init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_UART7);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_UART7);

    //初始化用于锁定等待OPS的按键   板载PB2（按键） 按下为高电平
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;   //普通输出模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART7, &USART_InitStructure);

    USART_Cmd(UART7, ENABLE);

    USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    OPS_DMA_init(DMA1_Stream3, DMA_Channel_5, (u32)&UART7->DR, (u32)&OPS.data[2], 28); // DMA初始化 自动发送数据至LCD  从数组[2]开始 为避免结构体成员裂缝 保证数据正常
    OPS_Inspection_TIM_init(1000, 8400);                                               //每100ms进行一次OPS数据自检
}

void OPS_DMA_init(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{ 
	DMA_InitTypeDef  DMA_InitStructure;
	
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 	
	}else 
	{
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
    DMA_DeInit(DMA_Streamx);   // 进行dma的复位
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;                                       //通道选择
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;                            //DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;                               //DMA 存储器0地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                    //外设到储存器
    DMA_InitStructure.DMA_BufferSize = ndtr;                                   //数据传输量 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;           //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;    //外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;            //存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                            //使用连续模式 
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                      //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;				 //外设突发单次传输
    DMA_Init(DMA_Streamx, &DMA_InitStructure);																 //初始化DMA Stream
	
	USART_DMACmd(USART3,USART_DMAReq_Rx,DISABLE);
	DMA_Cmd(DMA1_Stream1, DISABLE);      
}

void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}

void UART7_IRQHandler(void)
{
    uint8_t ch = 0;
    static uint8_t i = 0;
    if (USART_GetITStatus(UART7, USART_IT_RXNE) == SET)
    {
        USART_ClearITPendingBit(UART7, USART_IT_RXNE);
        ch = USART_ReceiveData(UART7);

        if (i == 1 && ch == 0x0d) //等待一帧完整数据 数据结尾为0x0A 0x0D  越过后打开DMA 保证数据帧完整性
        {
            USART_ITConfig(UART7, USART_IT_RXNE, DISABLE); //关闭串口中断

            USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE); //打开DMA串口接收
            DMA_Cmd(DMA1_Stream3, ENABLE);                // DMA1 数据流3

            TIM_Cmd(TIM4, ENABLE); // OPS数据自检功能开启

            OPS_DMA_ready = 1;
            // OPS对帧结束标志位
            i = 0; //标志位复位 便于下次对帧
        }
        else
            i = 0; //连续接收到 0x0A 0x0D数据时开启DMA
        if (ch == 0x0a)
        {
            i = 1;
        }
    }
}

void OPS_Inspection_TIM_init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); ///使能TIM3时钟

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;                 //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;              //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); //初始化TIM3

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //允许定时器4更新中断
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;           //定时器4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, DISABLE); //使能定时器4
}

int X_for_DMA = 0, Y_for_DMA = 0;
int Z_for_DMA = 0;
int hwt101_DMA = 0;
void TIM4_IRQHandler(void) //中断服务程序
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update)) //进行OPS DMA数据流传输数据自检
    {
        if (OPS.data[2] != 0x0D || OPS.data[3] != 0x0A || OPS.data[28] != 0x0A || OPS.data[29] != 0x0D) // OPS返回数据帧头 帧尾检测
        {
            //若满足条件 进行数据对帧
            TIM_Cmd(TIM4, DISABLE);                        //关闭检测定时器
            USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);  //开启串口中断
            USART_DMACmd(UART7, USART_DMAReq_Rx, DISABLE); //光比DMA串口接收
            DMA_Cmd(DMA1_Stream3, DISABLE);                // DMA数据流关闭
//            LED_OFF;
        }
        else
        {
//            LED_ON;

        }
        if (data_commit_flag)
        {
            //更新LCD屏幕数据
            X_for_DMA = (int)(-OPS.ActVal[4] * 10);
            Y_for_DMA = (int)(-OPS.ActVal[5] * 10);   //在串口
            Z_for_DMA = (int)(-OPS.ActVal[1] * 100); // 在串口屏上显示的时候有两位小数
           
            // printf("X_for_DMA=%d\n\r",X_for_DMA);
            DMA_OPS_buf[1] = X_for_DMA >> 24; //取数据的高8位
            DMA_OPS_buf[2] = X_for_DMA >> 16;
            DMA_OPS_buf[3] = X_for_DMA >> 8;
            DMA_OPS_buf[4] = X_for_DMA;       //取数据的低8位
            DMA_OPS_buf[5] = Y_for_DMA >> 24; //取数据的高8位
            DMA_OPS_buf[6] = Y_for_DMA >> 16;
            DMA_OPS_buf[7] = Y_for_DMA >> 8;
            DMA_OPS_buf[8] = Y_for_DMA; //取数据的低8位

            DMA_OPS_buf[9] = Z_for_DMA >> 24;
            DMA_OPS_buf[10] = Z_for_DMA >> 16;
            DMA_OPS_buf[11] = Z_for_DMA >> 8; //取数据的低8位
            DMA_OPS_buf[12] = Z_for_DMA;      //取数据的低8位



            //遥控器的数据，两字节中的低字节的数据  0-255
            if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET) //等待DMA2_Steam7传输完成
            {
                DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7); //清除DMA2_Steam7传输完成标志
            }
            MYDMA_Enable(DMA2_Stream7, 48);
        }
    }
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

void PC_SendChar(uint8_t DataToSend)
{
    while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData(UART7, DataToSend);
    while (USART_GetFlagStatus(UART7, USART_FLAG_TC) == RESET)
        ;
}

void PC_SendString(uint8_t *str)
{
    while (*str)
    {
        PC_SendChar(*str);
        str++;
    }
}

void Cali_Ops()
{ /////////////清零

	#if USE_ABSOLUTE_POS
	/*此处实现对参考点的移动*/
	Reference_Position.x += pos_x;
	Reference_Position.y += pos_y;
	if(Reference_Position.z + zangle >= 360)		//超360的判断
	{
		Reference_Position.z = Reference_Position.z + zangle - 360;
	}
	else
	{
		Reference_Position.z += zangle;		
	}
	#endif
    OPS.ActVal[4] = 0;
    OPS.ActVal[5] = 0;
    OPS.ActVal[6] = 0;
    OPS.ActVal[7] = 0;
    OPS.ActVal[3] = 0;
    OPS.ActVal[1] = 0;
    OPS.ActVal[2] = 0;
    PC_SendString((uint8_t *)"ACT0");
    OPS.ActVal[4] = 0;
    OPS.ActVal[5] = 0;
    OPS.ActVal[6] = 0;
    OPS.ActVal[7] = 0;
    OPS.ActVal[3] = 0;
    OPS.ActVal[1] = 0;
    OPS.ActVal[2] = 0;
    delay_ms(50);
}

void cali_angle_ops(float angle)
{
    PC_SendString((uint8_t *)"ACTJangle");
}

void Stract(char str1[], uint8_t str2[], u8 num)
{
    int i = 0, j = 0;
    while (str1[i] != '\0')
        i++;

    for (j = 0; j < num; j++)
    {
        str1[i++] = str2[j];
    }
}

void Update_X(float posx)
{
    int i = 0;
    char update_x[8] = "ACTX";
    static union
    {
        float X;
        uint8_t data[4];
    } set;

    set.X = posx;
    Stract(update_x, set.data, 4);

    for (i = 0; i < 8; i++)
    {
        while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET)
            ;
        USART_SendData(UART7, update_x[i]);
    }
}

void Update_Y(float posy)
{
    int i = 0;
    char update_y[8] = "ACTY";
    static union
    {
        float Y;
        uint8_t data[4];
    } set;

    set.Y = posy;
    Stract(update_y, set.data, 4);

    for (i = 0; i < 8; i++)
    {
        while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET)
            ;
        USART_SendData(UART7, update_y[i]);
    }
}

void Update_Z(float posz)
{
    int i = 0;
    char update_z[8] = "ACTJ";
    static union
    {
        float Z;
        uint8_t data[4];
    } set;

    set.Z = posz;
    Stract(update_z, set.data, 4);

    for (i = 0; i < 8; i++)
    {
        while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET)
            ;
        USART_SendData(UART7, update_z[i]);
    }
}

#endif
