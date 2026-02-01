/*******************************************************************************
* 文件名：Servo.c
* 作  者：Jam
* 日  期：2024/8/26
* LSC系列舵机控制板基于stm32f4的二次开发
*******************************************************************************/
#include "System_Config.h"

#if USE_SERVO
#include "Servo.h"

uint8_t RobotTxBuf[128];  //发送缓存
uint8_t RobotRxBuf[16];
uint16_t batteryVolt;
uint16_t now_position;


u8 UART_RX_BUF[16];
bool isUartRxCompleted = false;

void uartNVICInit(void)
{
		//Usart3 NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}

void ServoCon_uart_Init(int bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10¸
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11¸´
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

	//USART 配置
	USART_InitStructure.USART_BaudRate = bound;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送接收

	USART_Init(USART3, &USART_InitStructure);
	uartNVICInit();
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}




 void uartWriteBuf(uint8_t *buf, uint8_t len)
{
	while (len--) {
		while ((USART3->SR & 0x40) == 0);
		USART_SendData(USART3,*buf++);
	}
}

extern uint8_t RobotRxBuf[16];

void USART3_IRQHandler(void)
{
	uint8_t Res;
	static bool isGotFrameHeader = false;
	static uint8_t frameHeaderCount = 0;
	static uint8_t dataLength = 2;
	static uint8_t dataCount = 0;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) { //判断接收中断
		Res = USART_ReceiveData(USART3);//(USART3->DR);	//取出接收寄存器数据
		if (!isGotFrameHeader) {  //判断帧头
			if (Res == FRAME_HEADER) {
				frameHeaderCount++;
				if (frameHeaderCount == 2) {
					frameHeaderCount = 0;
					isGotFrameHeader = true;
					dataCount = 1;
				}
			} else {
				isGotFrameHeader = false;
				dataCount = 0;
				frameHeaderCount = 0;
			}
		}
		if (isGotFrameHeader) { //接收接收数据部分
			UART_RX_BUF[dataCount] = Res;
			if (dataCount == 2) {
				dataLength = UART_RX_BUF[dataCount];
				if (dataLength < 2 || dataLength > 8) {
					dataLength = 2;
					isGotFrameHeader = false;
				}
			}
			dataCount++;
			if (dataCount == dataLength + 2) {
				if (isUartRxCompleted == false) {
					isUartRxCompleted = true;
					memcpy(RobotRxBuf, UART_RX_BUF, dataCount);
				}
				isGotFrameHeader = false;
			}
		}
	}
}
/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   sevoID:舵机ID，Position:目标位置,Time:转动时间
                    舵机ID取值:0<=舵机ID<=31,Time取值: Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t position, uint16_t Time)
{
	if (servoID > 31 || !(Time > 0)) {  //舵机ID不能大于31,可根据对应控制板修改
		return;
	}
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;    //填充帧头
	RobotTxBuf[2] = 8;
	RobotTxBuf[3] = CMD_SERVO_MOVE;           //数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
	RobotTxBuf[4] = 1;                        //要控制的舵机个数
	RobotTxBuf[5] = GET_LOW_BYTE(Time);       //取得时间的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Time);      //取得时间的高八位
	RobotTxBuf[7] = servoID;                  //舵机ID
	RobotTxBuf[8] = GET_LOW_BYTE(position);   //取得目标位置的低八位
	RobotTxBuf[9] = GET_HIGH_BYTE(position);  //取得目标位置的高八位

	uartWriteBuf(RobotTxBuf, 10);
}

/*********************************************************************************
 * Function:  moveServosByArray
 * Description： 控制多个舵机转动
 * Parameters:   servos[]:舵机结体数组，Num:舵机个数,Time:转动时间
                    0 < Num <= 32,Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServosByArray(RobotServo servos[], uint8_t Num, uint16_t Time)
{
	uint8_t index = 7;
	uint8_t i = 0;
	
	if (Num < 1 || Num > 32 || !(Time > 0)) {
		return;                                          //舵机数不能为零和大与32，时间不能为零
	}
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;      //填充帧头
	RobotTxBuf[2] = Num * 3 + 5;                       //数据长度 = 要控制舵机数*3+5
	RobotTxBuf[3] = CMD_SERVO_MOVE;                    //填充舵机移动指令
	RobotTxBuf[4] = Num;                               //要控制的舵机个数
	RobotTxBuf[5] = GET_LOW_BYTE(Time);                //取得时间的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Time);               //取得时间的高八位

	for (i = 0; i < Num; i++) {                        //循环填充舵机ID和对应目标位置
		RobotTxBuf[index++] = servos[i].ID;              //填充舵机ID
		RobotTxBuf[index++] = GET_LOW_BYTE(servos[i].position); //填充目标位置低八位
		RobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].position);//填充目标位置高八位
	}

	uartWriteBuf(RobotTxBuf, RobotTxBuf[2] + 2);             //发送
}

/*********************************************************************************
 * Function:  moveServos
 * Description： 控制多个舵机转动
 * Parameters:   Num:舵机个数,Time:转动时间,...:舵机ID,转动角，舵机ID,转动角度 如此类推
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
	uint8_t index = 7;
	uint8_t i = 0;
	uint16_t temp;
	va_list arg_ptr;  //

	va_start(arg_ptr, Time); //取得可变参数首地址
	if (Num < 1 || Num > 32) {
		return;               //舵机数不能为零和大与32，时间不能小于0
	}
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;      //填充帧头
	RobotTxBuf[2] = Num * 3 + 5;                //数据长度 = 要控制舵机数 * 3 + 5
	RobotTxBuf[3] = CMD_SERVO_MOVE;             //舵机移动指令
	RobotTxBuf[4] = Num;                        //要控制舵机数
	RobotTxBuf[5] = GET_LOW_BYTE(Time);         //取得时间的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Time);        //取得时间的高八位

	for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
		temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
		RobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
		temp = va_arg(arg_ptr, int);  //可变参数中取得对应目标位置
		RobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //填充目标位置低八位
		RobotTxBuf[index++] = GET_HIGH_BYTE(temp);//填充目标位置高八位
	}

	va_end(arg_ptr);  //置空arg_ptr

	uartWriteBuf(RobotTxBuf, RobotTxBuf[2] + 2);    //发送
}


/*********************************************************************************
 * Function:  runActionGroup
 * Description： 运行指定动作组
 * Parameters:   NumOfAction:动作组序号, Times:执行次数
 * Return:       无返回
 * Others:       Times = 0 时无限循环
 **********************************************************************************/
void runActionGroup(uint8_t numOfAction, uint16_t Times)
{
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;  //填充帧头
	RobotTxBuf[2] = 5;                      //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
	RobotTxBuf[3] = CMD_ACTION_GROUP_RUN;   //填充运行动作组命令
	RobotTxBuf[4] = numOfAction;            //填充要运行的动作组号
	RobotTxBuf[5] = GET_LOW_BYTE(Times);    //取得要运行次数的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Times);   //取得要运行次数的高八位

	uartWriteBuf(RobotTxBuf, 7);            //发送
}

/*********************************************************************************
 * Function:  stopActiongGroup
 * Description： 停止动作组运行
 * Parameters:   Speed: 目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void stopActionGroup(void)
{
	RobotTxBuf[0] = FRAME_HEADER;     //填充帧头
	RobotTxBuf[1] = FRAME_HEADER;
	RobotTxBuf[2] = 2;                //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
	RobotTxBuf[3] = CMD_ACTION_GROUP_STOP;   //填充停止运行动作组命令

	uartWriteBuf(RobotTxBuf, 4);      //发送
}
/*********************************************************************************
 * Function:  setActionGroupSpeed
 * Description： 设定指定动作组的运行速度
 * Parameters:   NumOfAction: 动作组序号 , Speed:目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
	RobotTxBuf[0] = RobotTxBuf[1] = FRAME_HEADER;   //填充帧头
	RobotTxBuf[2] = 5;                       //数据长度，数据帧除帧头部分数据字节数，此命令固定为5
	RobotTxBuf[3] = CMD_ACTION_GROUP_SPEED;  //填充设置动作组速度命令
	RobotTxBuf[4] = numOfAction;             //填充要设置的动作组号
	RobotTxBuf[5] = GET_LOW_BYTE(Speed);     //获得目标速度的低八位
	RobotTxBuf[6] = GET_HIGH_BYTE(Speed);    //获得目标熟读的高八位

	uartWriteBuf(RobotTxBuf, 7);             //发送
}

/*********************************************************************************
 * Function:  setAllActionGroupSpeed
 * Description： 设置所有动作组的运行速度
 * Parameters:   Speed: 目标速度
 * Return:       无返回
 * Others:
 **********************************************************************************/
void setAllActionGroupSpeed(uint16_t Speed)
{
	setActionGroupSpeed(0xFF, Speed);  //调用动作组速度设定，组号为0xFF时设置所有组的速度
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description： 发送获取电池电压命令
 * Parameters:   Timeout：重试次数
 * Return:       无返回
 * Others:
 **********************************************************************************/
void getBatteryVoltage(void)
{
//	uint16_t Voltage = 0;
	RobotTxBuf[0] = FRAME_HEADER;  //填充帧头
	RobotTxBuf[1] = FRAME_HEADER;
	RobotTxBuf[2] = 2;             //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
	RobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  //填充获取电池电压命令

	uartWriteBuf(RobotTxBuf, 4);   //发送
}
void getPosition(uint8_t Num)
{
//	uint16_t Voltage = 0;
	if (Num < 1 || Num > 32) {
		return;               //舵机数不能为零和大与32
	}
	
	RobotTxBuf[0] = FRAME_HEADER;  //填充帧头
	RobotTxBuf[1] = FRAME_HEADER;
	RobotTxBuf[2] = 0x04;          //读取单个舵机角度
	RobotTxBuf[3] = CMD_GET_POSITION;  
	RobotTxBuf[4] = 1;  
	RobotTxBuf[5] = Num;  

	uartWriteBuf(RobotTxBuf, 6);   //发送
}
void receiveHandle()
{
	//可以根据二次开发手册添加其他指令
	if (isUartRxCompleted) {
		isUartRxCompleted = false;
		switch (RobotRxBuf[3]) {
		case CMD_GET_BATTERY_VOLTAGE: //获取电压
			batteryVolt = (((uint16_t)(RobotRxBuf[5])) << 8) | (RobotRxBuf[4]);
			break;
		case CMD_GET_POSITION: //获取角度
			now_position = (((uint16_t)(RobotRxBuf[7])) << 8) | (RobotRxBuf[6]);
			break;
		default:
			break;
		}
	}
}

#endif

