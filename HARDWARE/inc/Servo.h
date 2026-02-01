#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx.h"
#include <stdarg.h>
#include <string.h>

#define SOURCENUM 						30					 
#define BLUE_POINT 						(SOURCENUM + 1)
#define BLACK_POINT						(SOURCENUM + 2)
#define GREEN_POINT						(SOURCENUM + 3)
#define RED_POINT 						(SOURCENUM + 4)
#define WHITE_POINT 					(SOURCENUM + 5)

#define POSITION_CHANGE					-33
#define POSITION_0 						((POSITION_2+POSITION_3)/2)
#define POSITION_1 						(220  +5)
#define POSITION_2 						(470  +5)
#define POSITION_3 						(725  +5)
#define POSITION_4 						(965  +5)
#define POSITION_5 						(1210 +5)
#define POSITION_UP 					740
#define POSITION_MID 					1000
#define POSITION_DOWN 					1080

#define R_Speed 						50
#define R_Speed_load 					50

#define FRAME_HEADER 					0x55	// 帧头
#define CMD_SERVO_MOVE 					0x03	// 舵机移动指令
#define CMD_ACTION_GROUP_RUN 			0x06	// 运行动作组指令
#define CMD_ACTION_GROUP_STOP 			0x07    // 停止动作组指令
#define CMD_ACTION_GROUP_SPEED 			0x0B	// 设置动作组运行速度
#define CMD_GET_BATTERY_VOLTAGE 		0x0F  	// 获取电池电压指令
#define CMD_GET_POSITION		 		0x15  	// 获取舵机角度指令

#define GET_LOW_BYTE(A) 				((uint8_t)(A))			// 宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) 				((uint8_t)((A) >> 8))	// 宏函数 获得A的高八位


typedef enum {
	false = 0,
	true = !false 
} bool;

typedef struct _robot_servo_ { 
  uint8_t ID;
  uint16_t position;
} RobotServo;

extern bool isUartRxCompleted;
extern uint8_t RobotRxBuf[16];
extern uint16_t batteryVolt;
extern uint16_t now_position;
extern void receiveHandle(void);


void ServoCon_uart_Init(int bound);
void uartWriteBuf(u8 *buf, u8 len);

void moveServo(uint8_t servoID, uint16_t position, uint16_t Time);
void moveServosByArray(RobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);
void getPosition(uint8_t Num);

#endif
