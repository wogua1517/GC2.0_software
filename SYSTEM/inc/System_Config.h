#ifndef  _SYS_CONFIG_H
#define  _SYS_CONFIG_H

/* Private define ------------------------------------------------------------*/

#define SYSTEM_SUPPORT_OS				0						//定义系统文件夹是否支持UCOS 0,不支持ucos 1,支持ucos

#define USE_BUZZER						1
#define USE_KEY                         1
#define USE_LED							1
#define USE_MOTOR			        	1 
#define USE_OPS							1
#define USE_QRCODE						1
#define USE_RASPI						1
#define USE_SERVO						1
#define USE_SLIDE						1

#define USE_UPPC 						0						//是否使用上位机
#define USE_ABSOLUTE_POS				1						//使用绝对位置

#define	DT_move							700
#define	DT_R							2
#define Move_distance					52		

#define V_pid							800
#define V_rotate						900
#define V_rotate_2						-800
#define V_straight						1400
#define RD								950.0f
#define Len								280.0f
#define D_disc							(Len + Move_distance)

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h" 

#if USE_BUZZER			
	#include "Buzzer.h"
#endif
#if USE_KEY
	#include "Key.h"
#endif
#if USE_LED
	#include "LED.h"
#endif
#if USE_MOTOR
	#include "Motor.h"
#endif
#if USE_OPS
	#include "OPS.h"
#endif
#if USE_QRCODE
	#include "QRcode.h"
#endif
#if USE_RASPI
	#include "Raspi.h"
#endif
#if USE_SERVO
	#include "Servo.h"
#endif
#if USE_SLIDE
	#include "Emm_V5.h"
#endif

#include "delay.h"
#include "Location.h"
#include "Points.h"
#include "sys.h"
#include "Tasks.h"

#include <math.h>
#include <stdlib.h>
#include <arm_math.h>

extern u8 data_commit_flag;
extern __GOOD goods[1+SOURCENUM];
extern __POINT points[1+SOURCENUM+5];				
extern float final_x ;
extern float final_y ;

void System_Resource_Init(void);  
void System_Tasks_Init(void);       	

#endif  

