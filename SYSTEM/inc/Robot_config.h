#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

/* Configuration ----------------------------------------------*/
#define USE_UPPC 						0						//是否使用上位机
#define USE_ABSOLUTE_POS				1						//使用绝对位置

#define SOURCENUM						30						//物块数量
#define BLUE_POINT						(SOURCENUM + 1)
#define BLACK_POINT						(SOURCENUM + 2)
#define GREEN_POINT						(SOURCENUM + 3)
#define RED_POINT						(SOURCENUM + 4)
#define WHITE_POINT						(SOURCENUM + 5)

/* 舵机点位----------------------------------------------------*/
#define POSITION_0						750
#define POSITION_1                      0
#define POSITION_2                      250
#define POSITION_3                      500
#define POSITION_4                      1000
#define POSITION_5                      1250
#define POSITION_UP                     630
#define POSITION_DOWN                   263

#define	R_Speed							50
#define R_Speed_load					50

#define	DT_move							700
#define	DT_R							2

#define Move_distance					35

#define final_x							165
#define final_y							87

/* Drivers ----------------------------------------------------*/
#define USE_CAN                         1
#define USE_DMA                         1
#define USE_USART                       1

#define USE_BEEP                        1
#define USE_COLOR                       1
#define USE_HWT101                      1
#define USE_JR6001                      1       
#define USE_KEY                         1
#define USE_LASER                       1
#define USE_LED							1
#define USE_OLED						1
#define USE_OPS							1
#define USE_PS2							1
#define USE_QRCODE						1
#define USE_VOICE						1
#define USE_BLE							1



/* Middlewares -----------------------------------------------*/
#define USE_ACTION                      1
#define USE_CONTROL                     1
#define USE_SERVOCON                    1
#define USE_LOCATION                    1
#define USE_COMMUNICATE                 1      
#define USE_DATACOM                     1
#define USE_ALGORITHM					1


/* Includes ------------------------------------------------------------------*/
/**
* @brief Include module's header file
*/

/* Drivers ----------------------------------------------------*/

/* Components header begin */
#if USE_CAN
  #include "Drivers/Components/can/can.h"
#endif
#if USE_DMA
  #include "Drivers/Components/dma/dma.h"
#endif
#if USE_USART
  #include "usart.h"
#endif
/* Components header end */

/* Devices header begin */
#if USE_BEEP
  #include "Drivers/Devices/beep/beep.h"
#endif
#if USE_COLOR
  #include "Drivers/Devices/color/color.h"
#endif
#if USE_HWT101
  #include "Drivers/Devices/hwt101/hwt101.h"
#endif
#if USE_JR6001
  #include "Drivers/Devices/jr6001/jr6001.h"
#endif
#if USE_KEY
  #include "Drivers/Devices/key/key.h"
#endif
#if USE_LASER
  #include "Drivers/Devices/laser/laser.h"
#endif
#if USE_LED
  #include "Drivers/Devices/led/led.h"
#endif
#if USE_OLED
  #include "Drivers/Devices/oled/oled.h"
#endif
#if USE_OPS
  #include "Drivers/Devices/ops/ops.h"
#endif
#if USE_PS2
  #include "Drivers/Devices/ps2/ps2.h"
#endif
#if USE_QRCODE
  #include "Drivers/Devices/qrcode/qrcode.h"
#endif
#if USE_VOICE
  #include "Drivers/Devices/voice/voice.h"
#endif
#if USE_BLE
  #include "Drivers/Devices/BLE/BLE.h"
#endif

/* Devices header end */

/* Middlewares -----------------------------------------------*/
/* Agorithm header begin */
#if USE_ALGORITHM
	#include "Middlewares/Algorithm/algorithm.h"
#endif
/* Agorithm header begin */
/* Actions header begin */
#if USE_ACTION
  #include "Middlewares/action/action.h"
#endif
#if USE_SERVOCON
  #include "Middlewares/lobotServoController/lobotServoController.h"
#endif
#if USE_LOCATION
  #include "Middlewares/location/location.h"
#endif
/* Actions header end */

/* Communications header begin */
#if USE_COMMUNICATE
  #include "Middlewares/communicate/communicate.h"
#endif
#if USE_DATACOM
  #include "Middlewares/data_commit/data_commit.h"
#endif
/* Communications header end */
#endif
