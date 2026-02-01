#include "System_Config.h"

#if USE_LED

#include "LED.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);		// 使能GPIOD时钟
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;	// GPIOD14,D15初始化设置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				// 普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		// 100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;			// 下拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);					// 初始化
}


void Flash_OFF(void) 
{
	LED0_OFF;
	LED1_OFF;
}

void Flash_ON(void) 
{
	LED0_ON;
	LED1_ON;
}
#endif




