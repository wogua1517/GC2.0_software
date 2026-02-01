#ifndef __LED_H
#define __LED_H


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


//LED端口定义
#define LED0 PFout(14)	// DS0
#define LED1 PFout(15)	// DS1	 

#define LED0_ON  PDout(14)=1
#define LED0_OFF PDout(14)=0
#define LED1_ON  PDout(15)=1
#define LED1_OFF PDout(15)=0


void LED_Init(void);//初始化	
void Flash_OFF(void);
void Flash_ON(void);
#endif
