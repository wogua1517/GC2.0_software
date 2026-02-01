#ifndef __OPS_H
#define __OPS_H

#include "stm32f4xx.h"                 
#include "delay.h"
#include "System_Config.h"

typedef union{
	
	uint8_t data[32];
	float ActVal[8];
	
}Union_OPS;

extern Union_OPS OPS;

void OPS_Init(void);
void OPS_DMA_Init(u32 bound); 
void OPS_DMA_init(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void OPS_Inspection_TIM_init(u16 arr,u16 psc);
void PC_SendChar(uint8_t DataToSend);
void PC_SendString(uint8_t *str);
void Cali_Ops(void);
void Update_X(float posx);
void Update_Y(float posy);
void Update_Z(float posz);
void Stract(char str1[],uint8_t str2[],u8 num);




#endif




