#ifndef __RASPI_H
#define __RASPI_H


#include "sys.h" 
#include "delay.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"	
#include "stdlib.h"

#define USART2_MAX_RECV_LEN		20					//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		20					//最大发送缓存字节数
#define USART2_RX_EN 			1					//0,不接收;1,接收.

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		//接收缓冲,最大USART3_MAX_RECV_LEN字节
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		//发送缓冲,最大USART3_MAX_SEND_LEN字节



extern u8 rxd_flag;//接收标志
extern u8 rxd_buf[8];//定义数组接收数据包，定长
void Raspi_uart_Init(u32 bound);
void Raspi_printf(char* fmt,...);


#endif
