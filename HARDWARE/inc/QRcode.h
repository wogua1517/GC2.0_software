#ifndef __QRCODE_H
#define __QRCODE_H

extern u8 code;

void qrcode_uart_Init(u32 bound);
void UART8_IRQHandler(void);
void get_code(void);
void UART8_Send(uint8_t *data,uint8_t len);

#endif

