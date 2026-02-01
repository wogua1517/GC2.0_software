#ifndef __MOTOR_H
#define __MOTOR_H

#include "math.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "sys.h"

#define USART6_MAX_RECV_LEN  255
#define USART6_MAX_SEND_LEN  255

void communicate_uart_init(u32 bound);
char *itoa(int num, char *str, int radix);
void send(int vx, int vy, int vz, int z, int motor1, int motor2, int holder);
void u2_printf(char *fmt, ...);

#endif
