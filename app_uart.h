#ifndef __APP_UART_H
#define __APP_UART_H

#include "tl_common.h"

void app_uart_init(void);
void app_uart_irq_proc(void);
void at_print(char * str);
void at_send(char * data, u32 len);

#endif 