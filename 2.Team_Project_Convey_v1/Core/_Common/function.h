/*
 * includes.h
 *
 *  Created on: Jan 19, 2026
 *      Author: kangs
 */

#ifndef COMMON_FUNCTION_H_
#define COMMON_FUNCTION_H_

#include "main.h"

// drv_uart.c
void DRV_UART_Init(void);
void DRV_UART_RxUpdate(uint8_t* p);
void DRV_UART_TxReport(void);


#endif /* COMMON_FUNCTION_H_ */
