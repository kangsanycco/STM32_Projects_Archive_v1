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

// drv_i2c.c
void DRV_I2C_Init(void);
HAL_StatusTypeDef DRV_I2C_WriteReg(uint8_t reg, uint8_t data);

#endif /* COMMON_FUNCTION_H_ */
