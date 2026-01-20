/*
 * includes.h
 *
 *  Created on: Jan 19, 2026
 *      Author: kangs
 */

#ifndef COMMON_INCLUDES_H_
#define COMMON_INCLUDES_H_

#include "main.h"

// drv_gpio.c
GPIO_PinState DRV_GPIO_Read(GPIO_TypeDef* port, uint16_t pin);

// bsp_gpio.c
int BSP_PLC_IsActive(void);


// drv_i2c.c


// bsp_i2c.c
HAL_StatusTypeDef DRV_I2C_Transmit(uint16_t devAddr, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef DRV_I2C_Transmit_DMA(uint16_t devAddr, uint8_t *pData, uint16_t size);
uint8_t DRV_I2C_GetReadyStatus(void);

// drv_uart.c
void UART_ReportStatus(void);

// --- _Bsp 계층 함수 (bsp_...) ---
int BSP_PLC_IsActive(void);

// PCA9685 관련 (필요시 추가)
void BSP_I2C_Init_PCA9685(void);
void BSP_I2C_Write_Motor_PWM(uint8_t channel, uint16_t value);
void BSP_Robot_SetSafeState(void);

#endif /* COMMON_INCLUDES_H_ */
