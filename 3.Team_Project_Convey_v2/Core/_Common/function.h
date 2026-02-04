/*
 * includes.h
 *
 *  Created on: Jan 19, 2026
 *      Author: kangs
 */

#ifndef COMMON_FUNCTION_H_
#define COMMON_FUNCTION_H_

#include "main.h"


// drv_i2c.c
HAL_StatusTypeDef DRV_I2C_WriteReg(uint8_t reg, uint8_t data);
void DRV_I2C_Init(void);
HAL_StatusTypeDef DRV_I2C_Robot_SendStart(void);
uint8_t DRV_I2C_Robot_ReadStatus(void);
void DRV_I2C_Robot_ReceiveInterrupt(uint8_t *pBuffer);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);

// drv_uart.c
void DRV_UART_Init(void);
void DRV_UART_RxUpdate(uint8_t* p);
void DRV_UART_TxReport(void);

// bsp_sensor.c
uint8_t BSP_Sensor_GetEmptyRackFloor(void);
void BSP_Sensor_UpdateAll(void);

// bsp_servo.c
void BSP_Servo_SetSpeed(uint8_t ch, uint8_t speed);
void BSP_Servo_Control_Speed(void);

// bsp_stepper.c
void BSP_Stepper_SetEnable(uint8_t enable);
void BSP_Stepper_Home(void);
void BSP_Stepper_MoveToFloor(uint8_t floor);
void BSP_Stepper_Interrupt_Handler(void);


// task_system.c
void TASK_System_Execute(void);



#endif /* COMMON_FUNCTION_H_ */


