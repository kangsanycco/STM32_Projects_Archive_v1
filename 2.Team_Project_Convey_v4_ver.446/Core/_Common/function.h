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

// drv_uart.c
void DRV_UART_Init(void);
void DRV_UART_RxUpdate(uint8_t* p);
void DRV_UART_TxReport(void);

// bsp_robot.c
void BSP_Robot_Start_Trigger(void);

// bsp_sensor.c
void BSP_Sensor_UpdateAll(void);

// bsp_servo.c
void BSP_Servo_SetSpeed(uint8_t ch, uint8_t speed);
void BSP_Servo_Control_Speed(void);


// task_system.c
void TASK_System_Execute(void);



#endif /* COMMON_FUNCTION_H_ */


