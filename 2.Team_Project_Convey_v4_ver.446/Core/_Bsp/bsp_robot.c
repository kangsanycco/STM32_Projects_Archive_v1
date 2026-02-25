/*
 * bsp_robot.c
 *
 *  Created on: Feb 7, 2026
 *      Author: codelab
 */

#include "config.h"
#include "system_state.h"
#include "function.h"

void BSP_Robot_Start_Trigger(void) {
	static uint32_t robot_last_tick = 0;
	static uint8_t robot_flag = 0;

	if (robot_flag == 0){
		HAL_GPIO_WritePin(PIN_SIGNAL_ROBOT_START, GPIO_PIN_SET);
		robot_last_tick = HAL_GetTick();
		robot_flag = 1;
	}
	else if (robot_flag == 1){
		if (HAL_GetTick() - robot_last_tick >= 100){
			HAL_GPIO_WritePin(PIN_SIGNAL_ROBOT_START, GPIO_PIN_RESET);
			robot_flag = 0;
		}
	}
}

