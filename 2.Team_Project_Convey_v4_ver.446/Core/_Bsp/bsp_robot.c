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
	HAL_GPIO_WritePin(PIN_SIGNAL_ROBOT_START, GPIO_PIN_SET);
}
