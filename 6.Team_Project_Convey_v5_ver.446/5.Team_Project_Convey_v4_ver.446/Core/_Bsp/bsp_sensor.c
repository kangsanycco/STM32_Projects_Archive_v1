/*
 * bsp_sensor.c
 *
 *  Created on: Jan 21, 2026
 *      Author: kangs
 */


#include "config.h"
#include "system_state.h"
#include "function.h"

/**
 * @brief [보고] 모든 센서 물리 상태를 장부에 동기화
 * 현재 상태가 어떤지 확인할 때 사용. OPC-UA에게 정보 전달을 위한 함수
 */
void BSP_Sensor_UpdateAll(void) {

    g_sys_status.sensor_robot_area = HAL_GPIO_ReadPin(PIN_SENSOR_ROBOT_AREA);


}
