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
 * @brief [판단] 적재 가능한 빈 랙의 층수를 반환
 * @return 0: 가득 참(대기), 1: 1층 비었음, 2: 2층 비었음
 */
uint8_t BSP_Sensor_GetEmptyRackFloor(void) {
    // 2층을 우선적으로 채우는 시나리오 (사용자 선택 가능)
    if (HAL_GPIO_ReadPin(PIN_SENSOR_RACK_2F) == GPIO_PIN_SET) {
        return 2; // 2층이 비었으면 바로 2층 결정
    }

    if (HAL_GPIO_ReadPin(PIN_SENSOR_RACK_1F) == GPIO_PIN_SET) {
        return 1; // 2층은 찼고 1층이 비었으면 1층 결정
    }

    return 0; // 둘 다 꽉 찼으면 0 반환 (나중에 FSM에서 대기 처리)
}

/**
 * @brief [보고] 모든 센서 물리 상태를 장부에 동기화
 * 현재 상태가 어떤지 확인할 때 사용. OPC-UA에게 정보 전달을 위한 함수
 */
void BSP_Sensor_UpdateAll(void) {
    g_sys_status.sensor_rack_full_1f = HAL_GPIO_ReadPin(PIN_SENSOR_RACK_1F);
    g_sys_status.sensor_rack_full_2f = HAL_GPIO_ReadPin(PIN_SENSOR_RACK_2F);
    g_sys_status.sensor_robot_area = HAL_GPIO_ReadPin(PIN_SENSOR_ROBOT_AREA);
    g_sys_status.sensor_lift_1f = HAL_GPIO_ReadPin(PIN_SENSOR_LIFT_1F);
    g_sys_status.sensor_lift_2f = HAL_GPIO_ReadPin(PIN_SENSOR_LIFT_2F);

    // [EXTI의 정보가 묻히는 거 아닌가에 대한 답변]
    // 1. 핀의 물리적 상태가 EXTI의 물리적 상태를 묻으려면 서로 데이터가 달라야 한다
    // 2. 하지만 해당 UpdateAll 함수는 주기적으로 상태를 체크하는 것이므로, 다루는 핀의 데이터가 다를 수가 없다
    // 3. 즉, EXTI 는 비일정주기, UpdateAll 은 일정 주기
}





