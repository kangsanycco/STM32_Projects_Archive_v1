/*
 * drv_exti.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */


#include "drv_exti.h"
#include "config.h"
#include "app_event.h"  // 사건 보고를 위해 필요

/**
 * @brief GPIO 외부 인터럽트 콜백 함수
 * STM32 HAL 라이브러리에서 핀 신호가 바뀌면 자동으로 호출됨
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        // 1. PLC 가동 승인 신호 (PA9)
        case GPIO_PIN_9:
            // PLC 신호가 ON(PLC_RUN_SIGNAL)과 같으면 '가동 시작' 사건 보고
            if (HAL_GPIO_ReadPin(PIN_PLC_STATUS) == PLC_RUN_SIGNAL) {
                APP_EVENT_Post(EVENT_PLC_RUN);  // MCU 에 EVENT가 들어왔음을 보고
            } else {
                APP_EVENT_Post(EVENT_PLC_STOP); // MCU 에 EVENT가 들어왔음을 보고
            }
            break;

        // 2. AGV 도착 감지 (PA8 - 분류쪽, PB5 - 적재쪽)
        case GPIO_PIN_8:
            if (HAL_GPIO_ReadPin(PIN_AGV_SORT_CHECK) == SENSOR_DETECTED) {   // 분류쪽 컨베이어 센서에 0V가 감지되면
                APP_EVENT_Post(EVENT_AGV_SORT_ARRIVED);		// AGV가 도착했음을 알림
            }
            break;

        case GPIO_PIN_5:
            if (HAL_GPIO_ReadPin(PIN_AGV_LOAD_CHECK) == SENSOR_DETECTED) {	 // 적재쪽 컨베이어 센서에 0V가 감지되면
                APP_EVENT_Post(EVENT_AGV_LOAD_ARRIVED);		// AGV가 도착했음으 알림
            }
            break;

        // 3. 로봇 정지 구역 물체 감지 (PA10)
        case GPIO_PIN_10:
            if (HAL_GPIO_ReadPin(PIN_IR_ROBOT_AREA) == SENSOR_DETECTED) {	 // 로봇 영역에 센서가 0V가 감지되면
                APP_EVENT_Post(EVENT_ITEM_ENTER_ROBOT_ZONE);	// 로봇 영역에 물품이 들어왔음을 알림
            }
            break;

        // 4. 로봇 작업 완료 신호 (PB1) - (보류) 중이었으나 인터럽트로 대기
//        case GPIO_PIN_1:
//            // "로봇이 활성화 신호를 주었는가?" -> 문장이 자연스럽게 읽힘
//            if (HAL_GPIO_ReadPin(PIN_ROBOT_DONE) == ROBOT_ACTIVE_SIGNAL) {
//                APP_EVENT_Post(EVENT_ROBOT_WORK_COMPLETE);
//            }
//            break;

        default:
            break;
    }
}
