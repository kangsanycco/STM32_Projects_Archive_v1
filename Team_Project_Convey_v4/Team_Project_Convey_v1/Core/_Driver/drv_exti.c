/*
 * drv_exti.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */



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
        // 1. AGV 도착 감지 (PA8 - 분류쪽, PB5 - 적재쪽)
        case GPIO_PIN_8:
                APP_EVENT_Post(EVENT_AGV_SORT_ARRIVED);     // 핀8 인터럽트 발생 -> 분류 AGV 도착
                break;

        case GPIO_PIN_5:
                APP_EVENT_Post(EVENT_AGV_LOAD_ARRIVED);     // 핀5 인터럽트 발생 -> 적재 AGV 도착
                break;

        case GPIO_PIN_10:
                APP_EVENT_Post(EVENT_ITEM_ENTER_ROBOT_ZONE); // 핀10 인터럽트 발생 -> 로봇 존 진입
                break;

        // 2. 로봇 작업 완료 신호 (PB1) - (보류) 중이었으나 인터럽트로 대기
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
