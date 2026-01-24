/*
 * drv_exti.c
 */
#include "config.h"
#include "system_state.h"
#include "function.h"

/**
 * @brief EXTI 인터럽트 통합 콜백 함수
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 1. 리니어 1층 (PA0) - 원점 및 하강 중 급제동
    if (GPIO_Pin == GPIO_PIN_0) {
        g_sys_status.sensor_lift_1f = HAL_GPIO_ReadPin(PIN_SENSOR_LIFT_1F);

        if (g_sys_status.sensor_lift_1f) {
            // [추가] 하강 중 센서 감지 시 DMA 즉시 정지 (치명적 오류 방지)
            if (g_sys_status.liftDirection == LIFT_DIR_DOWN) {
                HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
                g_sys_status.is_lift_busy = 0;
                g_sys_status.current_step_pos = 0;
                g_sys_status.lift_current_floor = 1;
                g_sys_status.liftDirection = LIFT_DIR_STOP;
            }

            // 초기화 모드에서 영점 완료 처리
            if (g_sys_status.mainState == STATE_BOOT) {
                g_sys_status.is_lift_homed = 1;
            }
        }
    }

    // 2. 리니어 2층 (PA1) - 오버런 감지 및 급제동
    else if (GPIO_Pin == GPIO_PIN_1) {
        g_sys_status.sensor_lift_2f = HAL_GPIO_ReadPin(PIN_SENSOR_LIFT_2F);

        if (g_sys_status.sensor_lift_2f) {
            // [추가] 상승 중 오버런 센서 감지 시 DMA 즉시 정지
            if (g_sys_status.liftDirection == LIFT_DIR_UP) {
                HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
                g_sys_status.is_lift_busy = 0;
                g_sys_status.lastError = ERR_LIFT_OVERRUN_2F;
                g_sys_status.liftDirection = LIFT_DIR_STOP;
            }
        }
    }

    // 3. 로봇 구역 (PA10)
    else if (GPIO_Pin == GPIO_PIN_10) {
        g_sys_status.sensor_robot_area = HAL_GPIO_ReadPin(PIN_SENSOR_ROBOT_AREA);
    }

    // 4. 로봇 완료 (PC6)
    else if (GPIO_Pin == GPIO_PIN_6) {
        g_sys_status.sensor_robot_done = HAL_GPIO_ReadPin(PIN_ROBOT_DONE);
        if (g_sys_status.sensor_robot_done) {
            g_sys_status.is_robot_work = 0;
            HAL_GPIO_WritePin(PIN_ROBOT_WORK, GPIO_PIN_RESET);
        }
    }
}
