/*
 * drv_exti.c
 */
#include "config.h"
#include "system_state.h"
#include "function.h"

/*
 * drv_exti.c
 * - 드라이버 계층: 센서 감지 시 물리적 정지만 수행
 */
#include "config.h"
#include "system_state.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 1. 리니어 1층 (PA6) - 원점/하강 중 감지
    if (GPIO_Pin == GPIO_PIN_6) {
        g_sys_status.sensor_lift_1f = HAL_GPIO_ReadPin(PIN_SENSOR_LIFT_1F);

        // 하강 중에 센서가 감지되면 무조건 모터부터 세움 (물리적 보호)
        if (!g_sys_status.sensor_lift_1f && g_sys_status.liftDirection == LIFT_DIR_DOWN) {
            HAL_TIM_Base_Stop_IT(&htim2);
            g_sys_status.is_lift_busy = 0;
            // 층수 업데이트나 pos=0 초기화는 app_fsm.c에서 처리하도록 제거함
        }
    }

    // 2. 리니어 2층 (PA7) - 오버런 감지
    else if (GPIO_Pin == GPIO_PIN_7) {
        g_sys_status.sensor_lift_overrun_2f = HAL_GPIO_ReadPin(PIN_SENSOR_LIFT_2F);

        // 상승 중에 2층 센서 감지 시 즉시 정지
        if (!g_sys_status.sensor_lift_overrun_2f && g_sys_status.liftDirection == LIFT_DIR_UP) {
            HAL_TIM_Base_Stop_IT(&htim2);
            g_sys_status.is_lift_busy = 0;
        }
    }

    // 3. 로봇 구역 (PA10) - 상태만 읽어줌
    else if (GPIO_Pin == GPIO_PIN_10) {
        g_sys_status.sensor_robot_area = HAL_GPIO_ReadPin(PIN_SENSOR_ROBOT_AREA);
    }
}
