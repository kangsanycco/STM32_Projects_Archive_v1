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
    // 로봇 구역 (PA10) - 상태만 읽어줌 ( 0: 감지, 1: 비감지 )
    if (GPIO_Pin == GPIO_PIN_10) {
        g_sys_status.sensor_robot_area = HAL_GPIO_ReadPin(PIN_SENSOR_ROBOT_AREA);
    }
}
