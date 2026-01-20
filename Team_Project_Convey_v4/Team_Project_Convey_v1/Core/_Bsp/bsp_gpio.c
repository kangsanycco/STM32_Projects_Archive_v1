/*
 * bsp_gpio.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

int BSP_PLC_IsActive(void) {
    // 0(RESET)이면 1(True) 반환, 1(SET)이면 0(False) 반환
    return (DRV_GPIO_Read(PIN_PLC_STATUS) == GPIO_PIN_RESET);
}
