/*
 * drv_gpio.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#include "main.h"

GPIO_PinState DRV_GPIO_Read(GPIO_TypeDef* port, uint16_t pin) {
    return HAL_GPIO_ReadPin(port, pin);
}
