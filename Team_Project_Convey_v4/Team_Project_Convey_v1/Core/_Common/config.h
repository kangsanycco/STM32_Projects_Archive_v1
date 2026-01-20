/*
 * config.h
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_


#include "main.h"

// 1. [PLC] 가동 상태
#define PIN_PLC_MOTOR_POWER       GPIOA, GPIO_PIN_9 // PLC 가동 상태 (전체 공정 ON/OFF)
#define PIN_SENSOR_ROBOT_AREA    GPIOA, GPIO_PIN_10 // 로봇 영역(가공 위치) 도착


// 2. [근접 센서] AGV 도착 감지 (Proximity)
#define PIN_SENSOR_SORT_AGV   GPIOA, GPIO_PIN_8 // 분류쪽 AGV 도착
#define PIN_SENSOR_LOAD_AGV   GPIOB, GPIO_PIN_5 // 적재쪽 AGV 도착


// 3. [적외선 센서] 물체 및 적재 감지 (Infrared)
#define PIN_SENSOR_AGV_FULL      GPIOB, GPIO_PIN_10  // 분류 끝단: 적재 초과 감지 (임시)
#define PIN_SENSOR_AGV_EMPTY     GPIOB, GPIO_PIN_11  // 적재 끝단: 비었는지 감지 (임시)
#define PIN_IR_SPARE_1       GPIOC, GPIO_PIN_13 // [여분] 예비 적외선 센서 1


// 4. [로봇 인터페이스] 로봇과 주고받을 신호
#define PIN_WORK_ROBOT_START      GPIOB, GPIO_PIN_0  // [Output] MCU -> 로봇 (작업 시작함)
#define PIN_WORK_ROBOT_DONE       GPIOB, GPIO_PIN_1  // [Input]  로봇 -> MCU (작업 다 함)


// 6. 컨베이어 모터 채널 (PCA9685)
#define MOTOR_CH_MAIN1_CONV  0 // 메인 라인 1
#define MOTOR_CH_MAIN2_CONV  1  // 메인 라인 2 (CONV0 과 통합 고려)
#define MOTOR_CH_SORT_CONV   2  // 분류 대상 물체를 AGV로 보내는 컨베이어
#define MOTOR_CH_LOAD_CONV   3  // AGV로부터 받는 적재쪽 컨베이어


// 7. 통신 및 핸들
extern UART_HandleTypeDef    huart2;  // UART 통신
extern I2C_HandleTypeDef     hi2c1;  // I2C 통신
#define UART_PC_SERVER       huart2
#define I2C_MOTOR_DRV        hi2c1
#endif /* COMMON_CONFIG_H_ */

