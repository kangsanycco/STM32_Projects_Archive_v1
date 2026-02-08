/*
 * config.h
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_


#include "main.h"


// 1. [PIN] 로봇
// 1속성: PIN / 2속성: 주체 / 3속성: 대상
#define PIN_SIGNAL_ROBOT_START  GPIOC, GPIO_PIN_7
#define PIN_SENSOR_ROBOT_AREA   GPIOA, GPIO_PIN_10  // 로봇 작업 영역 (적외선)


// 3. 로봇(램스보드 1.4)와 주고 받을 신호
//#define ROBOT_ORDER_START  0x01  // 램스보드에 시작하라고 보내는 신호
#define ROBOT_WORK_TIME_MS   5000  // 로봇 작업 예상 소요 시간 (5초)




// 4. 비전 데이터 저장용 큐 크기
#define VISION_QUEUE_SIZE    10


// 5. 컨베이어 모터 채널 (PCA9685)
#define MOTOR_CH_MAIN1_CONV  0  // 메인 라인 1 컨베이어
#define MOTOR_CH_MAIN2_CONV  4  // 메인 라인 2 컨베이어(CONV0 과 통합)
#define MOTOR_CH_SORT_CONV   8  // 분류 파트 컨베이어
//#define MOTOR_CH_LOAD_CONV   12  // 적재 파트 컨베이어


// 6. [ADDR] I2C 주소
#define ADDR_I2C_PCA9685     (0x40 << 1)
//#define ADDR_I2C_ROBOT     (0x08 << 1)


// 7. [HAL] 통신 핸들
extern UART_HandleTypeDef huart2;	// PC 서버(OPC-UA 연동), 비전, AGV 데이터
extern I2C_HandleTypeDef  hi2c1;	// PCA9685 드라이버
#define UART_PC_SERVER    &huart2
#define I2C_MOTOR_DRV     &hi2c1


// [선언 이유]
// 1. huart2, htim2 등의 메모리는 main.c 에만 생성이 됩니다
// 2. 하지만, 분리된 파일인 drv, bsp 등은 해당 메모리의 존재를 모르기에, extern 을 선언해줘야 합니다
// 3. extern 은 아무 파일에서나 선언해줘도 좋지만, 가독성을 위해 config.h 에 저장을 합니다


#endif /* COMMON_CONFIG_H_ */
