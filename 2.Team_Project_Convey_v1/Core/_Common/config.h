/*
 * config.h
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#ifndef COMMON_CONFIG_H_
#define COMMON_CONFIG_H_


#include "main.h"


// 1. [PIN] 센서류 (입력)
// 1속성: PIN_Sensor / 2속성: 주체 / 3속성: 대상
#define PIN_SENSOR_ROBOT_AREA   GPIOA, GPIO_PIN_10  // 로봇 작업 영역 (적외선)
#define PIN_SENSOR_LIFT_1F  	GPIOA, GPIO_PIN_0   // 리니어 1층 도착 및 원점 (근접)
#define PIN_SENSOR_LIFT_2F  	GPIOA, GPIO_PIN_1   // 리니어 2층 범위 이탈 감지 (근접)
#define PIN_SENSOR_RACK_1F	 	GPIOA, GPIO_PIN_4   // 1층 랙 물품 유무 (적외선)
#define PIN_SENSOR_RACK_2F	 	GPIOA, GPIO_PIN_5   // 2층 랙 물품 유무 (적외선)


// 2. [PIN] 리니어 스텝 모터 (42H48H1704A2)
#define PIN_STEP_LIFT_PULSE			GPIOB, GPIO_PIN_12  // 펄스(속도)
#define PIN_STEP_LIFT_DIRECTION		GPIOB, GPIO_PIN_13	// 방향
#define PIN_STEP_LIFT_ENABLE		GPIOB, GPIO_PIN_14	// 활성화


// 3. [PIN] 로봇 인터페이스 (물리 신호 필요 시) - (임시: 서버로 받을지, 핀으로 받을지 고려)
#define PIN_ROBOT_WORK     GPIOC, GPIO_PIN_7 // 로봇 동작 시작 (OUT), UART2 PA2와 인터럽트 충돌은 없음
#define PIN_ROBOT_DONE     GPIOC, GPIO_PIN_6 // 로봇 동작 완료 (IN), UART2 PA3과 인터럽트 충돌은 없음


// 4. 비전 데이터 저장용 큐 크기
#define VISION_QUEUE_SIZE    10


// 5. 컨베이어 모터 채널 (PCA9685)
#define MOTOR_CH_MAIN1_CONV  0  // 메인 라인 1 컨베이어
#define MOTOR_CH_MAIN2_CONV  1  // 메인 라인 2 컨베이어(CONV0 과 통합)
#define MOTOR_CH_SORT_CONV   2  // 분류 파트 컨베이어
#define MOTOR_CH_LOAD_CONV   3  // 적재 파트 컨베이어


// 6. [ADDR] I2C 주소
#define ADDR_I2C_PCA9685     (0x40 << 1)


// 7. [HAL] 통신 핸들
extern UART_HandleTypeDef huart2;	// PC 서버(OPC-UA 연동), 비전, AGV 데이터
extern I2C_HandleTypeDef  hi2c1;	// PCA9685 드라이버
extern TIM_HandleTypeDef  htim2;	// 리니어 스텝 모터 제어용 타이머
#define UART_PC_SERVER    &huart2
#define I2C_MOTOR_DRV     &hi2c1
#define TIMER_LIFT		  &htim2

// [선언 이유]
// 1. huart2, htim2 등의 메모리는 main.c 에만 생성이 됩니다
// 2. 하지만, 분리된 파일인 drv, bsp 등은 해당 메모리의 존재를 모르기에, extern 을 선언해줘야 합니다
// 3. extern 은 아무 파일에서나 선언해줘도 좋지만, 가독성을 위해 config.h 에 저장을 합니다


#endif /* COMMON_CONFIG_H_ */

