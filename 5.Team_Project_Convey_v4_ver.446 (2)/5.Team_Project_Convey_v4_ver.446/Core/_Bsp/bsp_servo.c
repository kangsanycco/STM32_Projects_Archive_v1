/*
 * bsp_servo.c
 *
 *  Created on: Jan 18, 2026
 *      Author: kangs
 */
#include "config.h"
#include "system_state.h"
#include "function.h"

// [내부 함수] 속도(0~100)를 PCA9685용 PWM 값(약 300~600)으로 변환
static uint16_t convert_speed_to_pwm(uint8_t speed) {
    if (speed == 0) return 0; // 속도가 0이면 펄스 정지

    return (uint16_t)((speed * 2.5f) + 350);
    // 0~100을 350~600 범위로 매핑 (350: 저속 회전 시작, 600: 고속)
    // 공식: (speed * 2.5) + 350
    // 350은 기어 마찰을 이기고 실제로 바퀴를 굴리기 위한 최사한의 신호 세기
}

/**
 * @brief 각 채널별 모터 속도 설정
 * @param ch PCA9685 채널 (0~15)
 * @param speed 속도 (0~100)
 */
void BSP_Servo_SetSpeed(uint8_t ch, uint8_t speed) {   // 어떤 채널을 움직일 지는 더 상위 계층에서 진행
    uint16_t pwm_val = convert_speed_to_pwm(speed);	   // 가독성과 여러 변수에 나눠담기 위해 convert_speed_to_pwm 변수를 곧바로 사용하지 않는다

    // PCA9685의 각 채널은 4개의 레지스터를 가짐 (LEDn_ON_L, ON_H, OFF_L, OFF_H)
    // 보통 ON 시간은 0으로 고정하고 OFF 시간(pwm_val)으로 듀티비를 조절함
    uint8_t reg_base = 0x06 + (ch * 4); // 채널당 4바이트씩 점프

    uint8_t data[4];
    data[0] = 0x00;           // LEDn_ON_L, 전기를 켜는 시간
    data[1] = 0x00;           // LEDn_ON_H
    data[2] = pwm_val & 0xFF; // LEDn_OFF_L (& 를 사용하여 하위 8비트를 추출), 모터 속도 결정 (전기를 끄는 시간)
    data[3] = (pwm_val >> 8) & 0x0F; // LEDn_OFF_H (앞자리 4비트를 사용하여 data[2] 와 결합),

    // AI 모드가 켜져있으므로 4바이트를 한 번에 전송 (최적화)
    // AI 모드: 데이터 전송 시, 주소를 일일이 말하지 않아도 다음 칸에 차곡차곡 채워주는 기능
    HAL_I2C_Mem_Write(I2C_MOTOR_DRV, ADDR_I2C_PCA9685, reg_base, I2C_MEMADD_SIZE_8BIT, data, 4, 10);
    // [흐름 요약]
    // 1. 숫자 입력 및 변환
    // 2. 구조체에 숫자 투입 후 분리
    // 3. I2C로 전송 (HAL_I2C_Mem_Write)
}



/**
 * @brief 전역 장부(g_sys_status)의 속도값을 모든 모터에 반영
 */
void BSP_Servo_Control_Speed(void) {
    // 1. 메인 컨베이어 1, 2 (같은 속도)
    BSP_Servo_SetSpeed(MOTOR_CH_MAIN1_CONV, g_sys_status.speed_main_convey);
    BSP_Servo_SetSpeed(MOTOR_CH_MAIN2_CONV, g_sys_status.speed_main_convey);

    // 2. 분류 컨베이어
    BSP_Servo_SetSpeed(MOTOR_CH_SORT_CONV, g_sys_status.speed_sort_convey);

    // 3. 적재 컨베이어
    BSP_Servo_SetSpeed(MOTOR_CH_LOAD_CONV, g_sys_status.speed_load_convey);
}


// [전체 흐름 요약]
// 1. convert_speed_to_pwm : 숫자 선정 및 속도 변환
// 2. BSP_Servo_SetSpeed : 속도를 구조체에 넣고, 분리 및 전송
// 3. BSP_Servo_Control_All : uart2의 데이터인 g_sys_status.speed 값을 바탕으로
// 전체 서보 모터 속도 결정, 함수 내에서 2번 함수(BSP_Servo_SetSpeed) 실행
