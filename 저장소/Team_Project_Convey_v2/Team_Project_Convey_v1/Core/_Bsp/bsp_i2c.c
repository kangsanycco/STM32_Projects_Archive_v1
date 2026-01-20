/*
 * bsp_i2c.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#include "system_state.h"
#include "config.h"

// 1. PCA9685 레지스터 및 주소 정의
#define PCA9685_ADDR       (0x40 << 1) // I2C 슬레이브 주소(7비트 0x40을 왼쪽으로 1비트 시프트). 이름표
#define PCA9685_MODE1      0x00        // 모드 설정 레지스터. 전원 스위치
#define PCA9685_PRESCALE   0xFE        // 분주/주파수(Hz) 설정 레지스터. 속도 조절 기어(속도 결정 공식을 적음)
#define LED0_ON_L          0x06        // 0번 채널 모터 제어 시작 지점
	// 칩의 주소를 찾고(ADDR), 깨운다음(MODE1), 속도를 맞춘 뒤(PRESCALE), 각 모터칸(LED0 등)에 값을 적음


/**
 * @brief PCA9685 초기화 (시스템 준비 흐름)
 * 전원 On 시 app_init.c에서 딱 한 번 호출됩니다.
 */
void BSP_I2C_Init_PCA9685(void) {
    uint8_t setupData[2];

    // [Step 1] 모드 설정: Sleep 모드 해제 및 내부 오실레이터 활성화
    setupData[0] = PCA9685_MODE1;
    setupData[1] = 0x01; // ALLCALL 비트 활성화 및 Sleep 해제
    DRV_I2C_Transmit(PCA9685_ADDR, setupData, 2); // 초기화는 Blocking 방식으로 확실하게 완료

    // [Step 2] 주파수 설정: 50Hz (서보 모터 및 컨베이어 드라이버 표준)
    // PRE_SCALE 계산식: round(25MHz / (4096 * 50Hz)) - 1 = 121
    setupData[0] = PCA9685_PRESCALE;
    setupData[1] = 121;		// 십진수
    DRV_I2C_Transmit(PCA9685_ADDR, setupData, 2);
}
	// ADDR의 setupData 첫 번째 칸에 0x00 을 담고, 두 번째 칸에 0x01 을 담아 상자째로 전송한다
    // ADDR의 setupData 첫 번째 칸에 0xFE 를 넣고, 두 번째 칸에 121; 을 담아 상자째로 전송한다
	// MODE1 이라는 공간을 투입한다.

/**
 * @brief 특정 컨베이어 모터의 속도(PWM)를 제어
 * @param channel: config.h에 정의된 MOTOR_CH_... 번호
 * @param value: 0 ~ 4095 사이의 PWM 듀티비
 */
void BSP_I2C_Write_Motor_PWM(uint8_t channel, uint16_t value) {
    uint8_t packet[5];

    // PCA9685는 채널당 4개의 레지스터(ON_L, ON_H, OFF_L, OFF_H)를 사용하여 PWM 파형을 만듭니다.
    // 시작 레지스터 주소 계산: LED0_ON_L(0x06) + (채널번호 * 4)
    uint8_t regAddr = LED0_ON_L + (channel * 4);

    packet[0] = regAddr;    // 전송 시작할 레지스터 주소
    packet[1] = 0x00;       // LEDn_ON_L: 파형이 켜지는 시점 (0)
    packet[2] = 0x00;       // LEDn_ON_H
    packet[3] = (uint8_t)(value & 0xFF);         // LEDn_OFF_L: 파형이 꺼지는 시점 (Low Byte)
    packet[4] = (uint8_t)((value >> 8) & 0x0F);  // LEDn_OFF_H: 파형이 꺼지는 시점 (High Byte)

    /*
     * [출력 흐름 핵심]
     * task_motor가 20ms마다 이 함수를 호출하면,
     * drv_i2c의 '통로 스위치' 로직을 거쳐 DMA 전송이 시작됩니다.
     */
    DRV_I2C_Transmit_DMA(PCA9685_ADDR, packet, 5);
}
