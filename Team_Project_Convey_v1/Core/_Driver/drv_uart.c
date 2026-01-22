/*
 * drv_uart.c
 *
 *  Created on: Jan 19, 2026
 *      Author: codelab
 */


/*
 * drv_uart.c
 * 설계된 8바이트 고정 패킷 규격을 완벽히 준수합니다.
 */
#include "function.h"

uint8_t rx_uart2_data[8]; // DMA 수신 버퍼

// UART 초기화: DMA Circular 모드 설정 권장
void DRV_UART_Init(void) {
    HAL_UART_Receive_DMA(UART_PC_SERVER, rx_uart2_data, 8);
}

// 패킷 해석: 수신 (PC -> MCU)
void DRV_UART_RxUpdate(uint8_t* p) {		// 0부터 255까지의 숫자를 갱신하여 저장
    // 1. STX/ETX 검증
    if (p[0] != 0xFE || p[7] != 0xFF) return;
    // 첫 번째 바이트가 0xFE 가 아니거나, 여덟 번째 바이트가 0xFF이 아니라면, 가짜 데이터니까 해석하지 말고 무시하라는 뜻
    // 전기적 노이즈로 데이터가 한 칸씩 밀려 들어옴을 방지

    // 2. 시스템 제어 (Byte 1)
    g_sys_status.rx_uart2_approved = p[1];

    // 만약 긴급 정지 명령(2)이 들어오면 즉시 메인 상태 변경
    if(p[1] == 2) g_sys_status.mainState = STATE_EMERGENCY_ROBOT;
    // STATE_EMERGENCY_ROBOT 을 거친 후, STATE_EMERGENCY 상태가 된다

    // 3. 컨베이어 속도 (Byte 2, 3, 4)
    g_sys_status.speed_main_convey = p[2];
    g_sys_status.speed_sort_convey = p[3];
    g_sys_status.speed_load_convey = p[4];

    // 4. 공정 플래그 (Byte 5 - Bit 분리)
    g_sys_status.agv_sort_parking = (p[5] >> 0) & 0x01;
    g_sys_status.agv_sort_full    = (p[5] >> 1) & 0x01;
    g_sys_status.agv_load_parking = (p[5] >> 2) & 0x01;
    g_sys_status.agv_load_empty   = (p[5] >> 3) & 0x01;
    g_sys_status.is_scan_done     = (p[5] >> 4) & 0x01;
    // & 0x01 의 기능: 해당되지 않는 비트를 깨끗하게 비우고, 그 자리의 비트만 추출하기 위해서이다.
    // 0과 1만 취급하고 나머지 비트는 취급하지 않겠다는 로직이다
}

// 상태 보고: 송신 (MCU -> PC)
void DRV_UART_TxReport(void) {		// 값을 송신
    uint8_t tx_p[8] = {0,};
    // 배열의 모든 칸을 0으로 비우라는 함수
    // 비어진 Reserved 데이터에 불특정한 숫자를 넣지 않고 0으로 통일하겠다는 함수

    tx_p[0] = 0xFE; // STX
    tx_p[1] = (uint8_t)g_sys_status.mainState; // 통합 공정 상태

    // 물리 센서 상태 비트 매핑 (Byte 2)
    uint8_t s = 0;
    if (g_sys_status.sensor_lift_level_1) s |= (1 << 0);	// s의 0번 스위치를 올려라
    if (g_sys_status.sensor_lift_level_2) s |= (1 << 1);	// s의 1번 스위치를 올려라
    if (g_sys_status.sensor_robot_area)   s |= (1 << 2);	// s의 2번 스위치를 올려라
    if (g_sys_status.sensor_rack_full1)   s |= (1 << 3);	// s의 3번 스위치를 올려라
    if (g_sys_status.sensor_rack_full2)   s |= (1 << 4);	// s의 4번 스위치를 올려라
    if (g_sys_status.sensor_robot_done)   s |= (1 << 5);	// s의 5번 스위치를 올려라
    tx_p[2] = s;
    // 다른 비트를 건들지 않고, 해당 비트 자리를 1로 만들라는 함수
    // 6개의 상태의 데이터 전부를 갱신하지 않고, 통신을 축소하는 방법

    tx_p[7] = 0xFF; // ETX

    HAL_UART_Transmit_DMA(UART_PC_SERVER, tx_p, 8);
}

// DMA 콜백
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        DRV_UART_RxUpdate(rx_uart2_data);
    }
}
