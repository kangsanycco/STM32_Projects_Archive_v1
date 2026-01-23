/*
 * drv_uart.c
 *
 *  Created on: Jan 19, 2026
 *      Author: codelab
 */


/*
 * drv_uart.c
 * 설계된 8바이트 고정 패킷 규격(v1.1)을 완벽히 준수합니다.
 */
#include "function.h"

uint8_t rx_uart2_data[8]; // DMA 수신 버퍼 (PC -> MCU)

void DRV_UART_Init(void) {
    // UART 수신 시작 (DMA Circular 모드 설정 전제)
    HAL_UART_Receive_DMA(UART_PC_SERVER, rx_uart2_data, 8);
}

// 패킷 해석: 수신 (PC -> MCU)
void DRV_UART_RxUpdate(uint8_t* p) {
    // 1. STX/ETX 검증 (데이터 밀림 방지)
    if (p[0] != 0xFE || p[7] != 0xFF) return;

    // 2. 시스템 상태 수신 (Byte 1)
    g_sys_status.mainState = (MainControlState_t)p[1];

    // 3. 3종 컨베이어 속도 수신 (Byte 2, 3, 4)
    g_sys_status.speed_main_convey = p[2];
    g_sys_status.speed_sort_convey = p[3];
    g_sys_status.speed_load_convey = p[4];

    // 4. 공정 플래그 (Byte 5 - 비트 분리)
    g_sys_status.rx_agv_sort_arrived  = (p[5] >> 0) & 0x01;
    g_sys_status.rx_agv_sort_departed = (p[5] >> 1) & 0x01;
    g_sys_status.rx_agv_load_arrived  = (p[5] >> 2) & 0x01;
    g_sys_status.rx_agv_load_departed = (p[5] >> 3) & 0x01;

    // 5. 비전 데이터 엣지 트리거 처리 (Byte 6)
    // 이전 값은 0인데 현재 값(p[6])이 1이라면 -> 새로운 물체 발견!
    if (g_sys_status.currentScan == 0 && p[6] == 1) {
        visionQ_push(ITEM_LARGE); // 큐에 데이터 삽입
    }
    g_sys_status.currentScan = p[6]; // 현재 상태 업데이트
}

// 상태 보고: 송신 (MCU -> PC)
void DRV_UART_TxReport(void) {
    uint8_t tx_p[8] = {0,}; // Reserved 영역 0으로 초기화

    tx_p[0] = 0xFE; // STX
    tx_p[1] = (uint8_t)g_sys_status.mainState; // 통합 공정 상태 보고

    // 3종 컨베이어 현재 속도 보고 (Byte 2, 3, 4)
    tx_p[2] = (uint8_t)g_sys_status.speed_main_convey;
    tx_p[3] = (uint8_t)g_sys_status.speed_sort_convey;
    tx_p[4] = (uint8_t)g_sys_status.speed_load_convey;

    // 장치 상태 통합 비트 매핑 (Byte 5)
    uint8_t s = 0;
    s |= (g_sys_status.liftDirection & 0x03);    // Bit 0-1: 리니어 방향 (0,1,2)
    if (g_sys_status.is_lift_busy)   s |= (1 << 2); // Bit 2: 리니어 이동중
    if (g_sys_status.is_robot_work)  s |= (1 << 3); // Bit 3: 로봇 작동중
    if (g_sys_status.sensor_lift_1f) s |= (1 << 4); // Bit 4: 1층 도착
    if (g_sys_status.sensor_lift_2f) s |= (1 << 5); // Bit 5: 2층 도착
    if (g_sys_status.sensor_robot_done) s |= (1 << 6); // Bit 6: PC6 로봇완료신호
    tx_p[5] = s;

    // 리프트 위치 정보 (Byte 6)
    tx_p[6] = (uint8_t)g_sys_status.lift_current_floor; // 1층 또는 2층

    tx_p[7] = 0xFF; // ETX

    HAL_UART_Transmit_DMA(UART_PC_SERVER, tx_p, 8);
}

// DMA 수신 콜백 함수
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        DRV_UART_RxUpdate(rx_uart2_data);
    }
}
