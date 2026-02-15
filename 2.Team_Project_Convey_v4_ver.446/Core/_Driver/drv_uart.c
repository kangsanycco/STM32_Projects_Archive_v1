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
#include "config.h"
#include "system_state.h"
#include "function.h"
#include "app_fsm.h"

uint8_t rx_uart2_data[8]; // 수신 버퍼 (PC -> MCU)
static uint8_t tx_uart2_data[8]; // 송신용 장부 (static을 붙여서 TxReport 함수가 끝나도 유지되게 함)

void DRV_UART_Init(void) {
    // UART 수신 시작
	HAL_UART_Receive_DMA(UART_PC_SERVER, rx_uart2_data, 8);
}

// 1. 신호 번역용 매핑 테이블 (파일 상단 static 선언)
// 서버의 mainSignal(0, 1, 2)을 내부 MainControlState_t(0, 3, 5)로 매칭
static const MainControlState_t signal_to_state[] = {	// 반드시 static 사용
    STATE_RUNNING,      // 0번 신호: 시작 명령
    STATE_STOP,      // 1번 신호: 정지 명령
    STATE_EMERGENCY  // 2번 신호: 비상 명령
};


// 패킷 해석: 수신 (PC -> MCU)
void DRV_UART_RxUpdate(uint8_t* p) {

	// 1. STX/ETX 검증 (데이터 밀림 방지)
	if (p[0] != 0xFE || p[7] != 0xFF) return;

	// 2. 서버 신호 처리 (배열 매핑 방식)
	uint8_t mainSignal = p[1]; // 서버에서 온 신호 (0, 1, 2)

	if (mainSignal <= 2) {
	    MainControlState_t nextState = signal_to_state[mainSignal];

	    // STOP(1)이나 EMERGENCY(2) 신호가 들어오면 현재 시간을 기록 (타이머 시작 시점)
	    if (mainSignal == 1 || mainSignal == 2) {
	        g_sys_status.stop_timer = HAL_GetTick();
	    }

	    if (!(mainSignal == 0 && g_sys_status.mainState == STATE_RUNNING)) {
	        // 정지나 비상 상황에서 START(0)를 누르면 IDLE로 목적지 변경
	        if (mainSignal == 0 && (g_sys_status.mainState == STATE_STOP || g_sys_status.mainState == STATE_EMERGENCY)) {
	            g_sys_status.mainState = STATE_IDLE;
	        } else {
	            g_sys_status.mainState = nextState;
	        }
	    }
	}

    // 3. 3종 컨베이어 속도 수신 (Byte 2, 3, 4)
    g_sys_status.speed_main_convey = p[2];
    g_sys_status.speed_sort_convey = p[3];

    // 4. 공정 플래그 (Byte 5 - 비트 분리)
    g_sys_status.rx_agv_sort_arrived  = (p[5] >> 0) & 0x01;
    g_sys_status.rx_agv_sort_departed = (p[5] >> 1) & 0x01;


    // 5. 비전 데이터 엣지 트리거 처리 (Byte 6)
    // 이전 값은 0인데 현재 값(p[6])이 1이라면 -> 새로운 물체 발견
    if (g_sys_status.currentScan == 0 && p[6] == 1) {
        visionQ_push(ITEM_LARGE); // 큐에 데이터 삽입
    }
    g_sys_status.currentScan = p[6]; // 현재 상태 업데이트
}

// 상태 보고: 송신 (MCU -> PC)
void DRV_UART_TxReport(void) {

    tx_uart2_data[0] = 0xFE; // STX
    tx_uart2_data[1] = (uint8_t)g_sys_status.mainState; // 통합 공정 상태 보고
    tx_uart2_data[2] = (uint8_t)g_sys_status.speed_main_convey;
    tx_uart2_data[3] = (uint8_t)g_sys_status.speed_sort_convey;
    tx_uart2_data[4] = 0;

    // 장치 상태 통합 비트 매핑 (Byte 5)
    uint8_t s = 0;
    if (g_sys_status.is_robot_work)             s |= (1 << 5); // Bit 5: 로봇 작동중 (0: OFF, 1: ON)
    // if (g_sys_status.signal_robot_done == 1) s |= (1 << 6); // Bit 6: 로봇완료신호 (0: 동작 중, 1: 동작 완료)
    tx_uart2_data[5] = s;

    // 리프트 위치 정보 (Byte 6)
    tx_uart2_data[6] = 0;
    tx_uart2_data[7] = 0xFF; // ETX

    HAL_UART_Transmit_IT(UART_PC_SERVER, tx_uart2_data, 8);
}

// 수신 콜백 함수
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        DRV_UART_RxUpdate(rx_uart2_data);

        HAL_UART_Receive_DMA(UART_PC_SERVER, rx_uart2_data, 8);
    }
}
