/*
 * drv_uart.c
 *
 *  Created on: Jan 19, 2026
 *      Author: codelab
 */


#include "main.h"
#include "config.h"
#include "system_state.h"
#include <stdio.h>
#include <string.h>

// 전송이 완료되었는지 확인하는 깃발 (0: 전송중, 1: 완료/대기)
volatile uint8_t g_uart_tx_ready = 1;	// 실시간 확인을 해야 하는 변수

/**
 * @brief printf가 호출될 때 내부적으로 실행되는 함수
 */
int __io_putchar(int ch) {		//  printf가 글자를 출력할 때 내부적으로 한 글자씩 호출하는 출구 함수
    static uint8_t temp_ch;		// 매개 변수로 받은 ch 가 사라질 우려가 있어, temp_ch 에 보관할 임시저장소 형성
    temp_ch = (uint8_t)ch;		// printf가 준 데이터(int)를 통신 규격에 맞는 크기(uint8_t) 로 형태를 바꿔 저장

    // 1. 이전 전송이 끝날 때까지는 잠시 기다립니다 (안전 장치)
    while (g_uart_tx_ready == 0);	// 0이라면 반복하고, 1이라면 통과합니다
    g_uart_tx_ready = 0;	// 1을 곧바로 0으로 바꿉니다

    // 2. 인터럽트 방식으로 한 글자 보냅니다
    if (HAL_UART_Transmit_IT(&UART_PC_SERVER, &temp_ch, 1) != HAL_OK) { // 전송을 시도해보고, 그 전송이 실패했다면, HAL_OK: 명령이 성공적으로 전달
        g_uart_tx_ready = 1; // 실패 시 다시 준비 상태로
        return -1;	// 반환에 실패했음을 알림 (-1은 데이터 범위에 들어갈 수 없다)
    }

    return ch;	// printf 에게 데이터 반환. __io_putchar 함수가 끝났으므로, __io_putchar에 다음 글자가 투입됩니다.
}

/**
 * @brief PC 서버(OPC-UA 대응)로 현재 시스템 장부의 핵심 데이터를 보고함
 * 흐름도 6번에 해당함.
 */
void UART_ReportStatus(void) {		//
    // [보고 양식]
    // PLC상태, 현재공정상태, 카메라결과, 로봇작업중여부, 총처리수량
    // 예: 1,2,1,0,15\n
    printf("%d,%d,%d,%d,%lu\r\n", 		// __io_putchar 함수 호출
           g_sys_status.isPlcActive,
           g_sys_status.currentState,
           g_sys_status.pendingResult, // 보류
           g_sys_status.isRobotBusy,
           g_sys_status.totalProcessed);  // 보류
}

/**
 * @brief UART 전송이 완료되면 자동으로 호출되는 함수 (Callback)
 * CubeMX가 만든 인터럽트 시스템이 이 함수를 찾아와 실행합니다.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART_PC_SERVER.Instance) {		// 현재 huart의 주소가 huart2의 주소라면
        // 전송이 끝났으니 다시 준비 상태로 만듭니다.
        g_uart_tx_ready = 1;
    }
}


// 부가 설명
// 0. 초기 상태: 언제든 데이터(Hello)를 보낼 수 있는 상태 (g_uart_tx_ready = 1)
// 1. 대기 단계: [while(ready == 0)] (H)값이 들어오면 통과시킵니다. (g_uart_tx_ready = 1)
// 2. 잠금 단계: [ready = 0] (H)값이 통과하면서 ready를 0으로 만들었기 때문에, 다른 글자(e)가 들어오면 while에서 멈추게 됩니다. (g_uart_tx_ready = 0)
// 3. 발송 시작: [HAL] 데이터를 쏘고 있지만, 여전히 잠금 상태 (g_uart_tx_ready = 0)
// 4. 전송 완료: [return] H의 전송이 끝낫지만, 아직 보고(Callback) 전이라 문은 잠겨있습니다. 이 때 __io_putchar 의 함수 사용이 끝났으므로 e 투입이 진행됩니다
// 5. 해제 단계: [TxCpltCallback] 보고 완료. 다음 글자가 들어올 수 있게 문을 엽니다 (g_uart_tx_ready = 1)
