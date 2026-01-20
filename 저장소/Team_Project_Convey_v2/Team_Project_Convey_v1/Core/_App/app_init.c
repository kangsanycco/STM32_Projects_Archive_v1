/*
 * app_init.c
 *
 * Created on: Jan 17, 2026
 * Author: kangs
 */

#include "system_state.h"
#include "app_event.h"
// 추후 구현될 계층들 포함
// #include "drv_i2c.h"
// #include "bsp_servo.h"

/**
 * [중요] g_sys_status 장부의 실체 선언
 * .h의 extern 선언이 이 실체를 가리키게 되어 메모리에 자리가 잡힙니다.
 */
SystemStatus_t g_sys_status;	// 전역 상황판

/**
 * @brief 시스템 통합 초기화 함수
 * main.c의 while(1) 직전에 이 함수를 호출합니다.
 */
void APP_Init(void)
{
    // 1. 시스템 장부(system_state) 초기화
    g_sys_status.currentState = STATE_IDLE;			// 최신 상태: 시스템 대기
    g_sys_status.pendingResult = ITEM_NOT_SCANNED;	// 카메라 데이터: 물품 없음
    g_sys_status.lastError = ERR_NONE;				// 에러 상태: 에러 없음
    g_sys_status.totalProcessed = 0;				// 총 처리 수량: 0

    // 센서 상태들 초기화
    g_sys_status.isPlcActive = 0;					// PLC 상태 OFF (컨베이어 상태 OFF)
    g_sys_status.isItemInRobotArea = 0;				// 로봇 AREA 물품 감지 OFF
    g_sys_status.isRobotDoneSignal = 0;				// 로봇으로 부터 받은 완료 신호 OFF
    g_sys_status.isSortAgvReady = 0;				// AGV 분류 파트 도착 신호 OFF
    g_sys_status.isLoadAgvReady = 0;				// AGV 적재 파트 도착 신호 OFF
    g_sys_status.isRobotBusy = 0;					// 로봇 작동중(임시)

    // 2. 하드웨어 실제 상태와 장부 동기화 (Snapshot)
    // 초기화 이후 인터럽트가 놓칠 수 있는 지금 당장의 상태를 확인합니다. 시작 전에 이미 놓여져 있는 상태를 감지합니다
    // 인터럽트는 Falling Edge 일 때 발생하는데, 시작 상황부터 이미 LOW 라면 사건으로 인식하지 않을 수 있습니다. 이를 방지합니다
    g_sys_status.isPlcActive       = (HAL_GPIO_ReadPin(PIN_PLC_STATUS) == PLC_RUN_SIGNAL);		// PLC(PA9)의 신호가 GPIO_PIN_RESET 상태라면 1을 출력
    g_sys_status.isItemInRobotArea = (HAL_GPIO_ReadPin(PIN_IR_ROBOT_AREA) == SENSOR_DETECTED);	// 로봇 영역 적외선 센서(PA10)가 GPIO_PIN_RESET 상태라면 1을 출력
    g_sys_status.isSortAgvReady    = (HAL_GPIO_ReadPin(PIN_AGV_SORT_CHECK) == SENSOR_DETECTED);	// 분류 AGV 파트 근접 센서(PA8)가 GPIO_PIN_RESET 상태라면 1을 출력
    g_sys_status.isLoadAgvReady    = (HAL_GPIO_ReadPin(PIN_AGV_LOAD_CHECK) == SENSOR_DETECTED);	// 적재 AGV 파트 근접 센서(PB5)가 GPIO_PIN_RESET 상태라면 1을 출력

    // 3. [보완] 출력 핀 안전 상태 확보
    // 전원 켜지자마자 로봇이 멋대로 움직이지 않게 신호를 IDLE로 고정
    HAL_GPIO_WritePin(PIN_ROBOT_START, ROBOT_IDLE_SIGNAL);	// 로봇과 연결된 핀(임시)의 전압을 GPIO_PIN_SET으로 만듦

    // 4. 초기 상태 알림 (디버깅용)
    // printf("System Initialized. Current State: IDLE\n");
}
