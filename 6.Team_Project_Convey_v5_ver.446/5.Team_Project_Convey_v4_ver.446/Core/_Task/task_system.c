/*
 * task_system.c
 *
 *  Created on: Jan 25, 2026
 *      Author: kangs
 */
#include "config.h"
#include "system_state.h"
#include "function.h"

// 주기 관리를 위한 시간 기록 장부
static uint32_t last_tick_10ms = 0;		// 센서, 리프트 멈춤 신호 등
static uint32_t last_tick_100ms = 0;	// UART 보고, 속도 제어 등

/**
 * @brief 시스템 통합 관리 태스크
 * @note main.c의 while(1) 무한루프에서 호출
 */
void TASK_System_Execute(void) {
    uint32_t current_tick = HAL_GetTick();

    // --- 1. 10ms 주기: 실시간 감시 ---
    if (current_tick - last_tick_10ms >= 10) {
        last_tick_10ms = current_tick;	// 10ms 마다 갱신
        BSP_Sensor_UpdateAll();
    }

    // --- 2. 100ms 주기: 보고 및 실행 ---
    if (current_tick - last_tick_100ms >= 500) {
        last_tick_100ms = current_tick;	 // 100ms 마다 갱신

        // 현재 공정 상태(FSM), 리프트 위치 등을 PC 서버로 보고
        DRV_UART_TxReport();

        // 컨베이어 가동 금지 상황 필터링]
        // 상황 1: 전체 정지(IDLE)나 에러/긴급 상황일 때
        if (g_sys_status.mainState != STATE_RUNNING) {
            g_sys_status.speed_main_convey = 0;
            g_sys_status.speed_sort_convey = 0;
        }
        else {
            // 시스템이 RUNNING 중이더라도 세부 공정에 따라 차단

            // 상황 2: 로봇 작업 중에는 메인(1,2) 컨베이어 정지
            if (g_sys_status.sortState == SORT_ROBOT_WORK) {
                g_sys_status.speed_main_convey = 0;
                g_sys_status.speed_sort_convey = 0;
            }
            else if (g_sys_status.sortState == SORT_IDLE) {
                g_sys_status.speed_main_convey = 0;
                g_sys_status.speed_sort_convey = 0;
            }

        }

        // 서버에서 받은 속도값이나 제어 명령을 실제 서보 드라이버(PCA9685)에 전달
        BSP_Servo_Control_Speed();
    }
}

