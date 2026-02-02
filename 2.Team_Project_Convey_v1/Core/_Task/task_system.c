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

    // --- 1. 10ms 주기: 실시간 감시 (눈) ---
    if (current_tick - last_tick_10ms >= 10) {
        last_tick_10ms = current_tick;	// 10ms 마다 갱신

        // 모든 센서 상태(PA0, PA1, PA4, PA5, PA10)를 읽어 장부에 기록
        BSP_Sensor_UpdateAll();
    }

    // --- 2. 100ms 주기: 보고 및 실행 (입과 발) ---
    if (current_tick - last_tick_100ms >= 100) {
        last_tick_100ms = current_tick;	 // 100ms 마다 갱신

        if (g_sys_status.sortState == SORT_ROBOT_WORK) {
            if (DRV_I2C_Robot_ReadStatus() == ROBOT_STATUS_DONE) {
                g_sys_status.sensor_robot_done = 1;
                g_sys_status.is_robot_work = 0; // 작업 종료 플래그
            }
        }

        // 현재 공정 상태(FSM), 리프트 위치 등을 PC 서버로 보고
        DRV_UART_TxReport();

        // 컨베이어 가동 금지 상황 필터링]
        // 상황 1: 전체 정지(IDLE)나 에러/긴급 상황일 때
        if (g_sys_status.mainState != STATE_RUNNING) {
            g_sys_status.speed_main_convey = 0;
            g_sys_status.speed_sort_convey = 0;
            g_sys_status.speed_load_convey = 0;
        }
        else {
            // 시스템이 RUNNING 중이더라도 세부 공정에 따라 차단

            // 상황 2: 로봇 작업 중에는 메인(1,2) 컨베이어 정지
            if (g_sys_status.sortState == SORT_ROBOT_WORK) {
                g_sys_status.speed_main_convey = 0;
                g_sys_status.speed_sort_convey = 0;
            }


            // 상황 3: 리프트가 이동 중이면 적재(4) 컨베이어 정지 (기구 파손 방지)
            if (g_sys_status.liftDirection != LIFT_DIR_STOP) {
                g_sys_status.speed_load_convey = 0;
            }
        }



        // 서버에서 받은 속도값이나 제어 명령을 실제 서보 드라이버(PCA9685)에 전달
        BSP_Servo_Control_Speed();
    }
}

// [요약] 센서, 모터 출력, UART 보고 등 갱신
// 1. 마지막 기록 틱(10ms, 100ms) 변수 생성
// 2. TASK_System_Execute 함수가 main.c 에서 실행
// 3. 10ms마다 센서 및 신호 갱신
// 4. 100ms 마다 모터 속도 업데이트
