/*
 * app.fsm.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */
#include "config.h"
#include "system_state.h"
#include "function.h"

/* [내부 유틸리티] 비전 데이터 큐 관리 */
void visionQ_push(CameraResult_t item) {	// 값 투입
    if (g_sys_status.visionQ.count < VISION_QUEUE_SIZE) {
        g_sys_status.visionQ.slot[g_sys_status.visionQ.tail] = item;
        g_sys_status.visionQ.tail = (g_sys_status.visionQ.tail + 1) % VISION_QUEUE_SIZE;
        g_sys_status.visionQ.count++;

        // [함수 요약]
        // 1. 값이 들어오면 drv_uart.c 에서 visionQ_push 함수가 실행된다
        // 2. 카운트가 큐 사이즈보다 크다면 슬롯에 투입이 중단된다
        // 3. 만약에 카운트가 위의 조건을 만족한다면, 슬롯에 데이터를 투입한다. 이 때 기존의 데이터가 있다면, 뒤덮는다(버린다).
        // 4. 데이터가 투입되면, 다음 투입 칸은 1이 추가되며, 칸 수는 큐 사이즈에 맞게 순환된다
        // 5. 카운트는 1이 추가된다
    }
}

CameraResult_t visionQ_pop(void) {	 // 값 빼기, visionQ_pop 함수가 끝나면 CameraResult_t 규격으로 값을 반환한다라고 선언
    if (g_sys_status.visionQ.count == 0) return ITEM_NONE;
    CameraResult_t item = g_sys_status.visionQ.slot[g_sys_status.visionQ.head];
    g_sys_status.visionQ.head = (g_sys_status.visionQ.head + 1) % VISION_QUEUE_SIZE;
    g_sys_status.visionQ.count--;
    return item;

    // [함수 요약]
    // 1. 값이 나가면 app_fsm.c 파일 APP_FSM_Execute에서 visionQ_pop 함수가 실행된다
    // 2. 카운트가 0이 되면, 아이템이 없다라고 반환된다
    // 3. 물품의 값은 slot의 head(출구칸)에 있는 값이다. 이 때 물품의 값은 CameraResult_t 의 규격을 따라간다
    // 4. 데이터가 빠지면, 다음 퇴출 칸은 1이 추가되며, 칸 수는 큐 사이즈에 맞게 순환된다. 이 때 지나간 데이터는 버려진 데이터로 남게 된다.
    // 5. 카운트는 1이 감소된다
}

/**
 * @brief 시스템 초기화 및 영점 복귀
 */
void APP_FSM_Init(void) {

    g_sys_status.mainState = STATE_BOOT;

    // 카메라 비전 큐 초기화
    g_sys_status.visionQ.count = 0;
    g_sys_status.visionQ.head = 0;
    g_sys_status.visionQ.tail = 0;

    // 리니어 모터 영점 잡기 (bsp_stepper.c 호출)
    BSP_Stepper_Home();

    if (g_sys_status.is_lift_homed) {
        g_sys_status.mainState = STATE_IDLE;
    }
}

/**
 * @brief 전체 공정 시나리오 실행 (FSM)
 */
void APP_FSM_Execute(void) {
    // 0. 긴급 정지 체크 (최우선순위, 모든 장비 즉시 정지)
    if (g_sys_status.mainState == STATE_EMERGENCY) {
        g_sys_status.speed_main_convey = 0;
        g_sys_status.speed_sort_convey = 0;
        g_sys_status.speed_load_convey = 0;
        BSP_Stepper_SetEnable(0); // 리니어 모터 힘 풀기

        // [흐름도 3번 반영] 로봇은 긴급 정지 시에도 하던 작업은 끝내야 함
        if (g_sys_status.is_robot_work) {
            if (g_sys_status.sensor_robot_done) {
                g_sys_status.is_robot_work = 0;
                HAL_GPIO_WritePin(PIN_ROBOT_WORK, GPIO_PIN_RESET);
            }
        }
        return;

        // [함수 요약]

    }

    // 1. 대기 상태 (IDLE) - 가동 승인 대기
    if (g_sys_status.mainState == STATE_IDLE) {
        if (g_sys_status.rx_uart2_approved) {
            g_sys_status.mainState = STATE_RUNNING;
        }
        return;
    }

    // 2. 가동 상태 (RUNNING)
    if (g_sys_status.mainState == STATE_RUNNING) {

        // --- Step 2 & 3: 로봇 분류 및 AGV 연동 (Sort Part) ---
        switch (g_sys_status.sortState) {
            case SORT_IDLE:
                // [정의 반영] IDLE은 정지 상태
                g_sys_status.speed_main_convey = 0;
                g_sys_status.speed_sort_convey = 0;

                // [흐름도 4번] AGV 도착 신호 수신 시 가동 시작
                if (g_sys_status.rx_agv_sort_arrived) {
                    g_sys_status.sortState = SORT_RUNNING;
                }
                break;

            case SORT_RUNNING:
                // 서버에서 받은 속도로 가동 (task_system에서 반영됨)

                // [흐름도 2, 3번] 가동 중 로봇 Area 센서 감지 시 정지 및 작업
                if (g_sys_status.sensor_robot_area) {
                    CameraResult_t item = visionQ_pop();
                    if (item == ITEM_LARGE) {
                        g_sys_status.sortState = SORT_ROBOT_WORK;
                    }
                }

                // [흐름도 4번] AGV 출발 신호 수신 시 다시 IDLE(정지)로 복귀
                if (g_sys_status.rx_agv_sort_departed) {
                    g_sys_status.sortState = SORT_IDLE;
                }
                break;

            case SORT_ROBOT_WORK:
                // 로봇 작업 중 컨베이어 정지
                g_sys_status.speed_main_convey = 0;
                g_sys_status.speed_sort_convey = 0;

                g_sys_status.is_robot_work = 1;
                HAL_GPIO_WritePin(PIN_ROBOT_WORK, GPIO_PIN_SET);

                // 로봇 작업 완료 신호 시 다시 가동(RUNNING)
                if (g_sys_status.sensor_robot_done) {
                    g_sys_status.is_robot_work = 0;
                    HAL_GPIO_WritePin(PIN_ROBOT_WORK, GPIO_PIN_RESET);
                    g_sys_status.sortState = SORT_RUNNING;
                }
                break;
        }

        // --- Step 5 & 6: 적재 공정 및 리니어 제어 (Load Part) ---
        switch (g_sys_status.loadState) {
            case LOAD_IDLE:
                // [흐름도 5번] AGV 도착 시 적재 컨베이어 가동
                if (g_sys_status.rx_agv_load_arrived) {
                    // 서버 속도 유지 (모터4 가동)
                }

                // [흐름도 6번] AGV 출발 이후 리니어 이동 및 적재 판단
                if (g_sys_status.rx_agv_load_departed) {
                    g_sys_status.speed_load_convey = 0; // 이동 전 정지
                    uint8_t target = BSP_Sensor_GetEmptyRackFloor(); // 빈 랙 탐색

                    if (target > 0) {
                        g_sys_status.target_floor = target;
                        g_sys_status.loadState = LOAD_LIFT_MOVE;
                    }
                }
                break;

            case LOAD_LIFT_MOVE:
                // 스텝 모터를 이용해 목표 층으로 이동
                if (!g_sys_status.is_lift_busy) {
                    BSP_Stepper_MoveToFloor(g_sys_status.target_floor);
                    if (g_sys_status.current_step_pos == g_sys_status.target_step_pos) {
                        g_sys_status.loadState = LOAD_RACK_INSERT;
                        g_sys_status.state_timer = HAL_GetTick(); // 투입 시간 측정 시작
                    }
                }
                break;

            case LOAD_RACK_INSERT:
                // [흐름도 6번] 해당 층에서 컨베이어 재가동하여 랙에 투입
                g_sys_status.speed_load_convey = 50;

                // 2초(일정 시간) 경과 후 중지 및 1층 복귀
                if (HAL_GetTick() - g_sys_status.state_timer >= 2000) {
                    g_sys_status.speed_load_convey = 0;
                    BSP_Stepper_MoveToFloor(1); // 원점(1층)으로 복귀
                    g_sys_status.loadState = LOAD_IDLE;
                }
                break;
        }
    }
}
