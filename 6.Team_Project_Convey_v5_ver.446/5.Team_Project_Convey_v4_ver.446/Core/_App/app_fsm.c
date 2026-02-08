/*
 * app.fsm.c
 *
 * Created on: Jan 17, 2026
 * Author: kangs
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
    }
}

CameraResult_t visionQ_pop(void) {	 // 값 빼기, visionQ_pop 함수가 끝나면 CameraResult_t 규격으로 값을 반환한다라고 선언
    if (g_sys_status.visionQ.count == 0) return ITEM_NONE;
    CameraResult_t item = g_sys_status.visionQ.slot[g_sys_status.visionQ.head];
    g_sys_status.visionQ.head = (g_sys_status.visionQ.head + 1) % VISION_QUEUE_SIZE;
    g_sys_status.visionQ.count--;
    return item;
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
    g_sys_status.is_robot_work = 1;

    BSP_Robot_Start_Trigger();

    // 3. 로봇이 초기 세팅(영점 등)을 할 시간을 줌
    HAL_Delay(6000);

    // 4. 완료 표시
    g_sys_status.is_robot_work = 0;
}

/**
 * @brief 전체 공정 시나리오 실행 (FSM)
 */
void APP_FSM_Execute(void) {
	// 0. 서버 신호(mainState)가 STATE_BOOT인지 확인
	if (g_sys_status.mainState == STATE_BOOT) {
		if (g_sys_status.is_init_done == 0) {
		    APP_FSM_Init();
		    g_sys_status.is_init_done = 1; // 초기화 완료 표시
		}
	    return;
	}

	// STATE_RUNNING 은 uart 통신을 별도로 제어된다.

    // 1. 정지 / 긴급 정지 체크 (장비 정지, 로봇 잔여 작업)
    if (g_sys_status.mainState == STATE_EMERGENCY || g_sys_status.mainState == STATE_STOP) {
        g_sys_status.speed_main_convey = 0;
        g_sys_status.speed_sort_convey = 0;

        if (g_sys_status.is_robot_work == 1) {
            g_sys_status.mainState = STATE_ROBOT_RESET_WORK;
        } else {
            g_sys_status.mainState = STATE_IDLE;
                }
        return;
    }

    // 로봇은 긴급 정지 시에도 하던 작업은 끝내야 함
    if (g_sys_status.mainState == STATE_ROBOT_RESET_WORK) {
    	if (HAL_GetTick() - g_sys_status.robot_timer >= ROBOT_WORK_TIME_MS) {
    	    g_sys_status.is_robot_work = 0; // 작업 종료
    	    g_sys_status.mainState = STATE_IDLE; // 이제 완전히 대기상태로
    	}
    	return;
    }



    // 3. 대기 상태 (IDLE) - 가동 승인 대기
    if (g_sys_status.mainState == STATE_IDLE) return;

    // drv_uart.c 가 통신을 통해 데이터를 받으면 장부(g_sys_status.mainState)를 받아
    // STATE_RUNNING으로 바꾼다


    // 4. 가동 상태 (RUNNING)
    if (g_sys_status.mainState == STATE_RUNNING) {

        // --- Step 2 & 3: 로봇 분류 및 AGV 연동 (Sort Part) ---
        switch (g_sys_status.sortState) {
            case SORT_IDLE:
                if (g_sys_status.rx_agv_sort_arrived) {
                    g_sys_status.sortState = SORT_RUNNING;
                }
                break;

                // [함수 요약]
                // 1. SORT_IDLE 상태에 진입하면, 메인&분류 컨베이어 속도를 0으로 고정

            case SORT_RUNNING:
                if (!g_sys_status.sensor_robot_area) {
                    CameraResult_t item = visionQ_pop();
                    if (item == ITEM_LARGE) {
                    	BSP_Robot_Start_Trigger();
                    	g_sys_status.is_robot_work = 1;
                    	g_sys_status.robot_timer = HAL_GetTick();
                    	g_sys_status.sortState = SORT_ROBOT_WORK;
                    }
                }
                if (g_sys_status.rx_agv_sort_departed) {
                    g_sys_status.sortState = SORT_IDLE;
                }
                break;


            case SORT_ROBOT_WORK:
            	if (HAL_GetTick() - g_sys_status.robot_timer >= ROBOT_WORK_TIME_MS) {
            	    g_sys_status.is_robot_work = 0;
            	    g_sys_status.sortState = SORT_RUNNING;
            	}
                break;
        }
    }
}
