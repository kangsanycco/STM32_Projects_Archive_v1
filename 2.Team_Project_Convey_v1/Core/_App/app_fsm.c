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
    g_sys_status.is_lift_homed = 0; // 영점을 잡기 시작 전에 초기화

    // 카메라 비전 큐 초기화
    g_sys_status.visionQ.count = 0;
    g_sys_status.visionQ.head = 0;
    g_sys_status.visionQ.tail = 0;

    // 리니어 모터 영점 잡기 (bsp_stepper.c 호출)
    BSP_Stepper_Home();

    if (g_sys_status.is_lift_homed) {
        g_sys_status.mainState = STATE_RUNNING;
    }
}

/**
 * @brief 전체 공정 시나리오 실행 (FSM)
 */
void APP_FSM_Execute(void) {
	// 0. 서버 신호(mainState)가 STATE_BOOT인지 확인
	if (g_sys_status.mainState == STATE_BOOT) {

	    // 아직 영점을 안 잡은 상태라면 (최초 1회 혹은 재부팅 신호 시)
	    APP_FSM_Init(); // 여기서 비로소 리프트 영점을 잡고 IDLE로 바꿉니다.
	    return;

	}

    // 1. 긴급 정지 체크 (최우선순위, 모든 장비 즉시 정지)
    if (g_sys_status.mainState == STATE_EMERGENCY) {
        g_sys_status.speed_main_convey = 0;
        g_sys_status.speed_sort_convey = 0;
        g_sys_status.speed_load_convey = 0;
        BSP_Stepper_SetEnable(0); // 스텝 모터 비활성화

        // 로봇 잔여 작업 체크
        if (g_sys_status.is_robot_work == 1) {
                g_sys_status.mainState = STATE_EMERGENCY_ROBOT;
            } else if (!g_sys_status.is_lift_busy) { // 로봇 안하면 리프트 확인 후 IDLE
                    g_sys_status.mainState = STATE_IDLE;
            }
            return;
        }

        // 로봇은 긴급 정지 시에도 하던 작업은 끝내야 함
    	if (g_sys_status.mainState == STATE_EMERGENCY_ROBOT) {
    		if (!g_sys_status.sensor_robot_done) { // 로봇이 완료 신호를 주면
    		    HAL_GPIO_WritePin(PIN_ROBOT_WORK, GPIO_PIN_RESET);
    		    g_sys_status.is_robot_work = 0;
    		    g_sys_status.mainState = STATE_IDLE; // 다시 완전 비상태로 복귀
    		}
    		return;

        // [함수 요약]
        // 1. EMERGENCY 버튼을 누르면 속도를 0으로 바꾸고 리니어 모터를 가동 가능하게 한다
        // 2. 만약 로봇이 작동하고, 로봇으로부터 작업 완료 신호를 받았다면,
        // 로봇이 쉬고 있다는 신호와 함께 핀의 입력을 끈다

    }
    // 2. 정지 상태 (STATE_STOP) 처리: 하던 작업 마무리 후 IDLE로 가기
    if (g_sys_status.mainState == STATE_STOP) {
    	// 유입을 막기 위해 컨베이어는 정지
    	g_sys_status.speed_main_convey = 0;
        g_sys_status.speed_sort_convey = 0;
    	g_sys_status.speed_load_convey = 0;

    	// STOP 시에도 로봇이 작업 중이면 마무리 단계로 보냄
    	if (g_sys_status.is_robot_work == 1) {
    	    g_sys_status.mainState = STATE_EMERGENCY_ROBOT;
    	}
    	else if (!g_sys_status.is_lift_busy) {
    	    g_sys_status.mainState = STATE_IDLE;
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
                // IDLE은 정지 상태
                g_sys_status.speed_main_convey = 0;
                g_sys_status.speed_sort_convey = 0;

                // AGV 도착 신호 수신 시 가동 시작
                if (g_sys_status.rx_agv_sort_arrived) {
                    g_sys_status.sortState = SORT_RUNNING;
                }
                break;

                // [함수 요약]
                // 1. SORT_IDLE 상태에 진입하면, 메인&분류 컨베이어 속도를 0으로 고정

            case SORT_RUNNING:
                // 서버에서 받은 속도로 가동 (task_system에서 반영됨)

                // 가동 중 로봇 Area 센서 감지 시 정지 및 작업
                if (!g_sys_status.sensor_robot_area) {
                    CameraResult_t item = visionQ_pop();
                    if (item == ITEM_LARGE) {
                        g_sys_status.sortState = SORT_ROBOT_WORK;
                    }
                }

                //  AGV 출발 신호 수신 시 다시 IDLE(정지)로 복귀
                if (g_sys_status.rx_agv_sort_departed) {
                    g_sys_status.sortState = SORT_IDLE;
                }
                break;

                // [함수 요약]
                // 1. SOTR_RUNNING, 분류 컨베이어 가동 상태라면
                // 2. 로봇 AREA 신호가 잡히면, visionQ에서 데이터 하나를 뽑아와 저장한다
                // 3. 저장한 데이터가 큰 상자라면, SORT_ROBOT_WORK 상태가 된다
                // 4. 만약 분류쪽 AGV가 출발헀다면, SORT_IDLE 상태가 된다

            case SORT_ROBOT_WORK:
                // 로봇 작업 중 컨베이어 정지
                g_sys_status.speed_main_convey = 0;
                g_sys_status.speed_sort_convey = 0;

                g_sys_status.is_robot_work = 1;
                HAL_GPIO_WritePin(PIN_ROBOT_WORK, GPIO_PIN_SET);

                // 로봇 작업 완료 신호 시 다시 가동(RUNNING)
                if (!g_sys_status.sensor_robot_done) {
                    g_sys_status.is_robot_work = 0;
                    HAL_GPIO_WritePin(PIN_ROBOT_WORK, GPIO_PIN_RESET);
                    g_sys_status.sortState = SORT_RUNNING;
                }
                break;

                // [함수 요약]
                // 1. 센서 신호를 받아 SORT_ROBOT_WORK 상태가 되면, 메인&분류 컨베이어 속도를 0으로 바꿉니다
                // 2. 로봇이 작동 상태라는 신호와 핀 입력을 줍니다
                // 3. 만약 로봇의 작업이 전부 끝난다면, 로봇 작동 상태 신호와 핀 입력을 끕니다.
                // 4. 분류 상태를 SORT_RUNNING 상태로 바꿉니다

        }

        // --- Step 5 & 6: 적재 공정 및 리니어 제어 (Load Part) ---
        switch (g_sys_status.loadState) {
            case LOAD_IDLE:
            	g_sys_status.speed_load_convey = 0; // 평소엔 정지


                // AGV가 출발하면 즉시 멈추고 다음 단계로 이동한다
                if (g_sys_status.rx_agv_load_departed) {
                    g_sys_status.speed_load_convey = 0; // 이동 전 정지
                    uint8_t target = BSP_Sensor_GetEmptyRackFloor(); // 빈 랙 탐색

                    if (target > 0) {	// 랙에 물품이 다 차지 않았다면
                        g_sys_status.target_floor = target;
                        g_sys_status.loadState = LOAD_LIFT_MOVE;
                    }

                    // [함수 요약]
                    // 1. 적재 대기 상태일 때, 기본적으로는 멈춰 있는다.
                    // 2. (대기)AGV 위의 컨베이어가 돌아가게 되며, 기다린다
                    // 3. AGV가 출발하면 컨베이어를 멈추고, 빈 랙을 탐색한다
                    // 4. 만약 랙에 빈 공간이 있다면, 목표층을 정하고, 적재파트 상태를 LOAD_LIFT_MOVE(리프트가 움직이는) 것으로 바꾼다

                }
                break;

            case LOAD_LIFT_MOVE:
                // 아직 이동 시작 전이라면 (busy가 0일 때 시작)
                if (!g_sys_status.is_lift_busy && (g_sys_status.lift_current_floor != g_sys_status.target_floor)) {
                    BSP_Stepper_MoveToFloor(g_sys_status.target_floor);
                }

                // 이동이 끝났다면 (DMA 콜백에 의해 busy가 0이 되고 층이 업데이트됨)
                if (!g_sys_status.is_lift_busy && (g_sys_status.lift_current_floor == g_sys_status.target_floor)) {
                    g_sys_status.loadState = LOAD_RACK_INSERT;
                    g_sys_status.state_timer = HAL_GetTick();

                    // [함수 요약]
                    // 1. 리프트가 작동 중일 때
                    // 2. 리프트가 정지 상태라면, 목표층으로 리니어 이동
                    // 3. 현재 리니어 위치(높이) 와 목표 층의 위치(높이)가 같다면
                    // 4. 상태는 LOAD_RACK_INSERT(랙에 물품을 집어넣는다)가 되며, 투입 시간을 측정한다

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

                // [함수 요약]
                // 1. 현재 상태가 LOAD_RACK_INSERT(투입 중)이라면, 컨베이어의 속도를 50으로 바꾼다
                // 2. 투입 시작으로 부터 2000ms 가 지나면, 컨베이어 속도는 0으로 변경된다
                // 3. 리프트는 1층으로 이동하며, 도착 시 LOAD_IDLE(대기 중) 상태로 변경된다

        }
    }
}
