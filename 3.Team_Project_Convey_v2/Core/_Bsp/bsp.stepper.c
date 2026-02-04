/*
 * bsp.stepper.c
 *
 *  Created on: Jan 21, 2026
 *      Author: kangs
 */

#include "config.h"
#include "system_state.h"
#include "function.h"

// [설정] 1층에서 2층까지 이동하는 데 필요한 총 걸음 수 (데모용 임시값)
#define STEPS_PER_FLOOR    5000  // 1층에서 2층까지 필요한 스텝 수
#define HOMING_TIMEOUT	   12000  // 홈 복귀 시 최대 허용 스텝 (넘길 시 에러)


/**
 * @brief 스텝 모터 활성화/비활성화
 * @param enable 1: 구동 가능, 0: 모터 힘 풀림 (긴급 정지용)
 */
void BSP_Stepper_SetEnable(uint8_t enable) {
    // TB6600은 보통 Low일 때 활성화인 경우가 많으나, 설계에 따라 High/Low 조정
    HAL_GPIO_WritePin(PIN_STEP_LIFT_ENABLE, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_sys_status.is_step_enable = enable;
    // [함수 요약]
    // 1. BSP_Stepper_SetEnable 함수 실행
    // 2. PIN_STEP_LIFT_ENABLE 핀으로 가서, 별도로 정한 enable 값에 따라 전기를 주거나 끊어라 (만약 1이면 SET, 0이면 RESET)
    // 3. 그 때 enable 의 값을 is_step_enable(스텝 모터 활성화) 에 저장한다
}

/**
 * @brief 펄스 1개 생성 (한 칸 이동)
 */
static void BSP_Stepper_SingleStep(void) {
	// 1. 핀 상태를 반전 (High -> Low 혹은 Low -> High)
	HAL_GPIO_TogglePin(PIN_STEP_LIFT_PULSE);

	// 2. 핀이 SET(High)일 때만 실제 스텝 카운트를 수행
    if (HAL_GPIO_ReadPin(PIN_STEP_LIFT_PULSE) == GPIO_PIN_SET) {
    	if (g_sys_status.liftDirection == LIFT_DIR_UP) g_sys_status.current_step_pos++;
    	else if (g_sys_status.liftDirection == LIFT_DIR_DOWN) g_sys_status.current_step_pos--;
    }
    // [함수 요약]
    // 1. 짧은 시간에 핀을 토글하여 온오프
    // 2. 만약 리프트의 방향이 윗쪽이라면, 현재 리프트의 실제 스텝(높이) 상황 +1
    // 3. 만약 리프트의 방향이 아랫쪽이라면, 현재 리프트의 실제 스텝(높이) 상황 -1
}



/**
 * @brief 리프트 원점 복귀 (Homing)
 * @note PA0 센서가 감지될 때까지 아래로 이동
 */
void BSP_Stepper_Home(void) {
	uint32_t safety_counter = 0; // 무한루프 방지용

    g_sys_status.mainState = STATE_BOOT;
    g_sys_status.liftDirection = LIFT_DIR_DOWN;   // 상태값
    HAL_GPIO_WritePin(PIN_STEP_LIFT_DIRECTION, GPIO_PIN_RESET); // 핀 설정. 하강으로 방향만 정해준다, PIN_STEP_LIFT_DIRECTION의 drv 생략 (출력)

    BSP_Stepper_SetEnable(1);

    // 1층 센서(PA0)가 인식될 때까지 무한 반복 (안전을 위해 최대 스텝 제한 권장)
    while (HAL_GPIO_ReadPin(PIN_SENSOR_LIFT_1F) == GPIO_PIN_SET) {
        BSP_Stepper_SingleStep();
        HAL_Delay(1); // 홈 복귀는 안전하게 천천히
        // 리프트를 제외하고 다른 공정은 시작되기 전이므로, CPU에 집중을 하여도 된다

        if (++safety_counter > HOMING_TIMEOUT) {	// while 내를 반복하면서 1씩 추가되는 safety_counter
            g_sys_status.lastError = ERR_LIFT_HOMING_FAIL;
            BSP_Stepper_SetEnable(0); // 위험하니까 모터 힘 풀기
            return;
        }
    }

    // 도착 후 초기화
    g_sys_status.current_step_pos = 0;
    g_sys_status.lift_current_floor = 1;
    g_sys_status.is_lift_homed = 1;
    g_sys_status.liftDirection = LIFT_DIR_STOP;

    // [함수 요약]
    // 1. 부팅 시, BSP_Stepper_Home 함수가 실행되며, 메인 공정 상태가 STATE_BOOT 로 변경된다
    // 2. 리프트의 방향을 아래로 향한다고 상태값과 핀(출력)을 설정한다
    // 3. enable = 1인 BSP_Stepper_SetEnable 함수를 실행하며, is_step_enable(스텝 모터 활성화) 상태로 바꾼다
    // 4. 1층의 근접센서가 RESET(작동)할 때까지 펄스를 준다
    // 5. 1층의 근섭 센서 예상 범위를 넘었는데도(safety_counter > 12000)작동하고 있다면, 에러처리하며 리프트 작동을 중단한다
    // 6. 1층에 도착하면, 값을 초기화한다.
}

/**
 * @brief 특정 층으로 이동
 */
void BSP_Stepper_MoveToFloor(uint8_t floor) {
    if (!g_sys_status.is_lift_homed) return; // 초기화 이후, 영점 안 잡혔으면 거부
    if (g_sys_status.lift_current_floor == floor) return; // 이미 목표 층이면 거부

    g_sys_status.target_floor = floor;
    g_sys_status.is_lift_busy = 1;

    // 방향 설정
    if (floor == 2) {
        g_sys_status.liftDirection = LIFT_DIR_UP;	// 상태 설정
        HAL_GPIO_WritePin(PIN_STEP_LIFT_DIRECTION, GPIO_PIN_SET);	// 핀 설정
        g_sys_status.target_step_pos = STEPS_PER_FLOOR;	   // 목표 층 스텝 (1층: 5000)
    } else {
        g_sys_status.liftDirection = LIFT_DIR_DOWN;	  // 상태 설정
        HAL_GPIO_WritePin(PIN_STEP_LIFT_DIRECTION, GPIO_PIN_RESET);	  // 핀 설정
        g_sys_status.target_step_pos = 0;	// 목표 층 스텝 (2층: 0)
    }

    // 인터럽트(IT) 시작
    HAL_TIM_Base_Start_IT(&htim2);

    // [함수 요약]
    // 1. BSP_Stepper_MoveToFloor : 리프트(스텝모터)가 목표층으로 이동 할 때 켜지는 함수
    // 2. 반환: 만약 영점이 맞지 않거나, 이미 목표층이면 그대로 반환
    // 3. 만약 2층이 목표라면, 상태와 핀을 윗방향으로. 1층이 목표라면 아랫방향으로 정하고, 목표 층의 스텝 위치를 정한다
    // 4. 인터럽트 신호를 보낸다
}


/**
 * @brief 타이머 인터럽트 완료 콜백 (목적지 도착 지점)
 * 속도가 느리다면 htim2의 Prescaler나 Period 값을 조정해볼 것
 */
void BSP_Stepper_Interrupt_Handler(void) {
    // 현재 위치가 목표 위치와 다르면 계속 한 스텝씩 이동
    if (g_sys_status.current_step_pos != g_sys_status.target_step_pos) {
        BSP_Stepper_SingleStep();
    }
    else {
        // 목표 도달 시 타이머 중단 및 상태 업데이트
        HAL_TIM_Base_Stop_IT(&htim2);

        // 정지 시 핀을 Low로 확실히 고정
        HAL_GPIO_WritePin(PIN_STEP_LIFT_PULSE, GPIO_PIN_RESET);

        g_sys_status.lift_current_floor = g_sys_status.target_floor;
        g_sys_status.is_lift_busy = 0;
        g_sys_status.liftDirection = LIFT_DIR_STOP;
    }
    // [함수 요약]
    // 1. HAL_TIM_Base_Start_IT 에 의해 목표치만큼
    // 펄스를 쏘면 인터럽트 콜백 함수(main.c 의 HAL_TIM_PeriodElapsedCallback)가 실행된다
    // 2. 만약 현재 위치가 목표층의 위치오 다르다면, 계속해서 한 칸씩 증가한다.
    // 3. 만약 도착했다면, 중단 신호를 보낸다

}



