/*
 * system_state.h
 *
 * Created on: Jan 17, 2026
 * Author: kangs
 */

#ifndef COMMON_SYSTEM_STATE_H_
#define COMMON_SYSTEM_STATE_H_

#include "config.h"
#include "error_code.h"

/**
 * [그룹 A] 메인 제어 및 시스템 상태
 */
typedef enum {
    STATE_IDLE = 0,             		// 전원 ON 후 UART 가동 승인 대기 (전체 컨베이어 OFF)
    STATE_MAIN_CONVEY_RUNNING,          // 가동 승인 후 메인/분류 컨베이어 구동 중,
    STATE_ERROR_STOP,         		    // 시스템 에러 (lastError 참조)
    STATE_EMERGENCY          		    // 비상 정지
} MainControlState_t;

/**
 * [그룹 B] 로봇 및 AGV 분류 공정
 */
typedef enum {
    STATE_CONVEY_PAUSED_ROBOT_BUSY,     // 컨베이어 중지, 로봇 작동 중
    STATE_CONVEY_RUNNING_ROBOT_IDLE,    // 컨베이어 가동, 로봇 중지
	STATE_SORT_AGV_WAIT,				// AGV 분류 파트 미도착
    STATE_SORT_AGV_PARKING,          	// AGV 분류 파트에 주차, 대분류 품목 투입
	STATE_LOAD_AGV_WAIT,				// AGV 적재 파트 미도착
    STATE_LOAD_AGV_PARKING,          	// AGV 적재 파트에 주차, 대분류 물품 수령 중
} RobotAgvState_t;

/**
 * [그룹 C] 리프트 및 랙 적재 공정
 */
typedef enum {
    STATE_LIFT_MOVING,        		    // 목표 층으로 리니어 모터 이동 중
    STATE_RACK_INSERTING,      		    // 목표 층 도착 후 컨베이어 가동, 랙에 투입
    STATE_LIFT_RETURNING,      		    // 1층으로 복귀 중
} LiftRackState_t;

/**
 * @brief 카메라 판독 데이터
 */
typedef enum {
    ITEM_NONE = 0,
    ITEM_SMALL,                 // 소분류 (통과)
    ITEM_LARGE                  // 대분류 (로봇 분류 대상)
} CameraResult_t;

/**
 * @brief 전체 시스템 상태 관리 장부
 */
typedef struct {
    // 1. 시스템 제어 및 모니터링
	// switch 에 넣기 위해 존재함 [ex. switch(currentState)]
	// ProcessState_t 는 데이터의 형식(타입) 이고, currentState 는 변수이기 때문이다
	// Enum은 진짜 이름, struct 는 별명이다.
    ProcessState_t currentState;	 // [결정] 현재 공정 단계 (어디쯤 왔나?)
    CameraResult_t scanResult;       // [판단] 카메라 판독 결과 (무엇인가?)
    ErrorCode_t    lastError;		 // [경고] 마지막 에러 코드 (어디가 고장인가?)

    // 2. 물리 센서 실시간 상태 (config.h와 1:1 매칭)
    int sensor_robot_area;           // PA10: 로봇 구역 물체 감지 (IR)
    int sensor_robot_done;           // PB2: 로봇 동작 완료 신호 (IN)
    int sensor_lift_lvl1;            // PA0: 리니어 1층 도착 (근접)
    int sensor_lift_lvl2;            // PA1: 리니어 2층 도착 (근접)
    int sensor_rack_full1;           // PA4: 1층 랙 물품 유무 (IR)
    int sensor_rack_full2;           // PA5: 2층 랙 물품 유무 (IR)

    // 3. 소프트웨어 플래그 (UART2 수신 데이터 기반)
    int soft_system_approved;     // UART 가동 승인 여부
    int soft_agv_sort_ready;      // 분류 AGV 도착 완료 신호
    int soft_agv_sort_full;       // 분류 AGV 적재 완료 (가득 참)
    int soft_agv_load_ready;      // 적재 AGV 도착 완료 신호
    int soft_agv_load_empty;      // 적재 AGV 하차 완료 (비었음)

    // 4. 구동부 상태 확인용
    int is_lift_busy;             // 리니어 모터 구동 중 여부
    int target_lift_floor;        // 리니어 모터 목표 층 (1 or 2)

    // 5. 통계 데이터
    uint32_t totalProcessed;
} SystemStatus_t;

extern SystemStatus_t g_sys_status;

#endif /* COMMON_SYSTEM_STATE_H_ */
