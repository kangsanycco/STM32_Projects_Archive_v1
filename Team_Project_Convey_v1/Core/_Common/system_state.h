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
	STATE_BOOT = 0,						// 전원 ON 직후 초기화 대기
    STATE_IDLE,             			// 전원 ON 후 UART 가동 승인 대기 (전체 컨베이어 OFF)
    STATE_MAIN_CONVEY_RUNNING,          // 가동 승인 후 메인/분류 컨베이어 구동 중,
    STATE_ERROR_STOP,         		    // 시스템 에러 (lastError 참조)
    STATE_EMERGENCY,        		    // 비상 정지
	STATE_STOP							// 일반 정지
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
	STATE_LIFT_INITIALIZING,			// 전원이 켜지면, 리프트가 원점을 잡는 과정
    STATE_LIFT_MOVING,        		    // 목표 층으로 리니어 모터 이동 중
    STATE_RACK_INSERTING,      		    // 목표 층 도착 후 컨베이어 가동, 랙에 투입
    STATE_LIFT_RETURNING,      		    // 1층으로 복귀 중
} LiftRackState_t;

/**
 * @brief 카메라 판독 데이터
 */
typedef enum {
    ITEM_NONE = 0,
    ITEM_LARGE                  // 대분류 (로봇 분류 대상)
} CameraResult_t;

/**
 * @brief 에러 데이터
 */
typedef enum {
    ERR_NONE = 0,                       // 정상 (에러 없음)

    // --- 100번대: 하드웨어 보호 (즉시 중단 필요) ---
    // 근접 센서(PA0, PA1)가 범위를 벗어났을 때 즉시 감지
    ERR_LIFT_OVERRUN_1F = 101,          	// 1층 범위 초과 (PA0 감지, 더 내려가면 충돌 위험)
    ERR_LIFT_OVERRUN_2F = 102,         	    // 2층 범위 초과 (PA1 감지, 더 올라가면 충돌 위험)
    ERR_LIFT_HOMING_FAIL = 103,         	// 초기화 시 1층 센서를 찾지 못함 (구동 불능)
	ERR_EMERGENCY_STOP_BUTTON = 104,    	// 비상 정지(사용자가 중단)

	// --- 200번대: 공정이 멈춰서 확인이 필요한 상황 (공정 지연) ---
    ERR_ROBOT_DONE_TIMEOUT = 201,       	// 로봇 작업이 끝났다는 신호(PC6)를 주지 않음
    ERR_AGV_SIGNAL_TIMEOUT = 202,      		// AGV 도착/완료 신호 수신 지연 (AGV 확인 필요)
    ERR_ROBOT_PICK_FAILURE = 203,     				// 적외선 센서에 물체가 계속 감지됨 (로봇 오작동)
    ERR_LIFT_TARGET_POSITION_FAIL = 204,    // 리프트가 목표 스텝(위치)에 도달하지 못함 (탈조, 걸림 의심)

	// --- 300번대: 통신이 안 되어 제어가 불가능한 상황 (연결 불량) ---
	ERR_UART2_DISCONNECTED = 301,  // PC 서버(OPC-UA 연동)와 연결 끊김
	ERR_I2C1_DISCONNECTED = 302,      	    // 서보 드라이버(PCA9685) I2C 통신 두절

    ERR_SYSTEM_UNKNOWN = 999            	// 정의되지 않은 기타 에러
} ErrorCode_t;

/**
 * @brief 전체 시스템 상태 관리 장부
 */
typedef struct {
    // 1. 시스템 제어 및 모니터링
	// switch 에 넣기 위해 존재함 [ex. switch(currentState)]
	// ProcessState_t 는 데이터의 형식(타입) 이고, currentState 는 변수이기 때문이다
	// Enum은 진짜 이름, struct 는 보관함이다.

	MainControlState_t mainState;	 	 // [결정] 현재 메인 공정 단계
	RobotAgvState_t    robotagvState;	 // [결정] 로봇 및 AGV 단계
	LiftRackState_t	   liftrackState;	 // [결정] 리프트 및 랙 단계
    CameraResult_t     scanResult;       // [판단] 카메라 판독 결과 (무엇인가?)
    ErrorCode_t        lastError;		 // [경고] 마지막 에러 코드 (어디가 고장인가?)

    // 2. 카메라 데이터
    int is_scan_done;     // [추가] 판독이 완료되었는가? (0:대기/처리완료, 1:새로운데이터도착)

    // 2. 물리 센서 실시간 상태 (config.h와 1:1 매칭)
    int sensor_robot_area;           // PA10: 로봇 구역 물체 감지 (IR)
    int sensor_robot_done;           // PC6: 로봇 동작 완료 신호 (IN)
    int sensor_lift_level_1;         // PA0: 리니어 1층 범위 초과 에러 (근접)
    int sensor_lift_level_2;         // PA1: 리니어 2층 범위 초과 에러 (근접)
    int sensor_rack_full1;           // PA4: 1층 랙 물품 유무 (IR)
    int sensor_rack_full2;           // PA5: 2층 랙 물품 유무 (IR)

    // 3. 소프트웨어 플래그 (UART2 수신 데이터 기반)
    int soft_system_approved;     // UART 가동 승인 여부
    int soft_agv_sort_ready;      // 분류 AGV 도착 완료 신호
    int soft_agv_sort_full;       // 분류 AGV 적재 완료 (가득 참)
    int soft_agv_load_ready;      // 적재 AGV 도착 완료 신호
    int soft_agv_load_empty;      // 적재 AGV 하차 완료 (비었음)

    // 4. 구동부 출력 상태 (출력 - PCA9685 & STEP & ROBOT_WORK)
    // 하드웨어에 명령을 내린 후 "현재 상태"를 기억해야 논리적 충돌이 안 생깁니다.
    int speed_main_convey;      // 메인 컨베이어 모터1, 모터2 PWM 값
    int speed_sort_convey;      // 분류 컨베이어 모터3 PWM 값
    int speed_load_convey;      // 적재 컨베이어 모터4 PWM 값

    int is_robot_work;    // 로봇에게 시작하라는 명령, 출력(0:OFF, 1:ON)

    int is_lift_homed;         // 원점 복귀 완료 여부. 재부팅 후 리니어 모터가 중간에 위치하는 경우, 강제로 밑으로 내려가 원점(1층)을 찍게 만듦 (0:미완료/이동불가, 1:완료/이동가능)
    int32_t lift_current_step; // 리니어 모터 현재 위치 (펄스 카운트 값)

    int8_t  lift_dir_state;    // PIN_STEP_LIFT_DIRECTION (PB13) 현재 방향 (0:Up, 1:Down, 여러 상태로 확장될 가능성이 있기 때문에 이름으로 is를 사용하지 않음)
    int     is_lift_busy;      // 리니어 모터 구동 중 여부 (Step 생성 중인가?)
    int     target_lift_floor; // 목표 층 (1 or 2)
    int32_t target_step_position;   // 목표 층의 스텝 좌표

    // 5. 통계 데이터 (임시)
    uint32_t totalProcessed;
} SystemStatus_t;

extern SystemStatus_t g_sys_status;
// #include 의 경우는 데이터를 참조만 하고 메모리를 공유하지 않는다 (참조 파일의 데이터를 바꿔도 원본은 바뀌지 않음)
// extern 의 경우는 데이터의 메모리를 공유한다 (참조 파일의 데이터를 바꾸면 원본도 바뀜)
// app_init.c 에 위치

#endif /* COMMON_SYSTEM_STATE_H_ */
