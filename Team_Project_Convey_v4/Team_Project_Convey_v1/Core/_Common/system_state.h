/*
 * system_state.h
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#ifndef COMMON_SYSTEM_STATE_H_
#define COMMON_SYSTEM_STATE_H_

#include "config.h"
#include "error_code.h"

/**
 * @brief 공정 메인 상태 FSM(Finite State Machine) - 현재 어느 시점에 있는지를 나타냄
 */
typedef enum {
	// [그룹 1] 전체 모터 스위치
	STATE_ENTRY_MOTOR_IDLE = 0,		// PLC OFF: 시스템 전체 정지 및 대기
	STATE_ENTRY_MOTOR_READY,		// PLC ON: 가동 시작, 메인 컨베이어 구동 가능

	// [그룹 2] 메인 분류 라인 (모터 1, 모터2, 로봇)
    STATE_MAIN_MOTOR_RUNNING,		// 메인 컨베이어 모터1, 모터2 가동 중
	STATE_ROBOT_IDLE,				// 로봇 정지 중 (대분류 대상물 분류 상태)
    STATE_ROBOT_WORKING,			// 컨베이어 정지 후, 로봇 작동 중 (작동 완료 신호 대기)

    // [그룹 3] 분류 라인 상태 (모터3, AGV)
	STATE_SORT_MOTOR_RUNNING,		// 분류 컨베이어 모터3 가동 중
	STATE_SORT_MOTOR_IDLE,			// 분류 컨베이어 모터3 정지 중, AGV 도착하지 않음
	STATE_SORT_AGV_PARKING,			// 분류쪽 AGV 도착 후 주차
	STATE_SORT_AGV_FULL,			// AGV 가득 참, 배출 중단 및 AGV 출발 대기

	// [그룹 4] 적재 라인 상태 (모터4, AGV)
    STATE_LOAD_MOTOR_RUNNING,
	STATE_LOAD_MOTOR_IDLE,// 적재 컨베아어 모터4 가동 중
    STATE_LOAD_AGV_PARTKING,   		// 적재쪽 AGV 도착 상태
    STATE_LOAD_AGV_EMPTY,   		// 적재 완료 및 컨베이어 정지

	// [그룹 5] 예외 관리
	STATE_ERROR_STOP,				// 시스템 에러 발생 (lastError 확인 필요)
	STATE_EMERGENCY_STOP			// 비상 정지 버튼 눌림 (최우선 정지)

} ProcessState_t;

/**
 * @brief 카메라 판독 데이터 (로봇 구간 도착 전 선행 정보)
 */
typedef enum {
    ITEM_NOT_SCANNED = 0,        // 아직 카메라 구간을 지나지 않음
    ITEM_SMALL_CLASS,            // 소분류 (로봇 구간 비정지 통과 대상)
    ITEM_LARGE_CLASS             // 대분류 (로봇 구간 정지 및 AGV 배출 대상)
} CameraResult_t;

/**
 * @brief 전체 시스템 상태 관리 장부 (중앙 통제 구조체)- 진행 과정의 데이터
 */
typedef struct {
    // 1. 시스템 제어 상태
    ProcessState_t currentState;  // 현재 공정이 무슨 상태인지 나타내는 지표
    CameraResult_t pendingResult; // 카메라가 미리 보낸 데이터 (로봇 구간의 판단 근거)
    ErrorCode_t    lastError;     // 발생한 에러를 기록하는 칸

    // 2. 실시간 센서 상태 (0: 미감지, 1: 감지)
    int isPlcActive;              // PLC ON/OFF 상태
    int isItemInRobotArea;        // 로봇 구간 적외선 센서 (PA10)
    int isRobotDoneSignal;		  // 로봇으로부터 받은 완료 신호 (PB1)
    int isSortAgvReady;           // 분류쪽 AGV 도착 여부 (PA8)
    int isLoadAgvReady;           // 적재쪽 AGV 도착 여부 (PB5)

    // 3. 적재/비움 상태 (박제된 적외선 센서)
    int isAgvFull;               // 분류 파트 AGV 컨베이어 만재 여부 (PB10)
    int isAgvEmpty;              // 적재 파트 AGV 컨베이어 끝 비워짐 여부 (PB11)

    // 4. 로봇 제어 상태 (추가)
    int isRobotBusy;              // [추가] 현재 로봇이 가공 중인지 여부 (Flag)

    // 4. 통계 및 디버깅 데이터
    uint32_t totalProcessed;      // 총 처리 수량
} SystemStatus_t;

/**
 * @brief 전역 변수 공유 선언
 * app_init.c에서 실제 메모리 할당이 이루어집니다.
 */
extern SystemStatus_t g_sys_status;		// 시스템의 상태

#endif /* COMMON_SYSTEM_STATE_H_ */
