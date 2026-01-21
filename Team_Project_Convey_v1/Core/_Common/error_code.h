/*
 * error_code.h
 *
 * Created on: Jan 17, 2026
 * Author: kangs
 */

#ifndef COMMON_ERROR_CODE_H_
#define COMMON_ERROR_CODE_H_

/**
 * @brief 시스템 에러 코드 정의
 * - 0: 정상(ERR_NONE)
 * - 100번대: 하드웨어 치명적 오류 (EXTI 센서 및 통신)
 * - 200번대: 공정 타임아웃 및 논리 오류 (시스템 흐름)
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

#endif /* COMMON_ERROR_CODE_H_ */
