/*
 * error_code.h
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#ifndef COMMON_ERROR_CODE_H_
#define COMMON_ERROR_CODE_H_

/**
 * @brief 시스템 에러 코드 정의
 * 모든 에러는 0이 아닌 양수로 정의하며, 0은 '정상(OK)'을 의미합니다.
 */
typedef enum {
    ERR_NONE = 0,               // 정상 (에러 없음)

    // 1. 센서/통신 관련 에러 (상황 데이터 오류)
    ERR_SENSOR_TIMEOUT,         // 센서 반응 시간 초과
    ERR_UART_COMM_FAIL,         // 카메라(PC) 통신 끊김
    ERR_I2C_MOTOR_FAIL,         // 모터 드라이버(PCA9685) 응답 없음

    // 2. 공정 흐름 관련 에러 (진행 과정 오류)
    ERR_ITEM_JAM,               // 컨베이어에 물체가 걸림 (센서가 계속 눌려 있음)
    ERR_EMERGENCY_PRESSED,      // 비상 정지 버튼 눌림
	ERR_ROBOT_WORK_TIMEOUT,     // 로봇이 작업을 끝내지 않음
	ERR_AGV_HANDSHAKE_FAIL,		// AGV 도착 센서(PA8, PB5) 응답 지연
    ERR_SORT_FULL_OVERFLOW,     // 분류함이 꽉 찼는데 물체가 계속 들어옴
    ERR_LOAD_EMPTY_TIMEOUT,     // 적재 라인이 비워져야 하는데 정해진 시간 내에 안 비워짐

    // 3. 하드웨어 보호
    ERR_MOTOR_OVERLOAD,         // 모터 과부하 감지 (모터를 돌렸을 때, 예상되는 시간 안에 물체가 도착하지 않음)
    ERR_SYSTEM_UNKNOWN          // 알 수 없는 치명적 오류
} ErrorCode_t;

#endif /* COMMON_ERROR_CODE_H_ */
