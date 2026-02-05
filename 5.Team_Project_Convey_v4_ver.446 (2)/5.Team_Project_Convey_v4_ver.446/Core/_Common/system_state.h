/*
 * system_state.h
 *
 * Created on: Jan 17, 2026
 * Author: kangs
 */

#ifndef COMMON_SYSTEM_STATE_H_
#define COMMON_SYSTEM_STATE_H_

#include "config.h"


/**
 * [그룹 A] 메인 제어 및 시스템 상태
 */
typedef enum {
	STATE_BOOT = 0,							// 전원 ON 직후와 초기화 작업
    STATE_IDLE = 1,             			// 전원 ON 후 UART 가동 승인 대기 (전체 컨베이어 OFF)
    STATE_RUNNING = 2,       	    		// 가동 승인 후 메인/분류 컨베이어 구동 중,
	STATE_STOP = 3,							// 일반 정지
	STATE_ROBOT_REST_WORK = 4,				// 정지 중 로봇 잔여 작업
    STATE_EMERGENCY = 5,        		  	// 비상 정지
} MainControlState_t;

/**
 * [그룹 B] 분류 공정 상세 상태 (모터 1, 2,
 * 3 및 로봇 제어)
 */
typedef enum {
    SORT_IDLE = 0,       // 메인&분류 컨베이어 멈춤 (AGV가 없거나, 로봇 작업이 없거나, 정지 명령 시)
    SORT_ROBOT_WORK = 1, // 로봇 작업 중 (컨베이어 일시 정지)
    SORT_RUNNING = 2     // AGV 도착 확인됨, 모든 컨베이어 가동 (상차 중)
} SortState_t;

/**
 * [그룹 C] 적재 공정 상세 상태 (모터 4 및 리니어 제어)
 */
typedef enum {
    LOAD_IDLE = 0,          // 1층 대기 (AGV 수령 중 포함)
    LOAD_LIFT_MOVE = 1,     // 리프트 작동 중, 목표 층(1층, 2층)으로 상승 또는 복귀 이동
    LOAD_RACK_INSERT = 2    // 해당 층 도착 후 랙에 물건 밀어넣기
} LoadState_t;

/**
 * [그룹 D] 리니어 이동 방향 (물리 핀 PB13 매칭용)
 */
typedef enum {
    LIFT_DIR_STOP = 0,
    LIFT_DIR_UP = 1,            // PB13 상태에 맞게 연동
    LIFT_DIR_DOWN = 2
} LiftDir_t;

/**
 * @brief 카메라 판독 데이터
 */
typedef enum {
    ITEM_NONE = 0,
    ITEM_LARGE = 1,                		    // 대분류 (로봇 분류 대상)
} CameraResult_t;


/**
 * @brief 에러 데이터
 */
typedef enum {
    ERR_NONE = 0,                       // 정상 (에러 없음)

    // --- 100번대: 하드웨어 보호 (즉시 중단 필요) ---
    // 근접 센서(PA0, PA1)가 범위를 벗어났을 때 즉시 감지
    ERR_LIFT_OVERRUN_1F = 11,          	// 1층 범위 초과 (PA0 감지, 더 내려가면 충돌 위험)
    ERR_LIFT_OVERRUN_2F = 12,         	// 2층 범위 초과 (PA1 감지, 더 올라가면 충돌 위험)
    ERR_LIFT_HOMING_FAIL = 13,         	// 초기화 시 1층 센서를 찾지 못함 (구동 불능)
	ERR_EMERGENCY_STOP_BUTTON = 14,    	// 비상 정지(사용자가 중단)

	// --- 200번대: 공정이 멈춰서 확인이 필요한 상황 (공정 지연) ---
    ERR_ROBOT_DONE_TIMEOUT = 21,       	// 로봇 작업이 끝났다는 신호(PC6)를 주지 않음
    ERR_AGV_SIGNAL_TIMEOUT = 22,      	// AGV 도착/완료 신호 수신 지연 (AGV 확인 필요)
    ERR_ROBOT_PICK_FAILURE = 23,     	// 적외선 센서에 물체가 계속 감지됨 (로봇 오작동)
    ERR_LIFT_TARGET_POSITION_FAIL = 24, // 리프트가 목표 스텝(위치)에 도달하지 못함 (탈조, 걸림 의심)

	// --- 300번대: 통신이 안 되어 제어가 불가능한 상황 (연결 불량) ---
	ERR_UART2_DISCONNECTED = 31,  		// PC 서버(OPC-UA 연동)와 연결 끊김
	ERR_I2C1_DISCONNECTED = 32,      	// 서보 드라이버(PCA9685) I2C 통신 두절

	ERR_SYSTEM_UNKNOWN = 99            	// 정의되지 않은 기타 에러
} ErrorCode_t;

typedef struct {
    CameraResult_t slot[VISION_QUEUE_SIZE];	// 창고 칸
    uint8_t head;	// 맨 앞줄, 데이터를 가장 먼저 꺼내는 곳 (출구)
    uint8_t tail;	// 맨 뒷줄, 데이터가 새로 들어와서 붙는 곳 (입구)
    uint8_t count;	// 현재 재고량
} VisionQueue_t;

/**
 * @brief 전체 시스템 상태 관리 장부
 */
typedef struct {
    // 1. 시스템 제어 및 모니터링 (모든 상태 Enum을 이곳에 집중)
    MainControlState_t mainState;      // 통합 시스템 상태
    SortState_t        sortState;      // 분류 공정 상태
    LoadState_t        loadState;      // 적재 공정 상태
    LiftDir_t          liftDirection;  // 리프트 이동 방향 상태 (0:정지, 1:상승, 2:하강)
    CameraResult_t     currentScan;    // 현재 스캔된 물체 타입
    ErrorCode_t        lastError;      // 시스템 에러 상태

    // 2. 카메라 데이터 (Queue 관리)
    VisionQueue_t      visionQ;        // 비전 데이터를 순서대로 저장하는 FIFO 큐
    uint8_t            is_scan_done;   // 비전 PC로부터 새로운 판독 데이터가 수신됨을 알림

    // 3. 물리 센서 실시간 상태 (PA/PC Input 핀 직접 매칭)
    uint8_t sensor_robot_area;             // PA10: 로봇 분류 구역 내 물체 감지 유무
    uint8_t signal_robot_done;             // 로봇으로부터 작업 완료 신호 수신 여부 (0: 로봇 완료 신호 대기 상태, 1: 로봇 작업 완료)
    uint8_t sensor_lift_1f;                // PA6: 리프트 1층(영점) 도달 확인 센서 (0: 센서 감지, 1: 비감지)
    uint8_t signal_lift_2f;                // 리프트 2층 도달 신호	(0: 도착 안함, 1: 2층 도착)
    uint8_t sensor_lift_overrun_2f;		   // PA7: 리프트 2층 범위 이탈
    uint8_t sensor_rack_full_1f;           // PA4: 1층 적재함 물품 존재 여부
    uint8_t sensor_rack_full_2f;           // PA5: 2층 적재함 물품 존재 여부

    // 4. 소프트웨어 플래그 (UART2 Rx 8바이트 패킷 데이터 기반)
    uint8_t rx_uart2_approved;       	   // 서버로부터의 가동 승인 신호 (Run/Stop)
    uint8_t rx_agv_sort_arrived;      	   // 분류부 AGV 도착 (상차 준비됨)
    uint8_t rx_agv_sort_departed;          // 분류부 AGV 출발 (상차 끝남)
    uint8_t rx_agv_load_arrived;      	   // 적재부 AGV 도착 (하차 준비됨)
    uint8_t rx_agv_load_departed;          // 적재부 AGV 출발 (하차 끝남)

    // 5. 구동부 출력 상태 (실제 하드웨어 제어 명령 값 기록)
    uint8_t speed_main_convey;  	       // 모터 1, 2 (메인 라인) PWM 속도 값
    uint8_t speed_sort_convey;             // 모터 3 (분류 라인) PWM 속도 값
    uint8_t speed_load_convey;     	       // 모터 4 (적재 라인) PWM 속도 값
    uint8_t is_robot_work;    			   // PC7: 로봇 가동 트리거 출력 상태 (0:OFF, 1:ON)
    uint8_t is_step_enable;				   // PB14: 스텝 모터 활성화(시동) (Emergency 차단용)

    // 6. 리프트(스텝 모터) 물리 정보
    uint8_t  is_lift_homed;          // 전원 투입 후 영점 완료 여부
    uint8_t  lift_current_floor;     // 리프트 현재 층수 (1 또는 2)
    uint8_t  target_floor;           // 이동해야 할 목표 층수
    uint8_t  is_lift_busy;           // 리프트 이동 여부(1: 이동 중, 0: 대기 중, is_step_enable 과 전원과 동작 여부에서 다름) (
    int32_t  current_step_pos;       // 현재 리프트의 실제 스텝(높이)상황
    int32_t  target_step_pos;        // 이동해야 할 최종 목표 스텝 상황

    // 7. [기타] 통계 & 타이머 & 로봇 통신 수신용 버퍼

    uint32_t   state_timer;            // 공정 내 지연 시간(Time-out)이나 대기 시간 카운트
    uint32_t   robot_timer;            // 로봇 분류 공정 전용 타이머
    uint32_t   totalProcessed;         // 가동 이후 총 처리된 물품 개수
} SystemStatus_t;

    extern SystemStatus_t g_sys_status;	  // main.c 에 실체 위치
// #include 의 경우는 데이터를 참조만 하고 메모리를 공유하지 않는다 (참조 파일의 데이터를 바꿔도 원본은 바뀌지 않음)
// extern 의 경우는 데이터의 메모리를 공유한다 (참조 파일의 데이터를 바꾸면 원본도 바뀜)
// app_init.c 에 위치

#endif /* COMMON_SYSTEM_STATE_H_ */
