/*
 * app_event.h
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#ifndef APP_APP_EVENT_H_
#define APP_APP_EVENT_H_


/**
 * @brief 시스템에서 발생하는 모든 사건(Event) 목록
 */
typedef enum {
    EVENT_NONE = 0,             // 발생한 사건 없음

    // 1. 시스템 제어 관련 (PLC)
    EVENT_PLC_RUN,              // PLC로부터 가동 승인 신호가 들어옴
    EVENT_PLC_STOP,             // PLC 가동 정지 신호가 들어옴

    // 2. 센서 감지 관련 (from drv_exti.c)
    EVENT_AGV_SORT_ARRIVED,     // 분류쪽(PA8) AGV가 제자리에 도착함
    EVENT_AGV_LOAD_ARRIVED,     // 적재쪽(PB5) AGV가 제자리에 도착함
    EVENT_ITEM_ENTER_ROBOT_ZONE, // 로봇 가공 영역(PA10)에 물체가 들어옴

    // 3. 외부 장치 통신 관련
    EVENT_ROBOT_WORK_COMPLETE,  // 로봇으로부터 작업 완료 신호(PB1)를 받음
    EVENT_CAMERA_SCAN_SMALL,    // 카메라 판독 결과: 소분류(그냥 통과) 대상
    EVENT_CAMERA_SCAN_LARGE,    // 카메라 판독 결과: 대분류(로봇 가공) 대상

    // 4. 시스템 예외 및 에러 관련
    EVENT_EMERGENCY_STOP,       // 비상 정지 발생
    EVENT_TIMEOUT_ERROR,        // 정해진 시간 내에 센서/로봇 신호가 안 옴
    EVENT_ERROR_OCCURRED        // 기타 정의되지 않은 에러 발생
} AppEvent_t;

/**
 * @brief 이벤트를 우체통(Queue/Flag)에 넣는 함수
 * @param event 발생한 사건 번호
 */
void APP_EVENT_Post(AppEvent_t event);

/**
 * @brief 우체통에서 이벤트를 꺼내오는 함수 (Main 루프용)
 * @return AppEvent_t 발생했던 사건 번호
 */
AppEvent_t APP_EVENT_Get(void);


#endif /* APP_APP_EVENT_H_ */
