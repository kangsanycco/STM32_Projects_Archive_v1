/*
 * app_event.c
 *
 * Created on: Jan 17, 2026
 * Author: kangs
 */

#include "app_event.h"

#define MAX_EVENT_QUEUE  16  // 동시에 저장할 수 있는 최대 이벤트 수

// 1. 이벤트 저장소 (순환 큐 구조)
static AppEvent_t event_queue[MAX_EVENT_QUEUE];  // AppEvent_t 타입을 담을 수 있는 16칸 배열(FIFO 구조)
static int head = 0; // 데이터를 꺼낼 위치(자리), 가장 오래된 사건의 위치
static int tail = 0; // 데이터를 넣을 위치(자리), 가장 최신 사건의 위치

/**
 * @brief 이벤트를 게시(Post)함 - Driver(인터럽트)에서 호출
 * @param event 발생한 사건 번호
 */
void APP_EVENT_Post(AppEvent_t incoming_event)  // 이벤트 발생, 저장, tail 증가에 관한 함수
{
    int next_tail = (tail + 1) % MAX_EVENT_QUEUE;	// 다음 데이터가 들어갈 자리(tail) 미리 계산, 15번 다음은 다시 0번으로 돌아가는 순환 구조(나머지 연산)

    // 큐가 꽉 찼는지 확인 (꽉 찼다면 가장 오래된 데이터는 포기하거나 덮어씀)
    if (next_tail != head) {	// 다음 넣을 자리가, 출구와 같지 않다면
        event_queue[tail] = incoming_event;		// 현재 비어있는 입구 자리에 사건을 넣음, 매개변수가 event로 되어있음
        tail = next_tail;		// 입구 위치를 다음 칸으로 한 칸 옮김
    }
}		// 3번 자리에 넣을건데 다음 자리가 4라면, 3번 자리에다가 사건을 넣고 다음 자리를 4로 바꿈. 15번을 넣을 건데, 다음 head 자리가 0이라면 아무일도 일어나지 않음

/**
 * @brief 발생한 이벤트를 하나 꺼내옴 - Main Loop(Task)에서 호출
 * @return AppEvent_t 꺼내온 이벤트 (없으면 EVENT_NONE)
 */
AppEvent_t APP_EVENT_Get(void) // 본부(App Task)가 "무슨 일 없었어?" 하고 물어볼 때 호출하는 함수, head 증가에 관한 함수
{
    // 꺼낼 데이터가 없는 경우
    if (head == tail) {		// 만약 head와 tail 이 같은 자리에 위치한다면
        return EVENT_NONE;	// event는 없다는 뜻임
    }

    AppEvent_t outgoing_event = event_queue[head];	// 출구에 있는 가장 오래된 사건을 불러들임
    head = (head + 1) % MAX_EVENT_QUEUE;	// 사건을 보냈다면, 출구 위치를 다음 칸으로 옮김, 순환 구조(15번 다음은 0번)

    return outgoing_event;		// 꺼낸 사건을 본부(App Task)에 전달, event는 지역변수처럼 보이지만, event_queue[] 를 통해 공유하고 있음
}

/**
 * @brief 현재 대기 중인 이벤트가 있는지 확인 (필요 시 사용)
 */
int APP_EVENT_IsEmpty(void)
{
    return (head == tail);
}
