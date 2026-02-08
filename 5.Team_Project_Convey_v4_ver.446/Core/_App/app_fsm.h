/*
 * app.fsm.h
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */

#ifndef APP_APP_FSM_H_
#define APP_APP_FSM_H_

/* 비전 큐 */
void visionQ_push(CameraResult_t item);
CameraResult_t visionQ_pop(void);

/* 공정 관리 함수 */
void APP_FSM_Init(void);
void APP_FSM_Execute(void);

#endif /* APP_APP_FSM_H_ */
