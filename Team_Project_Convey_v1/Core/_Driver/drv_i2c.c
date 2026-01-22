/*
 * drv_i2c.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */


#include "config.h"      // I2C_MOTOR_DRV(hi2c1) 정의 참조
#include "system_state.h"

// 전송 상태 플래그: DMA 전송 중 중복 요청을 방지하기 위함
static volatile uint8_t i2c_tx_in_progress = 0; 	// 0일 시 통로 개방. 세마포어 구현

/**
 * @brief I2C DMA 전송 완료 콜백 (인터럽트)
 * 데이터가 전선으로 다 나갔을 때 호출되어 통로를 비워줌
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {	// DMA 전송 완료 확인
    if (hi2c->Instance == I2C_MOTOR_DRV.Instance) {
        i2c_tx_in_progress = 0; 	// 통로 개방 스위치 ON, 다음 데이터를 보낼 수 있는 상태로 전환합니다
    }
}

/**
 * @brief I2C 에러 콜백
 * 전송 중 하드웨어 에러 발생 시 처리
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {		// 전송 에러 처리. 에러난 부분은 ERR_I2C_MOTOR_FAIL 에 적을 수 있도록 통로를 열어줌
    if (hi2c->Instance == I2C_MOTOR_DRV.Instance) {
        i2c_tx_in_progress = 0; 	// 에러 시에도 통로 개방 스위치 ON, 다음 데이터를 보낼 수 있는 상태로 전환합니다
        // 여기서 직접 장부를 수정하지 않고, 에러 상태만 리턴하여 상위 계층이 판단하게 함
    }
}

/**
 * @brief I2C 블로킹 송신 (동기 방식)
 * 초기화 단계(app_init -> bsp_i2c)에서 설정값을 안전하게 보낼 때 사용
 */
HAL_StatusTypeDef DRV_I2C_Transmit(uint16_t devAddr, uint8_t *pData, uint16_t size) {
    // 버스가 사용 가능할 때까지 대기 후 전송 (타임아웃 100ms)
    return HAL_I2C_Master_Transmit(&I2C_MOTOR_DRV, devAddr, pData, size, 100);
}

/**
 * @brief I2C DMA 송신 (비동기 방식)
 * task_motor가 주기적으로 모터 데이터를 쏠 때 CPU 점유율을 최소화하기 위해 사용
 */
HAL_StatusTypeDef DRV_I2C_Transmit_DMA(uint16_t devAddr, uint8_t *pData, uint16_t size) {
    // 1. 이전 전송이 아직 끝나지 않았다면 Busy 반환
    if (i2c_tx_in_progress) {
        return HAL_BUSY;		// i2c_tx_in_progress 가 1이면 실행을 거절하고 상태를 반환
    }

    // 2. 전송 플래그 설정 후 DMA 기동
    i2c_tx_in_progress = 1;		// I2C 버스 점유

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(&I2C_MOTOR_DRV, devAddr, pData, size);

    // 3. 만약 즉시 실패(에러)가 발생했다면 플래그 복구
    if (status != HAL_OK) {
        i2c_tx_in_progress = 0;
    }

    return status;
}

/**
 * @brief 현재 I2C 전송 통로 상태 확인
 * @return 1: 전송 가능(Ready), 0: 전송 중(Busy)
 */
uint8_t DRV_I2C_GetReadyStatus(void) {
    return (i2c_tx_in_progress == 0) ? 1 : 0;
}



// 정리
// HAL_I2C_MasterTxCpltCallback :
// HAL_I2C_ErrorCallback :
// DRV_I2C_Transmit
// DRV_I2C_Transmit_DMA
//
