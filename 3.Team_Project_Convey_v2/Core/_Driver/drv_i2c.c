/*
 * drv_i2c.c
 *
 *  Created on: Jan 17, 2026
 *      Author: kangs
 */


#include "config.h"
#include "system_state.h"
#include "function.h"

/**
 * @brief PCA9685 특정 레지스터에 1바이트 쓰기
 */
HAL_StatusTypeDef DRV_I2C_WriteReg(uint8_t reg, uint8_t data) // reg: 레지스터, data: 그 안에 넣을 데이터
{
    return HAL_I2C_Mem_Write(I2C_MOTOR_DRV, ADDR_I2C_PCA9685, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    // HAL_I2C_Mem_Write : 특정 주소의 공간에 데이터를 집어넣는다
    // ADDR_I2C_PCA9685: 칩 주소, config.h에 정의된 (0x40 << 1)
    // I2C_MOTOR_DRV: 통로, config.h에 정의된 &hi2c1
}

/**
 * @brief PCA9685 드라이버 초기화
 * @note 기상(Wake up) 및 주파수 설정
 */
void DRV_I2C_Init(void)
{
    // 1. 장치 존재 확인
    if (HAL_I2C_IsDeviceReady(I2C_MOTOR_DRV, ADDR_I2C_PCA9685, 3, 10) != HAL_OK) {
        g_sys_status.lastError = ERR_I2C1_DISCONNECTED;
        return;
        //HAL_I2C_IsDeviceRedady(): 드라이버가 제대로 연결되어 있는지 확인
    }

    // 2. 주파수 설정 (50Hz - 서보 모터 표준)
    // PRE_SCALE 레지스터(0xFE)를 건드리려면 잠시 SLEEP(0x11) 모드로 들어가야 합니다.
    DRV_I2C_WriteReg(0x00, 0x11); // 잠시 취침 (Sleep ON)
    DRV_I2C_WriteReg(0xFE, 121);  // 50Hz 설정을 위한 계산값 (Prescale). Prescale = round(25000000/(4096*50))-1 = 121
    HAL_Delay(5);

    // 2. PCA9685 모드 설정 (SLEEP 해제)
    DRV_I2C_WriteReg(0x00, 0x21); // Mode 1 레지스터를 AI(Auto Increment) 모드로 설정, 데이터 0x21을 레지스터 0x00 위치에 집어넣는다
    HAL_Delay(5); // 안정화 시간
}


/**
 * @brief 로봇에게 동작 시작 명령 전송
 */
HAL_StatusTypeDef DRV_I2C_Robot_SendStart(void)
{
    uint8_t cmd = ROBOT_ORDER_START; // config.h의 0x01
    // 로봇은 레지스터 구조가 아닐 수 있으므로 일반 Transmit 사용
    return HAL_I2C_Master_Transmit(I2C_MOTOR_DRV, ADDR_I2C_ROBOT, &cmd, 1, 10);
}

/**
 * @brief 필요할 때 즉시, 로봇으로부터 현재 상태 읽기. 수동용 함수
 */
//uint8_t DRV_I2C_Robot_ReadStatus(void)
//{
//    uint8_t status = 0;
//    // 로봇 제어기로부터 1바이트 데이터 수신
//    if (HAL_I2C_Master_Receive(I2C_MOTOR_DRV, ADDR_I2C_ROBOT, &status, 1, 10) != HAL_OK) {
//        return 0; // 통신 실패 시 기본값
//    }
//    return status;
//}
//
//
//
//void DRV_I2C_Robot_ReceiveInterrupt(uint8_t *pBuffer)
//{
//    // 인터럽트 방식으로 1바이트 수신 대기 (CPU는 다른 일을 할 수 있음)
//    HAL_I2C_Master_Receive_IT(I2C_MOTOR_DRV, ADDR_I2C_ROBOT, pBuffer, 1);
//}
//
//
//
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if (hi2c->Instance == I2C1)
//    {
//        if (g_sys_status.robot_i2c_rx_buf == ROBOT_STATUS_DONE) // 0x02 확인
//        {
//            g_sys_status.sensor_robot_done = 1; // FSM이 기다리는 완료 플래그 ON
//        }
//        else
//        {
//            // 원하는 데이터가 아니면 다시 수신 대기
//            DRV_I2C_Robot_ReceiveInterrupt(&g_sys_status.robot_i2c_rx_buf);
//        }
//    }
//}
