//
// Created by ROG STRIX on 2025/11/23.
//
#include "main.h"
#include "usertask.h"
#include "usart.h"
#include "can.h"
#include "motor.h"
#include "remote_control.h"

// 遥控器中断回调
extern Motor g_yaw_motor;
extern Motor g_pitch_motor;
extern remote_control g_rc;
extern osThreadId_t controlTaskHandle;
extern osThreadId_t imuTaskHandle;
extern osThreadId_t motorTaskHandle;
extern osThreadId_t watchdogTaskHandle;

// 信号量与事件标志
extern osSemaphoreId_t imu_semHandle;
extern osSemaphoreId_t rc_semHandle;
extern osEventFlagsId_t sys_eventHandle;
extern"C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*huart,uint16_t size) {
    if (huart->Instance == USART3) {
        osSemaphoreRelease(rc_semHandle);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rc.rx_buf, 32);
    }
}

// CAN 中断回调
extern"C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId==0x208) {
            g_pitch_motor.can_rx_msg_callback(rx_data);
        }
        else if (rx_header.StdId==0x205) {
            g_yaw_motor.can_rx_msg_callback(rx_data);

        }
    }
}