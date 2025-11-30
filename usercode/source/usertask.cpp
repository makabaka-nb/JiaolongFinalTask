#include "usertask.h"
#include "bmi88.h"
#include "usart.h"
#include "can.h"
#include "dma.h"
#include <cmath>

#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"

// ==========================================
// 1. 全局对象实例化
// ==========================================
PID pitch_spd(0.0009f, 0.0f, 0.f, 0.4, 3, 0.1f);
PID pitch_pos(230.0f, 0.025f, 3400.0f, 50, 3000, 0.06f);
PID yaw_spd(0.005f, 0.0f, 0.f, 0.4, 3, 0.1f);
PID yaw_pos(11.0f, 0.001f, 195.0f, 0, 3000, 0.1f);

Motor g_pitch_motor(4, 1.0f,POSITION_SPEED, 0, 1, pitch_spd, pitch_pos);
Motor g_yaw_motor(1, 1.0f, POSITION_SPEED, 0, 1, yaw_spd, yaw_pos);

remote_control g_rc(&huart3);

float R_imu_matrix[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
float gyro_bias_arr[3] = {0,0,0};
IMU g_imu(0.001f, 0.1f, 0.05f, R_imu_matrix, gyro_bias_arr);

osThreadId_t controlTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t motorTaskHandle;
osThreadId_t watchdogTaskHandle;

// 信号量与事件标志
osSemaphoreId_t imu_semHandle;
osSemaphoreId_t rc_semHandle;
osEventFlagsId_t sys_eventHandle;
#define EVENT_IMU_READY (1 << 0)

// IMU 任务
void StartIMUTask(void *argument) {
    EulerAngle_t eulerAngle(0,0,0);
    g_imu.init(eulerAngle);
    osEventFlagsSet(sys_eventHandle, EVENT_IMU_READY);
    for(;;) {
        // 1. 读取与解算
        g_imu.readSensor();
        g_imu.update();
        osSemaphoreRelease(imu_semHandle);
        osDelay(10);
    }
}

void StartControlTask(void *argument) {
    // 开启DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_rc.rx_buf, 32);
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;

    // 使用 Setter 设置限位 (因为变量是 private)
    g_pitch_motor.SetAngleLimit(-90.0f, 90.0f);

    osEventFlagsWait(sys_eventHandle, EVENT_IMU_READY, osFlagsWaitAny, osWaitForever);
    osDelay(500); // 等待读数稳定
    g_pitch_motor.SetZeroPoint();
    g_yaw_motor.SetZeroPoint();
    for(;;) {
        if (osSemaphoreAcquire(imu_semHandle, osWaitForever) == osOK) {
            static int rc_timeout = 0;
            if (osSemaphoreAcquire(rc_semHandle, 0) == osOK) {
                rc_timeout = 0;
                g_rc.handle(g_rc.rx_buf, 18);
            } else {
                rc_timeout++;
            }
            if (rc_timeout > 50) {
                g_pitch_motor.stop_flag_ = true;
                g_yaw_motor.stop_flag_ = true;
            }
            const RC_Data_t& rc = g_rc.get_data();

            if (rc.s1 == RC_SW_DOWN) {
                // 停止
                g_pitch_motor.stop_flag_ = true;
                g_yaw_motor.stop_flag_ = true;
                g_pitch_motor.SetTorque(0);
                g_yaw_motor.SetTorque(0);

                target_pitch = g_pitch_motor.GetAngle();
                target_yaw = g_yaw_motor.GetAngle();
            } else if (rc.s1 == RC_SW_UP) {
                // 运动
                g_pitch_motor.stop_flag_ = false;
                g_yaw_motor.stop_flag_ = false;

                float yaw_in = rc.ch0;
                float pitch_in = rc.ch2;

                target_yaw  = yaw_in *170.0f;
                target_pitch = pitch_in * 55.0f;

                if (target_pitch > 90.0f) target_pitch = 90.0f;
                if (target_pitch < -90.0f) target_pitch = -90.0f;

                g_pitch_motor.SetPosition(target_pitch, 0, 0);

                g_yaw_motor.SetPosition(target_yaw, 0, 0);

            }
        }
    }
}
//电机任务
void StartMotorTask(void *argument) {
    for(;;) {
        const auto tick = osKernelGetTickCount();
        if (!g_pitch_motor.stop_flag_) g_pitch_motor.Handle();
        if (!g_yaw_motor.stop_flag_) g_yaw_motor.Handle();
        Motor::SendControlData(&hcan1);
        osDelayUntil(tick + 1);
    }
}

// // --- 看门狗任务 ---
// void StartWatchdogTask(void *argument) {
//     extern IWDG_HandleTypeDef hiwdg;
//     for(;;) {
//         // HAL_IWDG_Refresh(&hiwdg);
//         osDelay(500);
//     }
// }

// --- 初始化入口 ---
void User_Tasks_Init(void) {
    // 1. 创建二值信号量
    // max_count = 1, initial_count = 0 (表示开始是“红灯”，等待生产者释放变“绿灯”)
    bmi088_init();
    imu_semHandle = osSemaphoreNew(1, 0, NULL);
    rc_semHandle = osSemaphoreNew(1, 0, NULL);

    sys_eventHandle = osEventFlagsNew(NULL);

    const osThreadAttr_t imu_attr = { .name = "IMU", .stack_size = 512 * 4, .priority = (osPriority_t)osPriorityHigh };
    const osThreadAttr_t ctrl_attr = { .name = "Control", .stack_size = 512 * 4, .priority = (osPriority_t)osPriorityAboveNormal };
    const osThreadAttr_t motor_attr = { .name = "Motor", .stack_size = 512 * 4, .priority = (osPriority_t)osPriorityRealtime };
    const osThreadAttr_t wdg_attr = { .name = "Watchdog", .stack_size = 128 * 4, .priority = (osPriority_t)osPriorityLow };

    imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imu_attr);
    controlTaskHandle = osThreadNew(StartControlTask, NULL, &ctrl_attr);
    motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motor_attr);
    // watchdogTaskHandle = osThreadNew(StartWatchdogTask, NULL, &wdg_attr);
}