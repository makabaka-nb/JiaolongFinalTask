//
// Created by ROG STRIX on 2025/10/25.
//

#ifndef BMI088_REMOTE_CONTROL_H
#define BMI088_REMOTE_CONTROL_H
#include <cstdint>

#include "main.h"

// 开关状态枚举
typedef enum {
    RC_SW_UP = 1,
    RC_SW_MID = 3,
    RC_SW_DOWN = 2
} RC_SwitchState;
// 遥控器数据结构体
typedef struct {
    // 摇杆数据 (-1.0 ~ 1.0)
    float ch0;  // 右摇杆左右
    float ch1;  // 右摇杆上下
    float ch2;  // 左摇杆左右
    float ch3;  // 左摇杆上下
    // 开关状态
    RC_SwitchState s1;  // 左开关
    RC_SwitchState s2;  // 右开关
    // 鼠标数据 (可选)
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t press_l;
    uint8_t press_r;
    // 按键 (16位掩码)
    uint16_t key;
} RC_Data_t;
class remote_control {
private:
    UART_HandleTypeDef* huart_;

    bool is_connected;
    RC_Data_t rc_data;
    uint8_t last_update_time;
public:
    uint8_t rx_buf[32];
    uint8_t rx_data1[32];
    remote_control(UART_HandleTypeDef* huart);
    void init();
    void handle(uint8_t *pdata,uint8_t size);
    bool isconnected(){return is_connected;}
    uint32_t get_last_update() { return last_update_time; }
    static float linearmapping(uint16_t ch);
    RC_Data_t get_data(){return rc_data;}
};
#endif //BMI088_REMOTE_CONTROL_H