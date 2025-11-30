//
// Created by ROG STRIX on 2025/10/25.
//
#include"remote_control.h"

#include <cstring>

remote_control::remote_control(UART_HandleTypeDef *huart):huart_(huart) {
    last_update_time = 0;
    is_connected = false;
    memset(&rc_data, 0, sizeof(rc_data));
    memset(rx_buf, 0, sizeof(rx_buf));
}
float remote_control::linearmapping(uint16_t ch) {
    return static_cast<float>((ch-1024.0f)/660.0f);
}
void remote_control::init() {
    rc_data.ch0 = 0.0f;
    rc_data.ch1 = 0.0f;
    rc_data.ch2 = 0.0f;
    rc_data.ch3 = 0.0f;
    rc_data.s1 = RC_SW_MID;
    rc_data.s2 = RC_SW_MID;
    rc_data.mouse_x = 0;
    rc_data.mouse_y = 0;
    rc_data.mouse_z = 0;
    rc_data.press_l=0;
    rc_data.press_r=0;
    rc_data.key = 0;
}

void remote_control::handle(uint8_t *pdata,uint8_t size) {
    last_update_time = HAL_GetTick();
    is_connected = true;
    rc_data.ch0= linearmapping((static_cast<uint16_t>(pdata[0])|(static_cast<uint16_t>(pdata[1])<<8))&0x07FF);
    rc_data.ch1 = linearmapping(((static_cast<uint16_t>(pdata[1]) >> 3) | (static_cast<uint16_t>(pdata[2]) << 5))& 0x07FF);
    rc_data.ch2 = linearmapping(((static_cast<uint16_t>(pdata[2]) >> 6) | (static_cast<uint16_t>(pdata[3]) << 2) |((uint16_t)pdata[4] << 10)) & 0x07FF);
    rc_data.ch3 = linearmapping(((static_cast<uint16_t>(pdata[4]) >> 1) | (static_cast<uint16_t>(pdata[5])<<7)) &0x07FF);
    rc_data.s1 = RC_SwitchState(((static_cast<int16_t>(pdata[5])>>4) & 0x000C) >> 2);
    rc_data.s2 = RC_SwitchState(((static_cast<int16_t>(pdata[5])>>4) & 0x0003));
    rc_data.mouse_x = (static_cast<int16_t>(pdata[6])) | (static_cast<int16_t>(pdata[7]) << 8);
    rc_data.mouse_y = ((int16_t)pdata[8]) | ((int16_t)pdata[9] << 8);
    rc_data.mouse_z = ((int16_t)pdata[10]) | ((int16_t)pdata[11] << 8);
    rc_data.press_l = pdata[12];
    rc_data.press_r = pdata[13];
    rc_data.key = ((int16_t)pdata[14]);// | ((int16_t)pData[15] << 8);
}
