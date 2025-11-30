//
// Created by ROX on 2025/10/18.
//
#include "stm32f4xx_hal.h"
#include "can.h"
#include "motor.h"
#include <cmath>
uint8_t Motor::tx_data_1_[8] = { 0 };
uint8_t Motor::tx_data_2_[8] = { 0 };
//构造函数
Motor::Motor(uint8_t escid, float ratio, ControlMethod control_method,uint8_t M3508_flag,uint8_t GM6020_flag,PID spid,PID ppid):
    escid_(escid),
    ratio_(ratio),
    M3508_flag_(M3508_flag),
    GM6020_flag_(GM6020_flag),
    spid_(spid),
    ppid_(ppid),
    control_method_(control_method)
    {
    // 配置CAN报文头
    if (M3508_flag==1 && GM6020_flag==0) {
        if (1 <= escid && escid <= 4 ) {
            motor_tx_header_.StdId = M3508_TX_ID_1_4;
        } else if (5 <= escid && escid <= 8) {
            motor_tx_header_.StdId = M3508_TX_ID_5_8;
        }
    }
    else if (M3508_flag==0&&GM6020_flag==1) {
        if (1 <= escid && escid <= 4 ) {
            motor_tx_header_.StdId = GM6020_TX_ID_1_4;
        } else if (5 <= escid && escid <= 8) {
            motor_tx_header_.StdId = GM6020_TX_ID_5_8;
        }
    }
    motor_tx_header_.DLC = 8;
    motor_tx_header_.IDE = CAN_ID_STD;
    motor_tx_header_.RTR = CAN_RTR_DATA;
    motor_tx_header_.TransmitGlobalTime = DISABLE;
    // // 配置PID参数
    // spid_ = PID(0.005f, 0.0f, 0.0f, MAX_SPEED, MAX_CURRENT, 0.05f); // 内环 速度环PID
    // ppid_ = PID(20.0f, 0.01f, 0.0f, 100.0f, MAX_SPEED, 0.06f); // 外环 位置环PID
}
//解包代码
void Motor::can_rx_msg_callback(const uint8_t rxdata[8]) {
    // 解包编码器角度
    int16_t tmp = (rxdata[0] << 8) | rxdata[1];
    last_ecd_angle_ = ecd_angle_;
    ecd_angle_ = LinearMapping(tmp, 0, 8191, 0.0f, 360.0f);
    if (can_init_flag_) {
        last_ecd_angle_ = ecd_angle_;
        can_init_flag_ = false;
    }
    float delta_ecd_angle = ecd_angle_ - last_ecd_angle_;
    // 解决临界跳变
    if (delta_ecd_angle > 180.0f) {
        delta_ecd_angle -= 360.0f;
    } else if (delta_ecd_angle < -180.0f) {
        delta_ecd_angle += 360.0f;
    }
    if (M3508_flag_==1)delta_angle_ = delta_ecd_angle / ratio_;
    else if (GM6020_flag_==1) delta_angle_ = delta_ecd_angle;
    angle_ += delta_angle_;
    // 解包转速
    tmp = (rxdata[2] << 8) | rxdata[3];
    rotate_speed_ = static_cast<float>(tmp);
    // 解包电流
    tmp = (rxdata[4] << 8) | rxdata[5];
    current_ = LinearMapping(tmp, -16384, 16384, -3.0f, 3.0f);
    // 解包温度，并检查是否过热
    temp_ = static_cast<float>(rxdata[6]);
    if (temp_ >= MAX_TEMP) { // 滞回比较
        overheat_flag_ = true;
    } else if (temp_ <= WARN_TEMP) {
        overheat_flag_ = false;
    }
}
//电流发送
void Motor::SetCurrent(float current) {
    if (current > MAX_CURRENT) {
        current = MAX_CURRENT;
    } else if (current < -MAX_CURRENT) {
        current = -MAX_CURRENT;
    }
    // 映射到电调指令范围 -16384~16384 -20A~20A
    int16_t current_cmd = static_cast<int16_t>(current * 16384.0f / 3.0f);
    int data_pos_high = 0, data_pos_low = 0;
    if (1 <= escid_ && escid_ <= 4) {
        data_pos_high = 2 * escid_ - 2;
        data_pos_low = 2 * escid_ - 1;
        tx_data_1_[data_pos_high] = current_cmd >> 8;
        tx_data_1_[data_pos_low] = current_cmd & 0xff;
        //HAL_CAN_AddTxMessage(&hcan1, &motor_tx_header_, tx_data_1_, nullptr);
    } else if (5 <= escid_ && escid_ <= 8) {
        data_pos_high = 2 * (escid_ - 4) - 2;
        data_pos_low = 2 * (escid_ - 4) - 1;
        tx_data_2_[data_pos_high] = current_cmd >> 8;
        tx_data_2_[data_pos_low] = current_cmd & 0xff;
        //HAL_CAN_AddTxMessage(&hcan1, &motor_tx_header_, tx_data_2_, nullptr);
    }
}
void Motor::SendControlData(CAN_HandleTypeDef *hcan) {
    CAN_TxHeaderTypeDef tx_header;
    tx_header.DLC = 8;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t send_mail_box;

    tx_header.StdId = 0x1FE;
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data_1_, &send_mail_box);

    // 如果有 ID 5-8，再发一帧
    tx_header.StdId = 0x2FE;
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data_2_, &send_mail_box);
}
float Motor::LinearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    return static_cast<float>(in - in_min) * (out_max - out_min) / static_cast<float>(in_max - in_min) + out_min;
}

void Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    control_method_ = POSITION_SPEED;
    if (target_position<limited_min_angle_) target_position=limited_min_angle_;
    if (target_position>limited_max_angle_)  target_position=limited_max_angle_;
    this->target_angle_ = target_position;
    this->feedforward_speed_ = feedforward_speed;
    this->feedforward_torque_ = feedforward_intensity;
}

void Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    control_method_ = SPEED;
    this->target_speed_ = target_speed;
    this->feedforward_torque_ = feedforward_intensity;
}

void Motor::SetTorque(float torque) {
    control_method_ = TORQUE;
    this->output_torque_= torque;
}

void Motor::Handle() {
    float output = 0.0f;
    if (prev_method_ != control_method_) {
        // 切换控制模式时重置PID状态
        spid_.reset();
        ppid_.reset();
        prev_method_ = control_method_;
    }
    // 根据控制模式计算输出力矩
    if (control_method_ == TORQUE) {
        // 直接控制输出力矩 无需PID
    } else if (control_method_ == SPEED) {
        fdb_speed_ = rotate_speed_;
        // 内环PID 速度误差 -> 输出力矩
        output_torque_ = spid_.calc(target_speed_, fdb_speed_);
    } else if (control_method_ == POSITION_SPEED) {
        fdb_angle_ = angle_;
        fdb_speed_ = rotate_speed_;
        // 外环PID 角度误差 -> 目标速度
        target_speed_ = ppid_.calc(target_angle_, fdb_angle_);
        target_speed_ += feedforward_speed_; // 加上前馈速度
        // 内环PID 速度误差 -> 输出力矩
        output_torque_ = spid_.calc(target_speed_, fdb_speed_);
    }
    // 根据角度更新前馈力矩
    //feedforward_torque_ = FeedforwardTorqueCalc(angle_);
    output = output_torque_ + feedforward_torque_;
    // 过热保护 & 停止保护
    if (overheat_flag_ || stop_flag_) {
        output = 0.0f;
    }
    // 发送控制命令
    if (escid_==4)SetCurrent(output);
    if (escid_==1)SetCurrent(TorqueToCurrent(output));
}

void Motor::SetZeroPoint() {
    zero_angle_offset_=angle_;
    ppid_.reset();
    spid_.reset();
}

/*float Motor::FeedforwardTorqueCalc(float current_angle) {
    // const float m = 500.0f; // 负载质量 g
    // const float r = 55.24f; // 负载到旋转轴距离 mm
    // const float g = 9.81f; // 重力加速度 m/s²
    // float angle_rad = current_angle * PI / 180.0f;
    // return m * g * r * std::sin(angle_rad) / 1e6; // N·m
}*/

float Motor::TorqueToCurrent(float torque) {
    return torque / torque_constant_;
}