// //
// // Created by ROG STRIX on 2025/10/12.
// //
// #include"imu.h"
//
// #include <cmath>
// #include <cstring>
// #include"mahony.h"
//
// #include "bmi88.h"
// #include"main.h"
// #define g 9.81f
// #define PI 3.14159265
//  uint8_t rx_acc_data[6];
//  uint8_t rx_gyro_data[6];
//  uint8_t rx_temp_data[2];
//  uint32_t count;
// IMU::IMU(const float& dt, const float& kg, const float& g_thres,
//         const float R_imu[3][3], const float gyro_bias[3]):mahony_(dt,kg,g_thres) {
//     memcpy(R_imu_,R_imu,sizeof(R_imu_));
//     memcpy(gyro_bias_,gyro_bias,sizeof(gyro_bias_));
// }
// void IMU::init(EulerAngle_t euler_deg_init) {
//     euler_deg_init.pitch = euler_deg_init.roll=euler_deg_init.yaw=0.0f;
//     euler_deg_=euler_deg_init;
//     euler_rad_=EulerAngle_t(0,0,0);
//     q_[0]=1;q_[1]=q_[2]=q_[3]=0;
// }
//
// // 在 IMU 类中增加一个成员（头文件 imu.h）：
// // float prev_raw_gyro_[3] = {0,0,0};
// // uint32_t stable_detect_count_ = 0;
//
// // 更稳健的在线更新函数（基于 raw 值）
// void IMU::updategyrobias(const float raw_rad[3], float dt) {
//     const float g_val = 9.81f;
//     const float acc_thresh = 0.12f; // m/s^2, 加速度接近1g的阈值
//     const float gyro_delta_thresh = 0.5f * (PI/180.0f); // 0.5 deg/s -> rad/s : 单次变化阈
//     const float alpha = 0.0003f; // 学习率，建议 1e-4 ~ 5e-4
//     const uint32_t stable_required = 250; // 250帧 * 2ms = 0.5s
//     // 1) accel magnitude 检查（使用已转换的 accel_sensor_，单位 m/s^2）
//     float acc_mag = sqrtf(accel_sensor_[0]*accel_sensor_[0] +
//                           accel_sensor_[1]*accel_sensor_[1] +
//                           accel_sensor_[2]*accel_sensor_[2]);
//     // 2) gyro 变化量检查（raw 与上次 raw 的差）
//     float max_delta = 0.0f;
//     for (int i = 0; i < 3; ++i) {
//         float delta = fabsf(raw_rad[i] - prev_raw_gyro_[i]);
//         if (delta > max_delta) max_delta = delta;
//     }
//     // 更新 prev_raw_gyro_ 以备下一帧判断（但要在判断后或之前根据需求）
//     // 我们暂时先用它来判断，再在最后更新 prev_raw_gyro_
//     if (fabsf(acc_mag - g_val) < acc_thresh && max_delta < gyro_delta_thresh) {
//         stable_detect_count_++;
//     } else {
//         stable_detect_count_ = 0;
//     }
//     // 当稳定超过阈值时才开始缓慢更新 bias
//     if (stable_detect_count_ >= stable_required) {
//         for (int i = 0; i < 3; ++i) {
//             // bias = (1-alpha)*bias + alpha * raw
//             gyro_bias_[i] = (1.0f - alpha) * gyro_bias_[i] + alpha * raw_rad[i];
//         }
//     }
//     // 最后保存本次 raw 作为下次比较基础
//     for (int i = 0; i < 3; ++i) prev_raw_gyro_[i] = raw_rad[i];
// }
//
//
// void IMU::readSensor() {
//     //加速度读取
//     uint8_t rx_acc_range_raw;
//     bmi088_accel_read_reg(0x41,&rx_acc_range_raw,1);
//     float rx_acc_range=(float) pow(2,rx_acc_range_raw+1)*1.5*g;
//     bmi088_accel_read_reg(0x12,rx_acc_data,6);
//     accel_sensor_[0]=(float)(int16_t)(rx_acc_data[0]|rx_acc_data[1]<<8)/32768.f*(float)rx_acc_range;
//     accel_sensor_[1]=(float)(int16_t)(rx_acc_data[2]|rx_acc_data[3]<<8)/32768.f*(float)rx_acc_range;
//     accel_sensor_[2]=(float)(int16_t)(rx_acc_data[4]|rx_acc_data[5]<<8)/32768.f*(float)rx_acc_range;
//     //角速度读取
//     uint8_t rx_gyro_range_raw;
//     bmi088_gyro_read_reg(0x0F,&rx_gyro_range_raw,1);
//     float rx_gyro_range;
//     switch(rx_gyro_range_raw) {
//         case 0x00: rx_gyro_range=16.384f;break;
//         case 0x01: rx_gyro_range=32.768f;break;
//         case 0x02: rx_gyro_range=65.536f;break;
//         case 0x03: rx_gyro_range=131.072f;break;
//         case 0x04: rx_gyro_range=262.144f;break;
//     }
//     bmi088_gyro_read_reg(0x02,rx_gyro_data,6);
//     gyro_sensor_dps_[0]=(float)(int16_t)(rx_gyro_data[0]|rx_gyro_data[1]<<8)/(float)rx_gyro_range;
//     gyro_sensor_dps_[1]=(float)(int16_t)(rx_gyro_data[2]|rx_gyro_data[3]<<8)/(float)rx_gyro_range;
//     gyro_sensor_dps_[2]=(float)(int16_t)(rx_gyro_data[4]|rx_gyro_data[5]<<8)/(float)rx_gyro_range;
//     gyro_sensor_[0] = gyro_sensor_dps_[0] * PI / 180.0f;
//     gyro_sensor_[1] = gyro_sensor_dps_[1] * PI / 180.0f;
//     gyro_sensor_[2] = gyro_sensor_dps_[2] * PI / 180.0f;
//     updategyrobias(gyro_sensor_,0.002f);
//     for (int i=0;i<3;i++) {
//         gyro_sensor_[i]-=gyro_bias_[i];
//     }
//     //updategyrobias(0.002f);
//     /*for (uint32_t i=0;i<3;i++) {
//         if (count<10000) {
//             gyro_bias_[i]+=gyro_sensor_[i];
//             count++;
//         }
//     }*/
//
//
//     //温度读取
//     bmi088_accel_read_reg(0x22,rx_temp_data,2);
//     uint8_t msb = rx_temp_data[0];
//     uint8_t lsb = rx_temp_data[1];
//     // 公式： Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32)
//     uint16_t temp_uint11 = ((uint16_t)msb << 3) | (lsb >> 5);
//     // 二补数转换（11 位）
//     int16_t temp_int11;
//     if (temp_uint11 > 1023) {
//         temp_int11 = (int16_t)(temp_uint11 - 2048);
//     } else {
//         temp_int11 = (int16_t)temp_uint11;
//     }
//     temprature_= (float)temp_int11 * 0.125f + 23.0f;
// }
//
// void IMU::update() {
//     float gyro_aligned[3],accel_aligned[3];
//     for (int i=0;i<3;i++) {
//         gyro_aligned[i]=R_imu_[i][0]*gyro_sensor_[0]+
//                         R_imu_[i][1]*gyro_sensor_[1]+R_imu_[i][2]*gyro_sensor_[2];
//         accel_aligned[i]=R_imu_[i][0]*accel_sensor_[0]+
//                         R_imu_[i][1]*accel_sensor_[1]+R_imu_[i][2]*accel_sensor_[2];
//     }
//     mahony_.update(q_,gyro_aligned,accel_aligned);
//     euler_rad_.roll=atan2f(2 * (q_[0] * q_[1] + q_[2] * q_[3]),
//                               1 - 2 * (q_[1] * q_[1] + q_[2] * q_[2]));
//     euler_rad_.pitch = asinf(2 * (q_[0] * q_[2] - q_[3] * q_[1]));
//     euler_rad_.yaw   = atan2f(2 * (q_[0] * q_[3] + q_[1] * q_[2]),
//                               1 - 2 * (q_[2] * q_[2] + q_[3] * q_[3]));
//     euler_deg_.roll  = euler_rad_.roll * 180.0f / PI;
//     euler_deg_.pitch = euler_rad_.pitch * 180.0f / PI;
//     euler_deg_.yaw   = euler_rad_.yaw * 180.0f / PI;
// }
//
