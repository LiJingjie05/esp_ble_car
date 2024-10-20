#include "comp_filt.h"

#define M_PI		3.14159265358979323846

/**
 * @brief 互补滤波
 * 
 * @param dt 采样时间间隔，单位：秒
 * @param alpha 互补滤波器系数
 * @param acc_data 加速度 （输入）
 * @param gyro_data 陀螺仪 （输入）
 * @param pitch 欧拉角 （输出）
 * @param roll 欧拉角 （输出）
 */
void complementary_filter_roll(float dt, float alpha, uint16_t acc_data[3], uint16_t gyro_data[3], float *pitch, float *roll)
{
    float acc_pitch, acc_roll;
    float gyro_pitch, gyro_roll;

    // 加速度计计算俯仰角和横滚角
    acc_pitch = atan2f((float)acc_data[1], (float)acc_data[2]) * 180 / M_PI;
    acc_roll = atan2f((float)acc_data[0], (float)acc_data[2]) * 180 / M_PI;

    // 陀螺仪积分得到俯仰角和横滚角
    gyro_pitch = *pitch + ((float)gyro_data[0]) * dt;
    gyro_roll = *roll - ((float)gyro_data[1]) * dt;

    // 互补滤波器融合
    *pitch = alpha * gyro_pitch + (1 - alpha) * acc_pitch;
    *roll = alpha * gyro_roll + (1 - alpha) * acc_roll;
}

