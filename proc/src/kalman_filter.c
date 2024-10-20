#include "kalman_filter.h"

// 初始化卡尔曼滤波结构体
void kalman_filter_init(kalman_filter_t *kf, float dt) {
    kf->q_angle = 0.001;
    kf->q_gyro = 0.003;
    kf->r_angle = 0.5;
    kf->dt = dt;

    kf->angle = 0.0;
    kf->bias = 0.0;
    kf->rate = 0.0;

    kf->p[0][0] = 1.0;
    kf->p[0][1] = 0.0;
    kf->p[1][0] = 0.0;
    kf->p[1][1] = 1.0;
}

// 卡尔曼滤波更新函数
void kalman_filter_update(kalman_filter_t *kf, float new_angle, float new_rate) {
    kf->rate = new_rate - kf->bias;
    kf->angle += kf->dt * kf->rate;

    kf->p[0][0] += kf->dt * (kf->dt*kf->p[1][1] - kf->p[0][1] - kf->p[1][0] + kf->q_angle);
    kf->p[0][1] -= kf->dt * kf->p[1][1];
    kf->p[1][0] -= kf->dt * kf->p[1][1];
    kf->p[1][1] += kf->q_gyro * kf->dt;

    float s = kf->p[0][0] + kf->r_angle;
    float k[2] = {
      kf->p[0][0] / s,
      kf->p[1][0] / s
    };

    float y = new_angle - kf->angle;
    kf->angle += k[0] * y;
    kf->bias += k[1] * y;

    float p00_temp = kf->p[0][0];
    float p01_temp = kf->p[0][1];

    kf->p[0][0] -= k[0] * p00_temp;
    kf->p[0][1] -= k[0] * p01_temp;
    kf->p[1][0] -= k[1] * p00_temp;
    kf->p[1][1] -= k[1] * p01_temp;
}

// // 使用示例
// kalman_filter_t kf;
// kalman_filter_init(&kf, 0.01); // 初始化卡尔曼滤波器，设置采样周期为10ms
// // 在每个采样周期调用kalman_filter_update(&kf, new_angle, new_rate);

