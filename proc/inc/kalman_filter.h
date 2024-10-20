#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q_angle; // 角度数据置信度，角度噪声的协方差
    float q_gyro;  // 角速度数据置信度，角速度噪声的协方差
    float r_angle; // 加速度计测量噪声的协方差
    float dt;      // 采样周期即计算任务周期10ms

    float angle;   // 角度
    float bias;    // 偏差
    float rate;    // 角速度

    float p[2][2]; // 误差协方差矩阵
} kalman_filter_t;

// 初始化卡尔曼滤波结构体
void kalman_filter_init(kalman_filter_t *kf, float dt);
// 卡尔曼滤波更新函数
void kalman_filter_update(kalman_filter_t *kf, float new_angle, float new_rate);

#ifdef __cplusplus
}
#endif

#endif /* _KALMAN_FILTER_H_ */
