#ifndef _PWM_H_
#define _PWM_H_

#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 电机结构体
typedef struct {
    int gpio_a;
    int gpio_b;
    ledc_channel_config_t ledc_channel_a;
    ledc_channel_config_t ledc_channel_b;
} motor_t;

// 初始化电机
void motor_init(motor_t *motor, int gpio_a, int gpio_b, ledc_channel_t channel_a, ledc_channel_t channel_b);

// 设置电机 PWM 占空比
void set_motor_pwm(motor_t *motor, uint8_t duty_a, uint8_t duty_b);

#ifdef __cplusplus
}
#endif

#endif /* _PWM_H_ */
