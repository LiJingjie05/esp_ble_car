#include "pwm.h"

void motor_init(motor_t *motor, int gpio_a, int gpio_b, ledc_channel_t channel_a, ledc_channel_t channel_b)
{
    motor->gpio_a = gpio_a;
    motor->gpio_b = gpio_b;

    // 配置 LEDC 定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 8000,  // 频率
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 配置 LEDC 通道 A
    motor->ledc_channel_a = (ledc_channel_config_t){
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = channel_a,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio_a,
        .duty           = 0,   // 初始占空比
        .hpoint         = 0
    };
    ledc_channel_config(&motor->ledc_channel_a);

    // 配置 LEDC 通道 B
    motor->ledc_channel_b = (ledc_channel_config_t){
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = channel_b,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio_b,
        .duty           = 0,   // 初始占空比
        .hpoint         = 0
    };
    ledc_channel_config(&motor->ledc_channel_b);
}

void set_motor_pwm(motor_t *motor, uint8_t duty_a, uint8_t duty_b)
{
    // 将 duty 转换为 LEDC 模块的占空比
    uint32_t duty_cycle_a = (duty_a * ((1 << LEDC_TIMER_13_BIT) - 1)) / 255;
    uint32_t duty_cycle_b = (duty_b * ((1 << LEDC_TIMER_13_BIT) - 1)) / 255;

    ledc_set_duty(motor->ledc_channel_a.speed_mode, motor->ledc_channel_a.channel, duty_cycle_a);
    ledc_update_duty(motor->ledc_channel_a.speed_mode, motor->ledc_channel_a.channel);

    ledc_set_duty(motor->ledc_channel_b.speed_mode, motor->ledc_channel_b.channel, duty_cycle_b);
    ledc_update_duty(motor->ledc_channel_b.speed_mode, motor->ledc_channel_b.channel);
}
