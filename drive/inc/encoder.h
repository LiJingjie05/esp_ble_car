#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdio.h>
#include "freertos/portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

// 编码器结构体
typedef struct {
    int gpio_a;
    int gpio_b;
    volatile int16_t count;
    int last_state;
    portMUX_TYPE mux;
} encoder_t;

// 初始化编码器
void encoder_init(encoder_t *encoder, int gpio_a, int gpio_b);

// 获取编码器计数值
int get_encoder_val(encoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
