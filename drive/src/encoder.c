#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "encoder.h"

// 中断处理函数
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    encoder_t *encoder = (encoder_t *)arg;
    int state_a = gpio_get_level(encoder->gpio_a);
    int state_b = gpio_get_level(encoder->gpio_b);
    int current_state = (state_a << 1) | state_b;

    if (current_state != encoder->last_state) {
        if ((encoder->last_state == 0b00 && current_state == 0b01) ||
            (encoder->last_state == 0b01 && current_state == 0b11) ||
            (encoder->last_state == 0b11 && current_state == 0b10) ||
            (encoder->last_state == 0b10 && current_state == 0b00)) {
            encoder->count++;
        } else if ((encoder->last_state == 0b00 && current_state == 0b10) ||
                   (encoder->last_state == 0b10 && current_state == 0b11) ||
                   (encoder->last_state == 0b11 && current_state == 0b01) ||
                   (encoder->last_state == 0b01 && current_state == 0b00)) {
            encoder->count--;
        }
    }

    encoder->last_state = current_state;
}

void encoder_init(encoder_t *encoder, int gpio_a, int gpio_b) {
    encoder->gpio_a = gpio_a;
    encoder->gpio_b = gpio_b;
    encoder->count = 0;
    encoder->last_state = 0;
    encoder->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    // 配置 GPIO 引脚
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio_a) | (1ULL << gpio_b),
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // 安装中断处理程序
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK) {
        printf("Failed to install ISR service: %s\n", esp_err_to_name(err));
    }

    gpio_isr_handler_add(gpio_a, encoder_isr_handler, (void*) encoder);
    gpio_isr_handler_add(gpio_b, encoder_isr_handler, (void*) encoder);
    printf("Encoder initialized on GPIO %d and %d\n", gpio_a, gpio_b);
}

int get_encoder_val(encoder_t *encoder) {
    taskENTER_CRITICAL(&encoder->mux);
    int ret = encoder->count;
    encoder->count = 0;
    taskEXIT_CRITICAL(&encoder->mux);
    return ret;
}