#ifndef GPIO_H
#define GPIO_H

#include "driver/gpio.h"

void gpio_init(gpio_num_t gpio);

void led_on(void);

void led_off(void);

#endif // GPIO_H
