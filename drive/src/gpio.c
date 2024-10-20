#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gpio.h"

static gpio_num_t init_gpio;

void gpio_init(gpio_num_t gpio) 
{
    init_gpio = gpio;
    gpio_reset_pin(init_gpio);
    gpio_set_direction(init_gpio, GPIO_MODE_OUTPUT);
    printf("GPIO initialized!\n");
}

void led_on(void)
{
    gpio_set_level(init_gpio, 1);
}

void led_off(void)
{
    gpio_set_level(init_gpio, 0);
}
