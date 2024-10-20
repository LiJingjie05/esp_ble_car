#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_data_t;

void mpu6050_init(i2c_port_t i2c_num);
void mpu6050_read(i2c_port_t i2c_num, mpu6050_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
