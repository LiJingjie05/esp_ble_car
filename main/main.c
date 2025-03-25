#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "gpio.h"
#include "pwm.h"
#include "encoder.h"
#include "mpu6050.h"

#include "kalman_filter.h"
#include "pid.h"

#define I2C_MASTER_SCL_IO 9    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 8    // GPIO number for I2C master data
#define I2C_MASTER_NUM I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 400000 // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer

encoder_t encoder1;
encoder_t encoder2;

motor_t motor1;
motor_t motor2;

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}







#include <math.h>
// #include "kalman_filter.h"

// 定义卡尔曼滤波结构体
typedef struct {
    float angle;      // 估计角度
    float bias;       // 陀螺仪零偏
    float P[2][2];    // 协方差矩阵
    float Q_angle;    // 过程噪声（角度）
    float Q_bias;     // 过程噪声（零偏）
    float R_measure;  // 测量噪声
} KalmanFilter;

// 初始化卡尔曼滤波器
void kalman_init(KalmanFilter *kf, float Q_angle, float Q_bias, float R_measure) {
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;
    kf->angle = 0;
    kf->bias = 0;
    kf->P[0][0] = 1.0;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 1.0;
}

float g_pitch;
float g_roll;

// 卡尔曼滤波更新
float kalman_update(KalmanFilter *kf, float accel_angle, float gyro_rate, float dt) {
    // 预测步骤
    kf->angle += (gyro_rate - kf->bias) * dt;
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 计算卡尔曼增益
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // 更新状态
    float y = accel_angle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // 更新协方差矩阵
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

// 全局卡尔曼滤波器实例
KalmanFilter kalman_pitch, kalman_roll;

// 初始化滤波器
void init_kalman_filters() {
    // 参数需根据实际传感器噪声调整
    kalman_init(&kalman_pitch, 0.001, 0.003, 0.03);
    kalman_init(&kalman_roll, 0.001, 0.003, 0.03);
}

// 计算欧拉角（带卡尔曼滤波）
int calculate_euler_angles(mpu6050_data_t *data, float dt) {
    // printf("Raw Accel: X=%d, Y=%d, Z=%d\n", data->accel_x, data->accel_y, data->accel_z);
    // 加速度转换为g
    float ax = data->accel_x / 16384.0;
    float ay = data->accel_y / 16384.0;
    float az = data->accel_z / 16384.0;

    // 陀螺仪转换为度/秒
    float gx = data->gyro_x / 131.0;
    float gy = data->gyro_y / 131.0;

    // 加速度计计算瞬时角度
    float accel_pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180 / M_PI;
    float accel_roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

    // 卡尔曼滤波更新
    g_pitch = kalman_update(&kalman_pitch, accel_pitch, gx, dt);
    g_roll = kalman_update(&kalman_roll, accel_roll, gy, dt);

    return g_roll;
}



// pid
typedef struct {
    float Kp, Ki, Kd;       // PID参数
    float integral;         // 积分项
    float prev_error;       // 上一次误差
    float output;           // PID输出
    float max_output;       // 输出限制（防止过冲）
} PIDController;

PIDController pid_pitch; // 控制Pitch角度

void init_pid() {
    // 参数需根据实际硬件调整（示例值）
    pid_pitch.Kp = 10.0f;    // 比例系数
    pid_pitch.Ki = 0.01f;    // 积分系数
    pid_pitch.Kd = 1.5f;     // 微分系数
    pid_pitch.max_output = 255.0f; // PWM最大值（假设范围0-255）

    pid_pitch.integral = 0.0f;
    pid_pitch.prev_error = 0.0f;
}

float pid_update(PIDController *pid, float error, float dt) {
    pid->integral += error * dt;
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;

    float derivative = (error - pid->prev_error) / dt;
    float output = pid->Kp * error 
                + pid->Ki * pid->integral 
                + pid->Kd * derivative;

    if (output > pid->max_output) output = pid->max_output;
    if (output < -pid->max_output) output = -pid->max_output;

    pid->prev_error = error;
    return output;
}

void app_main()
{

    printf("Hello, ESP-IDF!\n");
    // gpio_init(10);

    mpu6050_data_t data;
    float dt = 0.01; // 采样时间间隔（与循环频率匹配）

    init_kalman_filters();
    init_pid();

    i2c_master_init();
    mpu6050_init(I2C_MASTER_NUM);

    encoder_init(&encoder1, 0, 1);
    encoder_init(&encoder2, 4, 5);

    motor_init(&motor1, 18, 19, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    motor_init(&motor2, 6, 7, LEDC_CHANNEL_2, LEDC_CHANNEL_3);

    while (1)
    {
        mpu6050_read(I2C_MASTER_NUM, &data);
        int pitch = calculate_euler_angles(&data, dt);

        float pitch_error = -3 - pitch;

        float pid_output = pid_update(&pid_pitch, pitch_error, dt);

        // 将PID输出限制在-255到255之间
        if (pid_output > 255.0f) pid_output = 255.0f;
        if (pid_output < -255.0f) pid_output = -255.0f;

        // 根据PID输出设置电机方向和PWM值
        uint8_t duty_a = 0;
        uint8_t duty_b = 0;

        if (pid_output > 0) {
            // 正转：duty_a=0，duty_b=PID输出
            duty_a = (uint8_t)pid_output;
        } else if (pid_output < 0) {
            // 反转：duty_b=0，duty_a=绝对值PID输出
            duty_b = (uint8_t)(-pid_output);
        }
        printf("Pitch: %.2f°, Roll: %.2f° ,duty_a: 0x%02x, duty_b: 0x%02x\n", g_pitch, g_roll, duty_a, duty_b);
        // 设置电机PWM（假设motor1控制左电机，motor2控制右电机）
        set_motor_pwm(&motor1, duty_a, duty_b);
        set_motor_pwm(&motor2, duty_a, duty_b); // 根据实际硬件调整左右电机逻辑
        vTaskDelay(10 / portTICK_PERIOD_MS); // 延迟 1 秒
    }
}















// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>


// /* Attributes State Machine */
// enum
// {
//     IDX_SVC,
//     IDX_CHAR_A,
//     IDX_CHAR_VAL_A,
//     IDX_CHAR_CFG_A,

//     IDX_CHAR_B,
//     IDX_CHAR_VAL_B,

//     IDX_CHAR_C,
//     IDX_CHAR_VAL_C,

//     HRS_IDX_NB,
// };

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "esp_system.h"
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "esp_bt.h"

// #include "esp_gap_ble_api.h"
// #include "esp_gatts_api.h"
// #include "esp_bt_main.h"
// #include "esp_gatt_common_api.h"

// #define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

// #define PROFILE_NUM                 1
// #define PROFILE_APP_IDX             0
// #define ESP_APP_ID                  0x55
// #define SAMPLE_DEVICE_NAME          "ESP_BLE_CAR"
// #define SVC_INST_ID                 0

// /* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
// *  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
// */
// #define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
// #define PREPARE_BUF_MAX_SIZE        1024
// #define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

// #define ADV_CONFIG_FLAG             (1 << 0)
// #define SCAN_RSP_CONFIG_FLAG        (1 << 1)

// static uint8_t adv_config_done       = 0;

// uint16_t heart_rate_handle_table[HRS_IDX_NB];

// typedef struct {
//     uint8_t                 *prepare_buf;
//     int                     prepare_len;
// } prepare_type_env_t;

// static prepare_type_env_t prepare_write_env;

// // #define CONFIG_SET_RAW_ADV_DATA
// #ifdef CONFIG_SET_RAW_ADV_DATA
// static uint8_t raw_adv_data[] = {
//         /* flags */
//         0x02, 0x01, 0x06,
//         /* tx power*/
//         0x02, 0x0a, 0xeb,
//         /* service uuid */
//         0x03, 0x03, 0xFF, 0x00,
//         /* device name */
//         0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D','E', 'M', 'O'
// };
// static uint8_t raw_scan_rsp_data[] = {
//         /* flags */
//         0x02, 0x01, 0x06,
//         /* tx power */
//         0x02, 0x0a, 0xeb,
//         /* service uuid */
//         0x03, 0x03, 0xFF,0x00
// };

// #else
// static uint8_t service_uuid[16] = {
//     /* LSB <--------------------------------------------------------------------------------> MSB */
//     //first uuid, 16bit, [12],[13] is the value
//     0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
// };

// /* The length of adv data must be less than 31 bytes */
// static esp_ble_adv_data_t adv_data = {
//     .set_scan_rsp        = false,
//     .include_name        = true,
//     .include_txpower     = true,
//     .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
//     .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
//     .appearance          = 0x00,
//     .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
//     .p_manufacturer_data = NULL, //test_manufacturer,
//     .service_data_len    = 0,
//     .p_service_data      = NULL,
//     .service_uuid_len    = sizeof(service_uuid),
//     .p_service_uuid      = service_uuid,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };

// // scan response data
// static esp_ble_adv_data_t scan_rsp_data = {
//     .set_scan_rsp        = true,
//     .include_name        = true,
//     .include_txpower     = true,
//     .min_interval        = 0x0006,
//     .max_interval        = 0x0010,
//     .appearance          = 0x00,
//     .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
//     .p_manufacturer_data = NULL, //&test_manufacturer[0],
//     .service_data_len    = 0,
//     .p_service_data      = NULL,
//     .service_uuid_len    = sizeof(service_uuid),
//     .p_service_uuid      = service_uuid,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };
// #endif /* CONFIG_SET_RAW_ADV_DATA */

// static esp_ble_adv_params_t adv_params = {
//     .adv_int_min         = 0x20,
//     .adv_int_max         = 0x40,
//     .adv_type            = ADV_TYPE_IND,
//     .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
//     .channel_map         = ADV_CHNL_ALL,
//     .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };

// struct gatts_profile_inst {
//     esp_gatts_cb_t gatts_cb;
//     uint16_t gatts_if;
//     uint16_t app_id;
//     uint16_t conn_id;
//     uint16_t service_handle;
//     esp_gatt_srvc_id_t service_id;
//     uint16_t char_handle;
//     esp_bt_uuid_t char_uuid;
//     esp_gatt_perm_t perm;
//     esp_gatt_char_prop_t property;
//     uint16_t descr_handle;
//     esp_bt_uuid_t descr_uuid;
// };

// static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
// 					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// /* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
// static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
//     [PROFILE_APP_IDX] = {
//         .gatts_cb = gatts_profile_event_handler,
//         .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     },
// };

// /* Service */
// static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x00FF;
// static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0xFF01;
// static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0xFF02;
// static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0xFF03;

// static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
// static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
// static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
// static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
// static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
// static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};


// /* Full Database Description - Used to add attributes into the database */
// static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
// {
//     // Service Declaration
//     [IDX_SVC]        =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
//       sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

//     /* Characteristic Declaration */
//     [IDX_CHAR_A]     =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
//       CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

//     /* Characteristic Value */
//     [IDX_CHAR_VAL_A] =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//       GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

//     /* Client Characteristic Configuration Descriptor */
//     [IDX_CHAR_CFG_A]  =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//       sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

//     /* Characteristic Declaration */
//     [IDX_CHAR_B]      =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
//       CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

//     /* Characteristic Value */
//     [IDX_CHAR_VAL_B]  =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//       GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

//     /* Characteristic Declaration */
//     [IDX_CHAR_C]      =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
//       CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

//     /* Characteristic Value */
//     [IDX_CHAR_VAL_C]  =
//     {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//       GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

// };

// static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
// {
//     switch (event) {
//     #ifdef CONFIG_SET_RAW_ADV_DATA
//         case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
//             adv_config_done &= (~ADV_CONFIG_FLAG);
//             if (adv_config_done == 0){
//                 esp_ble_gap_start_advertising(&adv_params);
//             }
//             break;
//         case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
//             adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
//             if (adv_config_done == 0){
//                 esp_ble_gap_start_advertising(&adv_params);
//             }
//             break;
//     #else
//         case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
//             adv_config_done &= (~ADV_CONFIG_FLAG);
//             if (adv_config_done == 0){
//                 esp_ble_gap_start_advertising(&adv_params);
//             }
//             break;
//         case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
//             adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
//             if (adv_config_done == 0){
//                 esp_ble_gap_start_advertising(&adv_params);
//             }
//             break;
//     #endif
//         case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
//             /* advertising start complete event to indicate advertising start successfully or failed */
//             if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
//             }else{
//                 ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
//             }
//             break;
//         case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
//             if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
//             }
//             else {
//                 ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
//             }
//             break;
//         case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
//             ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
//                   param->update_conn_params.status,
//                   param->update_conn_params.min_int,
//                   param->update_conn_params.max_int,
//                   param->update_conn_params.conn_int,
//                   param->update_conn_params.latency,
//                   param->update_conn_params.timeout);
//             break;
//         default:
//             break;
//     }
// }

// void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
// {
//     ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
//     esp_gatt_status_t status = ESP_GATT_OK;
//     if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
//         status = ESP_GATT_INVALID_OFFSET;
//     } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
//         status = ESP_GATT_INVALID_ATTR_LEN;
//     }
//     if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
//         prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
//         prepare_write_env->prepare_len = 0;
//         if (prepare_write_env->prepare_buf == NULL) {
//             ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
//             status = ESP_GATT_NO_RESOURCES;
//         }
//     }

//     /*send response when param->write.need_rsp is true */
//     if (param->write.need_rsp){
//         esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
//         if (gatt_rsp != NULL){
//             gatt_rsp->attr_value.len = param->write.len;
//             gatt_rsp->attr_value.handle = param->write.handle;
//             gatt_rsp->attr_value.offset = param->write.offset;
//             gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
//             memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
//             esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
//             if (response_err != ESP_OK) {
//                ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
//             }
//             free(gatt_rsp);
//         }else{
//             ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
//             status = ESP_GATT_NO_RESOURCES;
//         }
//     }
//     if (status != ESP_GATT_OK){
//         return;
//     }
//     memcpy(prepare_write_env->prepare_buf + param->write.offset,
//            param->write.value,
//            param->write.len);
//     prepare_write_env->prepare_len += param->write.len;

// }

// void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
//     if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
//         esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
//     }else{
//         ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
//     }
//     if (prepare_write_env->prepare_buf) {
//         free(prepare_write_env->prepare_buf);
//         prepare_write_env->prepare_buf = NULL;
//     }
//     prepare_write_env->prepare_len = 0;
// }

// static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {
//     switch (event) {
//         case ESP_GATTS_REG_EVT:{
//             esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
//             if (set_dev_name_ret){
//                 ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
//             }
//     #ifdef CONFIG_SET_RAW_ADV_DATA
//             esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
//             if (raw_adv_ret){
//                 ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
//             }
//             adv_config_done |= ADV_CONFIG_FLAG;
//             esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
//             if (raw_scan_ret){
//                 ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
//             }
//             adv_config_done |= SCAN_RSP_CONFIG_FLAG;
//     #else
//             //config adv data
//             esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
//             if (ret){
//                 ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
//             }
//             adv_config_done |= ADV_CONFIG_FLAG;
//             // config scan response data
//             // ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
//             // if (ret){
//             //     ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
//             // }
//             // adv_config_done |= SCAN_RSP_CONFIG_FLAG;
//     #endif
//             esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
//             if (create_attr_ret){
//                 ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
//             }
//         }
//        	    break;
//         case ESP_GATTS_READ_EVT:
//             ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
//        	    break;
//         case ESP_GATTS_WRITE_EVT:
//             if (!param->write.is_prep){
//                 // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
//                 ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
//                 esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
//                 if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
//                     uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
//                     if (descr_value == 0x0001){
//                         ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
//                         uint8_t notify_data[15];
//                         for (int i = 0; i < sizeof(notify_data); ++i)
//                         {
//                             notify_data[i] = i % 0xff;
//                         }
//                         //the size of notify_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
//                                                 sizeof(notify_data), notify_data, false);
//                     }else if (descr_value == 0x0002){
//                         ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
//                         uint8_t indicate_data[15];
//                         for (int i = 0; i < sizeof(indicate_data); ++i)
//                         {
//                             indicate_data[i] = i % 0xff;
//                         }

//                         // if want to change the value in server database, call:
//                         // esp_ble_gatts_set_attr_value(heart_rate_handle_table[IDX_CHAR_VAL_A], sizeof(indicate_data), indicate_data);


//                         //the size of indicate_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
//                                             sizeof(indicate_data), indicate_data, true);
//                     }
//                     else if (descr_value == 0x0000){
//                         ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
//                     }else{
//                         ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
//                         esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
//                     }

//                 }
//                 /* send response when param->write.need_rsp is true*/
//                 if (param->write.need_rsp){
//                     esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
//                 }
//             }else{
//                 /* handle prepare write */
//                 example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
//             }
//       	    break;
//         case ESP_GATTS_EXEC_WRITE_EVT:
//             // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
//             ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
//             example_exec_write_event_env(&prepare_write_env, param);
//             break;
//         case ESP_GATTS_MTU_EVT:
//             ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
//             break;
//         case ESP_GATTS_CONF_EVT:
//             ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
//             break;
//         case ESP_GATTS_START_EVT:
//             ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
//             break;
//         case ESP_GATTS_CONNECT_EVT:
//             ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
//             esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
//             esp_ble_conn_update_params_t conn_params = {0};
//             memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
//             /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
//             conn_params.latency = 0;
//             conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
//             conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
//             conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
//             //start sent the update connection parameters to the peer device.
//             esp_ble_gap_update_conn_params(&conn_params);
//             break;
//         case ESP_GATTS_DISCONNECT_EVT:
//             ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
//             esp_ble_gap_start_advertising(&adv_params);
//             break;
//         case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
//             if (param->add_attr_tab.status != ESP_GATT_OK){
//                 ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
//             }
//             else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
//                 ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) 
//                         doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
//             }
//             else {
//                 ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
//                 memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
//                 esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
//             }
//             break;
//         }
//         case ESP_GATTS_STOP_EVT:
//         case ESP_GATTS_OPEN_EVT:
//         case ESP_GATTS_CANCEL_OPEN_EVT:
//         case ESP_GATTS_CLOSE_EVT:
//         case ESP_GATTS_LISTEN_EVT:
//         case ESP_GATTS_CONGEST_EVT:
//         case ESP_GATTS_UNREG_EVT:
//         case ESP_GATTS_DELETE_EVT:
//         default:
//             break;
//     }
// }


// static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {

//     /* If event is register event, store the gatts_if for each profile */
//     if (event == ESP_GATTS_REG_EVT) {
//         if (param->reg.status == ESP_GATT_OK) {
//             heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
//         } else {
//             ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
//                     param->reg.app_id,
//                     param->reg.status);
//             return;
//         }
//     }
//     do {
//         int idx;
//         for (idx = 0; idx < PROFILE_NUM; idx++) {
//             /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
//             if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
//                 if (heart_rate_profile_tab[idx].gatts_cb) {
//                     heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
//                 }
//             }
//         }
//     } while (0);
// }

// void app_main(void)
// {
//     esp_err_t ret;

//     /* Initialize NVS. */
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK( ret );

//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ret = esp_bt_controller_init(&bt_cfg);
//     if (ret) {
//         ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     if (ret) {
//         ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_bluedroid_init();
//     if (ret) {
//         ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_bluedroid_enable();
//     if (ret) {
//         ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_ble_gatts_register_callback(gatts_event_handler);
//     if (ret){
//         ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
//         return;
//     }

//     ret = esp_ble_gap_register_callback(gap_event_handler);
//     if (ret){
//         ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
//         return;
//     }

//     ret = esp_ble_gatts_app_register(ESP_APP_ID);
//     if (ret){
//         ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
//         return;
//     }

//     esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
//     if (local_mtu_ret){
//         ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
//     }
// }




