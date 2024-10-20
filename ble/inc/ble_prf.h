#ifndef _BLE_PRF_H_
#define _BLE_PRF_H_

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    HRS_IDX_NB,
};

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

void ble_gatt_init(void);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_PRF_H_ */
