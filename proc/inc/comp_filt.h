#ifndef _COMP_FILT_H_
#define _COMP_FILT_H_

#include <stdio.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

void complementary_filter_roll(float dt, float alpha, uint16_t acc_data[3], uint16_t gyro_data[3], float *pitch, float *roll);

#ifdef __cplusplus
}
#endif

#endif /* _COMP_FILT_H_ */
