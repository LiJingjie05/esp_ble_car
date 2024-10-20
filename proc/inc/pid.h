#ifndef _PID_H_
#define _PID_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float last_error;
}pid_ctrl_t;

void pid_ctrl_init(pid_ctrl_t *pid, float Kp, float Ki, float Kd, float setpoint);

float pid_ctrl_update(pid_ctrl_t *pid, float measured_value);


#ifdef __cplusplus
}
#endif

#endif /* _PID_H_ */
