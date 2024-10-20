#include "pid.h"

// 初始化 PID 控制器
void pid_ctrl_init(pid_ctrl_t *pid, float Kp, float Ki, float Kd, float setpoint)
{
    pid->kp = Kp;
    pid->ki = Ki;
    pid->kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

// 更新 PID 控制器并返回调整值
float pid_ctrl_update(pid_ctrl_t *pid, float measured_value)
{
	// printf("measured_value:%f\n", measured_value);
    float error = pid->setpoint - measured_value;
    pid->integral += error;
    float derivative = error - pid->last_error;
    pid->last_error = error;
    // printf("setpoint:%f measured_value:%f integral:%f\n", pid->setpoint, measured_value, pid->integral);
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}
