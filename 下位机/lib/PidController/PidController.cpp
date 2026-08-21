#include "PidController.h"
#include "Arduino.h"

// 构造函数，传入3个pid参数
PidController::PidController(float kp, float ki, float kd)
{
    reset();
    update_pid(kp, ki, kd);
}

// 传入速度，计算pid
float PidController::update(float current)
{
    // 计算误差及其变化率
    float error = target - current;
    derror = error_last - error;
    error_last = error;

    // 先计算比例+微分
    float pd_out = kp * error + kd * derror;

    // 只有输出没到限幅边界时，才累加积分
    float output = pd_out + ki * error_sum;
    if(output > out_min && output < out_max)
    {
        // 输出在正常范围内，才累加积分
        error_sum += error;
        if (error_sum > integral_up)
            error_sum = integral_up;
        if (error_sum < -integral_up)
            error_sum = -integral_up;
        // 重新计算完整输出
        output = pd_out + ki * error_sum;
    }

    // 控制输出限幅
    if (output > out_max)
        output = out_max;
    if (output < out_min)
        output = out_min;
    return output;
}

void PidController::update_target(float target)
{
    this->target = target;
}

void PidController::update_pid(float kp, float ki, float kd)
{
    reset();
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PidController::reset()
{
    target = 0.0f;
    out_min = 0.0f;
    out_max = 0.0f;
    kp = 0.0f;
    ki = 0.0f;
    kd = 0.0f;
    error_sum = 0.0f;
    derror = 0.0f;
    error_last = 0.0f;
}

void PidController::out_limit(float out_min, float out_max)
{
    this->out_min = out_min;
    this->out_max = out_max;
}