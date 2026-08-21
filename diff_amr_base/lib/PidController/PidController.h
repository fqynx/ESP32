#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

private:
    // pid参数
    float target;  // 目标值
    float out_min; // 输出下限
    float out_max; // 输出上限
    float kp;      // 比例系数
    float ki;      // 积分系数
    float kd;      // 微分系数
    // pid相关变量
    float error;
    float error_sum;          // 误差累积和
    float derror;             // 误差变化率
    float error_last;         // 上一次误差
    float integral_up = 2500; // 积分上限

public:
    float update(float current);                   // 提供当前值，返回下次输出值，也就是PID的结果
    float get_target(void) { return target; }      // 获取当前pid目标值
    void update_target(float target);              // 更新目标值
    void update_pid(float kp, float ki, float kd); // 更新PID参数
    void reset();                                  // 重置PID
    void out_limit(float out_min, float out_max);  // 设置输出限制
};

#endif
