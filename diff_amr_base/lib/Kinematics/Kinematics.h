#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <Arduino.h>

// 电机参数结构体
typedef struct
{
    float per_pulse_distance;  // 单个脉冲对应轮子前进距离 mm
    int16_t motor_speed;       // 当前电动机速度 mm/s
    int64_t last_encoder_tick; // 上次电动机的编码器读数
} motor_param_t;

// 里程计结构体
typedef struct
{
    float x;            // X坐标，单位m
    float y;            // Y坐标，单位m
    float angle;        // 机器人航向角，单位rad
    float linear_speed; // 线速度，单位mm/s
    float angle_speed;  // 角速度，单位rad/s
} odom_t;

class Kinematics
{
public:
    Kinematics() = default;
    ~Kinematics() = default;

    // 设置电机参数，传入电机序号，单个脉冲对应轮子前进距离
    void set_motor_param(uint8_t id, float per_pulse_distance);
    // 设置轮距
    void set_wheel_distance(float wheel_distance);

    // 运动学逆解，传入角速度、线速度，转化为左右轮速度
    void kinematic_inverse(float linear_speed, float angle_speed,
                           float *out_left_speed, float *out_right_speed);
    // 运动学正解，传入左右轮速度，转化为角速度、线速度
    void kinematic_forward(float left_speed, float right_speed,
                           float *out_linear_speed, float *out_angle_speed);
    // 更新电机速度
    void update_motor_speed(uint64_t current_time, int32_t left_tick,
                            int32_t right_tick);
    // 获取当前速度
    int16_t get_motor_speed(uint8_t id);

    // 新增里程计相关方法
    void update_odom(uint16_t dt);
    odom_t &get_odom();
    static void TransAngleInPI(float angle, float &cut_angle);

private:
    motor_param_t motor_param[2];
    uint64_t last_update_time;
    float wheel_distance;
    odom_t odom; // 里程计信息
};

#endif
