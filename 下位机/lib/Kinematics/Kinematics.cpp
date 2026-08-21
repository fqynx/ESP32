#include "Kinematics.h"

// 设置电机参数，传入电机序号，单个脉冲对应轮子前进距离
void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance)
{
    motor_param[id].per_pulse_distance = per_pulse_distance;
}

// 设置轮距
void Kinematics::set_wheel_distance(float wheel_distance)
{
    this->wheel_distance = wheel_distance;
}

// 获取当前速度
int16_t Kinematics::get_motor_speed(uint8_t id)
{
    if(id < 0 | id > 1)
    {
        return -1;
    }

    return motor_param[id].motor_speed;
}

// 更新电机速度 mm/s。传入当前时间，左右轮脉冲数
void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick)
{
    // 计算积分时间，更新上次更新时间
    uint32_t dt = current_time - last_update_time;
    last_update_time = current_time;

    int32_t dtick1 = left_tick - motor_param[0].last_encoder_tick;
    int32_t dtick2 = right_tick - motor_param[1].last_encoder_tick;
    motor_param[0].last_encoder_tick = left_tick;
    motor_param[1].last_encoder_tick = right_tick;

    motor_param[0].motor_speed = float(dtick1 * motor_param[0].per_pulse_distance) / dt * 1000;
    motor_param[1].motor_speed = float(dtick2 * motor_param[1].per_pulse_distance) / dt * 1000;
}

// 运动学正解，传入左右轮速度，转化为角速度、线速度
void Kinematics::kinematic_forward(float left_speed, float right_speed, float *out_linear_speed, float *out_angle_speed)
{
    *out_linear_speed = (right_speed + left_speed) / 2.0;
    *out_angle_speed = (right_speed - left_speed)  / wheel_distance;
}

// 运动学逆解，传入角速度、线速度，转化为左右轮速度
void Kinematics::kinematic_inverse(float linear_speed, float angle_speed, float *out_left_speed, float *out_right_speed)
{
    *out_left_speed = linear_speed - (angle_speed * wheel_distance)  / 2.0;
    *out_right_speed = linear_speed + (angle_speed * wheel_distance) / 2.0;
}

// 更新里程计
void Kinematics::update_odom(uint16_t dt)
{
    // 先正解更新当前线速度、角速度
    kinematic_forward(motor_param[0].motor_speed, motor_param[1].motor_speed, &odom.linear_speed, &odom.angle_speed);
    // 计算移动距离、转动角度 积分
    float delta_d = odom.linear_speed * dt / 1000000.0;    // 线速度 mm/s * ms / 1000 / 1000 = m
    float delta_theta = odom.angle_speed * dt / 1000.0; // 角速度 rad/s
    // 更新航向角，X、Y坐标
    odom.angle += delta_theta;
    TransAngleInPI(odom.angle, odom.angle);
    odom.x += delta_d * cos(odom.angle);
    odom.y += delta_d * sin(odom.angle);
}

odom_t &Kinematics::get_odom()
{
    return odom;
}

void Kinematics::TransAngleInPI(float angle, float &cut_angle)
{
    cut_angle = angle - 2 * PI * floor((angle + PI) / (2 * PI));
}
