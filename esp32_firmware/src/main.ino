#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <Kinematics.h>
#include <PidController.h>
#include <esp_task_wdt.h> // 看门狗
#include <Wire.h>
#include <MPU6050_light.h>

// Microros和wifi相关的库
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>              // 消息接口
#include <nav_msgs/msg/odometry.h>                // 里程计消息接口
#include <micro_ros_utilities/string_utilities.h> // 字符串内存分配初始化工具

// 超声波与安全标志
#define TRIG 27
#define ECHO 21
#define OBSTACLE_STOP_CM 10.0f     // 障碍物距离阈值，小于10cm触发紧急停车
#define OBSTACLE_RELEASE_CM 15.0f // 解除急停距离（滞回区间)
#define OBSTACLE_DEBOUNCE_MAX 10  // 最大防抖计数，1000ms确认障碍物
#define FORWARD_TRIG_SPEED 20.0f  // mm/s，大于这个前进速度才开启急停保护

// 新增：超声波滤波与LED告警参数
#define US_VALID_MIN_CM 2.0f   // 最小有效距离
#define US_VALID_MAX_CM 400.0f // 最大有效距离
#define LED_WARN_FAST_CM 15.0f // 小于该值快速闪烁
#define LED_WARN_SLOW_CM 30.0f // 小于该值慢速闪烁
#define LED_FAST_HALF 10       // 快闪半周期：10*10ms = 100ms
#define LED_SLOW_HALF 25       // 慢闪半周期：25*10ms = 250ms
#define PRINT_PERIOD 100       // 串口打印周期：100*10ms = 1000ms

#define COMPLEMENTARY_ALPHA 0.98f // 互补滤波系数：值越大越信任陀螺仪，越小越信任编码器

// 函数声明
void microros_task(void *args);
void monitor_task(void *args);

float read_ultrasonic(void);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void twist_callback(const void *msg_in);

// 任务相关的结构体对象
rcl_allocator_t allocator; // 内存分配器，管理micro‑ROS动态内存
rclc_support_t support;    // micro‑ROS上下文支撑（DDS、时钟、内存）
rclc_executor_t executor;  // 任务执行器，轮询所有ROS回调
rcl_node_t node;           // ESP32上的ROS2节点对象

rcl_subscription_t subscriber;     // 创建订阅者
geometry_msgs__msg__Twist sub_msg; // 订阅到的数据存储到这里

rcl_publisher_t odom_publisher;   // 创建一个里程计发布者
nav_msgs__msg__Odometry odom_msg; // 里程计消息存储到这
rcl_timer_t timer;                // 定时器

Esp32McpwmMotor motor;        // 创建一个名为motor的对象用于控制电机
Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PidController pid_controller[2];
Kinematics kinematics;

// 传感器数据互斥锁
SemaphoreHandle_t g_DataMutex;

MPU6050 mpu(Wire); // 实例化IMU，绑定I2C总线

bool g_emergency_stop = false;     // 全局紧急停车标志
uint8_t obstacle_debounce_cnt = 0; // 障碍物防抖计数器
uint8_t g_led_blink_cnt = 0;       // LED闪烁计数器
uint8_t g_print_cnt = 0;           // 串口打印计数器
uint32_t g_loop_heartbeat = 0;     // 底盘任务心跳计数，用于看门狗监控

float g_shared_dist = -1.0f;       // 共享的距离数据，仅由采集任务写入，底盘/监控任务只读
float g_imu_yaw = 0.0f;            // 共享IMU数据（采集任务写，底盘任务读）,融合后的航向角，单位rad
float g_imu_gyro_z = 0.0f;         // Z轴角速度，单位rad/s
float target_linear_speed = 50.0;  // 目标线速度 mm/s
float target_angular_speed = 0.1f; // 目标角速度 rad/s
float out_left_speed;
float out_right_speed;

void setup()
{
  // 初始化串口
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

  // 设置编码器
  encoders[0].init(0, 32, 33); // 初始化第一个编码器，使用GPIO 32和33连接
  encoders[1].init(1, 26, 25); // 初始化第二个编码器，使用GPIO 26和25连接

  motor.attachMotor(0, 22, 23); // 将电机0连接到引脚22和引脚23
  motor.attachMotor(1, 12, 13); // 将电机1连接到引脚12和引|脚13

  // 初始化 PID 控制器参数
  pid_controller[0].update_pid(0.625, 0.125, 0.0);
  pid_controller[1].update_pid(0.625, 0.125, 0.0);
  pid_controller[0].out_limit(-100, 100);
  pid_controller[1].out_limit(-100, 100);

  // 运动学正逆解参数
  kinematics.set_wheel_distance(175);
  kinematics.set_motor_param(0, 0.103394);
  kinematics.set_motor_param(1, 0.103394);

  // 引脚初始化
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // 初始熄灭（低电平点亮、高电平熄灭）
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);

  // 初始化I2C
  Wire.begin(18, 19);

  // 初始化MPU6050，0成功
  byte imu_status = mpu.begin();
  if (imu_status != 0)
  {
    Serial.println("MPU6050 init failed!");
  }
  else
  {
    // 计算陀螺仪零偏，上电后保持小车静止
    mpu.calcOffsets(true, true);
    Serial.println("MPU6050 init done");
  }

  // 初始化互斥锁
  g_DataMutex = xSemaphoreCreateMutex();
  if (g_DataMutex == NULL)
  {
    Serial.println("Mutex create failed!");
  }

  // 创建传感器采集任务（优先级1，和底盘同级）
  xTaskCreate(sample_task, "sample", 4096, NULL, 1, NULL);

  // micro-R0S通信任务
  xTaskCreate(microros_task, "microros_task", 10240, NULL, 2, NULL);

  // 创建系统监控任务
  xTaskCreate(monitor_task, "monitor_task", 4096, NULL, 3, NULL);
}

// 底盘控制
void loop()
{
  delay(10); // 等待10毫秒

  g_loop_heartbeat++;
  g_led_blink_cnt++;
  g_print_cnt++;

  // 读取超声波距离数据
  float dist = -1.0f;
  if (xSemaphoreTake(g_DataMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    dist = g_shared_dist;
    xSemaphoreGive(g_DataMutex);
  }

  // 获取里程计
  odom_t odom = kinematics.get_odom();
  float chassis_linear_speed = odom.linear_speed; // 底盘当前线速度 mm/s

  // 障碍物防抖 + 硬件安全兜底（双阈值滞回状态机）
  if(!g_emergency_stop)
  {
      // 未触发状态：只有向前行驶+距离足够近，才触发保护
      bool enable_check = (chassis_linear_speed > FORWARD_TRIG_SPEED);
      if(enable_check && dist > 0 && dist < OBSTACLE_STOP_CM)
      {
          obstacle_debounce_cnt++;
          if(obstacle_debounce_cnt > OBSTACLE_DEBOUNCE_MAX)
              obstacle_debounce_cnt = OBSTACLE_DEBOUNCE_MAX;

          if(obstacle_debounce_cnt >= OBSTACLE_DEBOUNCE_MAX)
              g_emergency_stop = true;
      }
      else
      {
          // 不满足触发条件，计数衰减
          if(obstacle_debounce_cnt > 0)
              obstacle_debounce_cnt--;
      }
  }
  else
  {
      // 已触发状态：只有距离足够远才解除，和车速无关
      if(dist > 0 && dist > OBSTACLE_RELEASE_CM)
      {
          obstacle_debounce_cnt--;
          if(obstacle_debounce_cnt <= 0)
          {
              obstacle_debounce_cnt = 0;
              g_emergency_stop = false;
          }
      }
  }

  // LED分层告警逻辑，无效或距离安全则常亮，距离中等慢速闪烁，距离近快速闪烁
  if (dist < 0)
  {
    digitalWrite(2, HIGH);
    g_led_blink_cnt = 0;
  }

  else if (dist < LED_WARN_FAST_CM)
  {
    if (g_led_blink_cnt >= LED_FAST_HALF)
    {
      g_led_blink_cnt = 0;
      digitalWrite(2, !digitalRead(2));
    }
  }

  else if (dist < LED_WARN_SLOW_CM)
  {
    if (g_led_blink_cnt >= LED_SLOW_HALF)
    {
      g_led_blink_cnt = 0;
      digitalWrite(2, !digitalRead(2));
    }
  }

  else
  {
    digitalWrite(2, HIGH);
    g_led_blink_cnt = 0;
  }

  // 串口打印（1000ms一次）
  if (g_print_cnt >= PRINT_PERIOD)
  {
    g_print_cnt = 0;
    Serial.printf("Dist: %6.1f cm | EmergencyStop: %s | sp:%.1f mm/s\n",
                  dist,
                  g_emergency_stop ? "TRIGGERED" : "normal",
                  chassis_linear_speed);
  }

  // 更新电机速度
  kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());

  // PID闭环计算
  float out0 = pid_controller[0].update(kinematics.get_motor_speed(0));
  float out1 = pid_controller[1].update(kinematics.get_motor_speed(1));

  // 急停：仅截断正向输出，不修改PID任何内部状态
  if (g_emergency_stop)
  {
    if (out0 > 0)
      out0 = 0;
    if (out1 > 0)
      out1 = 0;
  }

  motor.updateMotorSpeed(0, out0);
  motor.updateMotorSpeed(1, out1);

  // // PID闭环控制
  // motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0)));
  // motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1)));

  // 每 10ms 更新里程计
  kinematics.update_odom(10);

  // // 打印两个电机速度
  // Serial.printf("speed1=%f mm/s, speed2=%f mm/s\n", current_speed[0], current_speed[1]);
}

// ========== micro‑ROS通信任务 ==========
void microros_task(void *args)
{
  // 1.设置传输协议并延迟一段时间等待设置的完成
  IPAddress agent_ip;
  agent_ip.fromString("10.137.196.12");                               // 设置agent的IP地址
  set_microros_wifi_transports("xxx", "xsm19491001", agent_ip, 8888); // 设置传输协议
  delay(2000);                                                        // 等待2秒,等待WIFI连接

  // 2.初始化内存分配器
  allocator = rcl_get_default_allocator(); // 获取默认的内存分配器

  // 3.初始化支持
  rclc_support_init(&support, 0, NULL, &allocator); // 初始化支持

  // 4.初始化节点
  rclc_node_init_default(&node, "test_motion_control", "", &support); // 初始化节点

  // 5.初始化执行器
  unsigned int num_handles = 2;                                             // 订阅和计时器的数量,注意这是一个要改的参数
  rclc_executor_init(&executor, &support.context, num_handles, &allocator); // 初始化执行器

  // 初始化订阅者，并将其添加到执行器中
  rclc_subscription_init_best_effort(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

  // 初始化msg
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");

  // 初始化发布者和定时器
  rclc_publisher_init_best_effort(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
  rclc_executor_add_timer(&executor, &timer);

  // 时间同步
  while (!rmw_uros_epoch_synchronized())
  {
    rmw_uros_sync_session(1000);
    delay(10);
  }

  // 循环执行器
  rclc_executor_spin(&executor); // 循环执行器
}

// ========== 系统监控任务（看门狗+故障检测） ==========
void monitor_task(void *args)
{
  // 初始化任务看门狗：超时1秒，触发后自动重启系统
  esp_task_wdt_init(1, true);
  // 将当前监控任务注册到看门狗
  esp_task_wdt_add(NULL);

  uint32_t last_heartbeat = 0;
  uint8_t heartbeat_timeout_cnt = 0;

  for (;;)
  {
    // 1. 检测底盘控制任务心跳
    if (g_loop_heartbeat != last_heartbeat)
    {
      last_heartbeat = g_loop_heartbeat;
      heartbeat_timeout_cnt = 0;
    }
    else
    {
      heartbeat_timeout_cnt++;
    }

    // 2. 心跳正常才喂狗；连续1秒无更新则停止喂狗，触发看门狗复位
    if (heartbeat_timeout_cnt < 20)
    {
      esp_task_wdt_reset();
    }

    // 后续可扩展：电池电压检测、通信超时判断等
    vTaskDelay(pdMS_TO_TICKS(50)); // 固定50ms周期运行
  }
}

//  ========== 数据采集任务（采集超声波、电压、故障状态等数据，周期100ms） ==========
void sample_task(void *args)
{
  for (;;)
  {
    // Serial.println("sample running...");

    // 超声波采集
    float dist = read_ultrasonic();

    // IMU读取与航向融合
    mpu.update();                                // 读取I2C数据，更新姿态
    float gyro_z = mpu.getGyroZ() * DEG_TO_RAD;  // 角度转弧度
    float enc_yaw = kinematics.get_odom().angle; // 编码器里程计航向

    // 获取 dt
    static uint32_t last_time = 0;
    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0f;
    last_time = now;

    // 互补滤波：98%信任陀螺仪短期变化，2%信任编码器长期修正
    float fused_yaw = COMPLEMENTARY_ALPHA * (g_imu_yaw + gyro_z * dt) + (1 - COMPLEMENTARY_ALPHA) * enc_yaw;

    // 角度归一化到[-pi, pi]
    Kinematics::TransAngleInPI(fused_yaw, fused_yaw);

    // 更新共享数据
    if (xSemaphoreTake(g_DataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      g_shared_dist = dist;
      g_imu_yaw = fused_yaw;
      g_imu_gyro_z = gyro_z;
      xSemaphoreGive(g_DataMutex);
    }

    // 100ms采集一次，保证角速度积分精度
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// 超声波测距函数
float read_ultrasonic(void)
{
  static float last_valid_dist = -1.0f; // 静态局部变量：保留上一次有效值，仅函数内部可见

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // 超时30000us，对应约5.1m，避免长时间阻塞
  unsigned long duration = pulseIn(ECHO, HIGH, 30000);

  // 情况1：超时无回波，返回上一次有效值
  if (duration == 0)
  {
    return last_valid_dist;
  }

  float dist = duration * 0.0343f / 2.0f;

  // 情况2：超出有效量程，丢弃数据，返回上一次有效值
  if (dist < US_VALID_MIN_CM || dist > US_VALID_MAX_CM)
  {
    return last_valid_dist;
  }

  // 情况3：数据有效，更新缓存并返回
  last_valid_dist = dist;
  return dist;
}

// 定时器回调函数
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  // 完成里程计的发布
  odom_t odom = kinematics.get_odom();                                        // 获取当前的里程计
  int64_t stamp = rmw_uros_epoch_millis();                                    // 获取当前的时间 ms
  odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000);             // 秒数 ms/1000= s
  odom_msg.header.stamp.nanosec = static_cast<int32_t>((stamp % 1000) * 1e6); // 取余数得纳秒部分
  odom_msg.pose.pose.position.x = odom.x;
  odom_msg.pose.pose.position.y = odom.y;
  odom_msg.pose.pose.orientation.w = cos(odom.angle * 0.5);
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = sin(odom.angle * 0.5);
  odom_msg.twist.twist.linear.x = odom.linear_speed;
  odom_msg.twist.twist.angular.z = odom.angle_speed;

  // 发布里程计，把数据发出去
  if (rcl_publish(&odom_publisher, &odom_msg, NULL) != RCL_RET_OK)
  {
    Serial.println("error: odom pub failed!");
  }
}

// 速度指令回调函数
void twist_callback(const void *msg_in)
{
  // void*为任意类型指针。将收到的消息指针转换成 geometry_msgs__msg__Twist类型的指针
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;

  // 调用运动学逆解，线速度单位为 mm/s，乘 1000 转换为 m/s，输出左右轮速度
  target_linear_speed = twist_msg->linear.x * 1000;
  target_angular_speed = twist_msg->angular.z;

  kinematics.kinematic_inverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
  Serial.printf("OUT:left speed=%f, right speed=%f\n", out_left_speed, out_right_speed);

  pid_controller[0].update_target(out_left_speed);
  pid_controller[1].update_target(out_right_speed);
}
