# 两轮差速式自主移动机器人底盘系统
这是一套面向室内场景的两轮差速自主移动机器人底盘系统，硬件采用ESP32-WROOM-32E 主控，以 YDLIDAR X2 激光雷达为主感知，搭配 MPU6050 IMU、HC-SR04 超声波模块实现多源感知冗余；软件基于 micro-ROS + ROS2 Humble 分层架构设计，实现了遥控运动、编码器里程计解算、SLAM 建图、AMCL 全局定位、自主路径规划导航、看门狗自恢复等功能。

## 项目目录结构
```
Project: diff‑amr‑base
├─ros2_ws                     # ROS2上位机工作空间
│   └─src                     # 上位机ROS2自研功能包
│       ├─xsmrobot_bringup    # 整机启动launch文件
│       ├─xsmrobot_description# URDF机器人模型、TF坐标配置
│       └─xsmrobot_nav        # Nav2导航参数yaml
├─esp32_firmware              # ESP32底盘固件PlatformIO工程
│   ├─lib                     # 自定义业务库：PID控制器、运动学解算、传感器驱动
│   ├─src
│   │   └─main.cpp            # 下位机程序主入口
│   ├─include                 # 头文件
│   └─platformio.ini          # PlatformIO编译配置
├─docs                        # 项目方案、硬件BOM、引脚分配、调试故障记录
└─README.md
```

### 目录说明
1. **ros2_ws/src**：上位机ROS功能包，build、install、log编译产物不上传仓库。
第三方驱动 ydlidar_ros2_driver不纳入本仓库，需要自行git clone到src目录编译。

2. **esp32_firmware 下位机固件工程**
- `lib/`：自己封装业务库，包含电机控制、PID、运动学、传感器相关代码
- `src/main.cpp`：程序入口，包含FreeRTOS任务：micro‑ROS通信任务、底盘控制loop、监控看门狗任务
- `.pio`为PlatformIO编译缓存，由gitignore过滤不上传。

## 核心功能
### 下位机 ESP32（FreeRTOS）
1. 双电机MCPWM驱动，PID速度闭环，采用限幅消积分抗积分饱和。
2. PCNT硬件编码器采集脉冲，做逆运动学解算，完成里程计积分。
3. MPU6050 IMU与编码器航向互补滤波融合，减小轮子打滑带来航向漂移。
4. HC‑SR04超声波底层完成避障逻辑，阈值做回退防抖，仅限制前进，允许后退与原地旋转。
5. FreeRTOS多任务调度：底盘控制任务、看门狗监控任务，网络异常不干扰电机实时控制。
6. micro‑ROS客户端，接收Twist速度指令，发布odom里程计数据。
7. 软件看门狗检测，异常状态下执行告警与紧急停车。

### 上位机（Ubuntu22.04 ROS2‑Humble）
1. URDF机器人模型，完整TF坐标树 map → odom → base_footprint → base_link
2. 健康监控与故障运动控制
3. slam‑toolbox 异步在线SLAM二维栅格地图构建
4. AMCL自适应蒙特卡洛定位，补偿里程计累计漂移
5. Nav2导航栈：A*全局规划、DWA局部规划、多层代价地图，支持目标点自主导航

## 外部依赖
1. ROS2‑Humble 环境
2. micro‑ROS‑Agent(UDP)
3. slam‑toolbox、nav2导航相关功能包
4. ydlidar_ros2_driver雷达驱动
5. PlatformIO，用于ESP32固件编译烧录

## 编译运行
### 上位机 ROS2
```bash
cd ros2_ws
colcon build
source install/setup.bash

ros2 launch xsmrobot_bringup bringup.launch.py
```

### 下位机 ESP32 PlatformIO
1. 修改platformio.ini，填写WiFi账号密码、micro‑ROS‑Agent主机IP
2. VSCode打开esp32_firmware工程，编译烧录至ESP32。

## 典型问题记录
1. 里程计数据为0：缺少运动学正解，轮速未转换为车体速度做积分。
2. /cmd_vel单位不匹配：ROS单位m/s，底盘内部mm/s，回调接口做单位转换。
3. 里程计单位错误：毫米直接参与米制坐标系积分，SLAM丢弃激光数据。
4. 超声波频繁振荡：采用双阈值滞回，区分触发阈值与解除阈值。
5. PID输出截断引发积分饱和：使用遇限削弱积分算法处理。
