# Sensor Controller - TurtleSim 传感器控制系统

## 项目描述

这个项目实现了一个基于传感器数据控制 TurtleSim 机器人的 ROS2 系统。传感器发布整数信号,控制器根据这些信号控制乌龟的移动。

## 系统架构

- **传感器节点** (`midterm_sensor_2`): 发布整数信号到 `/sensor2_signal` 话题
- **控制器节点** (`skhxec_controller`): 订阅传感器信号并发送速度命令到 TurtleSim
- **TurtleSim**: 接收速度命令并移动乌龟

## 控制逻辑

根据传感器发送的整数值,乌龟会执行不同的动作:

| 传感器值 | 动作     | 说明                       |
| -------- | -------- | -------------------------- |
| 负数     | 向前移动 | 线速度 = 1.0 m/s           |
| 正奇数   | 向右转   | 角速度 = -90 度 (π/2 弧度) |
| 正偶数   | 向左转   | 角速度 = 90 度 (π/2 弧度)  |

## 调查结果

### 任务 2: 传感器话题调查

- **话题名称**: `/sensor2_signal`
- **消息类型**: `std_msgs/msg/Int32`
- **发布频率**: 约每 1.2 秒一次

可以通过以下命令验证:

```bash
ros2 topic info /sensor2_signal
ros2 topic echo /sensor2_signal
```

## 安装和构建

### 1. 确保传感器包已安装

```bash
cd ~/ros2_ws/src
colcon build --packages-select midterm_sensor_2
```

### 2. 构建控制器包

```bash
cd ~/ros2_ws/src
colcon build --packages-select skhxec_controller
source install/setup.bash
```

## 使用方法

### 方法 1: 使用 Launch 文件 (推荐)

一次性启动所有节点:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch skhxec_controller full_system.launch.py
```

### 方法 2: 手动启动各个节点

在三个不同的终端中分别运行:

**终端 1 - 启动 TurtleSim:**

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

**终端 2 - 启动传感器节点:**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run midterm_sensor_2 sensor
```

**终端 3 - 启动控制器节点:**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run skhxec_controller turtle_controller
```

## 测试和验证

### 查看话题列表

```bash
ros2 topic list
```

应该看到:

- `/sensor2_signal` - 传感器数据
- `/turtle1/cmd_vel` - 速度命令

### 查看传感器数据

```bash
ros2 topic echo /sensor2_signal
```

### 查看速度命令

```bash
ros2 topic echo /turtle1/cmd_vel
```

### 查看节点图

```bash
rqt_graph
```

## 文件结构

```
skhxec_controller/
├── skhxec_controller/
│   ├── __init__.py
│   └── turtle_controller.py      # 主控制节点
├── launch/
│   └── full_system.launch.py     # Launch文件
├── resource/
│   └── skhxec_controller
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                     # 包配置文件
├── setup.py                        # Python包设置
├── setup.cfg                       # 配置文件
└── README.md                       # 本文件
```

## 实现细节

### turtle_controller.py

控制器节点的主要功能:

1. **订阅**: 订阅 `/sensor2_signal` 话题接收 Int32 消息
2. **发布**: 发布 Twist 消息到 `/turtle1/cmd_vel` 控制乌龟
3. **逻辑处理**:
   - 使用 `sensor_value < 0` 检测负数
   - 使用 `sensor_value % 2 == 1` 检测正奇数
   - 使用 `sensor_value % 2 == 0` 检测正偶数
4. **运动控制**:
   - 向前: `linear.x = 1.0`, `angular.z = 0.0`
   - 右转: `linear.x = 0.0`, `angular.z = -π/2`
   - 左转: `linear.x = 0.0`, `angular.z = π/2`

## 预期行为

运行系统后,你应该看到:

1. TurtleSim 窗口打开,显示乌龟
2. 传感器节点每 1.2 秒发布一个整数值
3. 控制器节点接收数据并记录动作
4. 乌龟根据传感器值按照预定逻辑移动

乌龟会按照以下模式移动:

- 前进 → 右转 → 前进 → 左转 → 前进 → 右转 → (循环)

## 依赖项

- ROS 2 Humble
- Python 3.10+
- rclpy
- std_msgs
- geometry_msgs
- turtlesim
