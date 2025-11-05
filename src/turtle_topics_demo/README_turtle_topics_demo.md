# turtle_topics_demo

一个用于期中考试热身与提交参考的 **ROS2(Humble)** 实践包：

- **Topics**：发布 `/turtle1/cmd_vel`（圆周运动），订阅 `/turtle1/pose` 与 `/turtle1/cmd_vel`；
- **Service**：对外提供 `/mission/reset`（`std_srvs/Trigger`），内部调用 turtlesim 的 `/reset`；
- **Action**：作为客户端调用 `/turtle1/rotate_absolute`（可带反馈）。
- **Launch**：`full_stack_demo.launch.py` 一键启动 `turtlesim + topics + service + action`。

本项目适配 **无桌面服务器 + Docker + VS Code Dev Container**，也支持 X11 GUI。

---

## 1. 目录结构

```
turtle_topics_demo/
├─ launch/
│  └─ full_stack_demo.launch.py     # 一键演示：Topics + Service + Action + turtlesim
├─ turtle_topics_demo/
│  ├─ __init__.py
│  ├─ vel_publisher.py              # 发布 /turtle1/cmd_vel（参数化 lin/ang/Hz）
│  ├─ pose_subscriber.py            # 订阅 /turtle1/pose 并打印位姿
│  ├─ cmdvel_subscriber.py          # 订阅 /turtle1/cmd_vel 并打印指令
│  ├─ reset_service.py              # 服务端：/mission/reset（内部调用 /reset）
│  └─ rotate_action_client.py       # 动作客户端：/turtle1/rotate_absolute
├─ package.xml
├─ setup.py                         # console_scripts 入口已配置
└─ setup.cfg
```

---

## 2. 依赖与环境

- ROS 2 Humble（镜像 `osrf/ros:humble-ros-base` 或等效发行版）
- 已安装软件包：`turtlesim`, `geometry_msgs`, `std_srvs`, `action_tutorials_interfaces`（若仅使用 turtlesim 的动作则不必）
- 建议在 `~/ros2_ws` 工作区内使用：
  ```bash
  mkdir -p ~/ros2_ws/src
  # 将本包放到 ~/ros2_ws/src 下
  ```

---

## 3. 构建

在工作区根目录：

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_topics_demo
source install/setup.bash
```

> 若你在 Dockerfile 中已配置自动 `source`，可忽略第二行。

**清理重构：**

```bash
rm -rf build/ install/ log/
colcon build --packages-select turtle_topics_demo
```

---

## 4. 快速启动（推荐方式）

```bash
# GUI + 全流程演示（支持调整参数）
ros2 launch turtle_topics_demo full_stack_demo.launch.py   linear_speed:=2.0 angular_speed:=1.2 Hz:=15.0 theta:=3.14
```

参数说明：

- `linear_speed`：线速度（默认 1.5）
- `angular_speed`：角速度（默认 1.0）
- `Hz`：发布频率（默认 10.0）
- `theta`：动作目标角度（弧度，默认 2.6）

---

## 5. 分步运行（可选）

```bash
# 1) 启动 turtlesim（GUI）
ros2 run turtlesim turtlesim_node

# 2) Topics：Publisher + 两个 Subscriber
ros2 run turtle_topics_demo vel_pub     --ros-args -p linear_speed:=2.0 -p angular_speed:=1.2 -p Hz:=15.0
ros2 run turtle_topics_demo pose_sub
ros2 run turtle_topics_demo cmdvel_echo

# 3) Service：/mission/reset（多次可稳定调用）
ros2 run turtle_topics_demo reset_srv
# 另开终端调用：
ros2 service call /mission/reset std_srvs/srv/Trigger "{}"

# 4) Action：旋转到指定角度
ros2 run turtle_topics_demo rot_client  --ros-args -p theta:=1.57
```

---

## 6. 接口一览（对照考试要求）

### Topics

| 角色 | 名称               | 类型                      | 说明                 |
| ---- | ------------------ | ------------------------- | -------------------- |
| Pub  | `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | 速度指令（圆周运动） |
| Sub  | `/turtle1/pose`    | `turtlesim/msg/Pose`      | 位姿回显             |
| Sub  | `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | 指令回显             |

### Service

| 角色   | 名称             | 类型                   | 说明                        |
| ------ | ---------------- | ---------------------- | --------------------------- |
| Server | `/mission/reset` | `std_srvs/srv/Trigger` | 对外服务，内部调用 `/reset` |
| Client | `/reset`         | `std_srvs/srv/Empty`   | turtlesim 自带              |

### Action

| 角色   | 名称                       | 类型                              | 说明                     |
| ------ | -------------------------- | --------------------------------- | ------------------------ |
| Client | `/turtle1/rotate_absolute` | `turtlesim/action/RotateAbsolute` | 旋转到绝对角度（带反馈） |

---

## 7. 快速验证命令

```bash
# 话题
ros2 topic list
ros2 topic info /turtle1/cmd_vel
ros2 topic echo /turtle1/pose
ros2 topic hz   /turtle1/pose

# 服务
ros2 service list | grep mission
ros2 service call /mission/reset std_srvs/srv/Trigger "{}"

# 动作
ros2 action list | grep rotate
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback

# 可视化
rqt_graph
```

- **GUI 无法显示**  
  通过 X11 转发或使用 `xvfb-run` 进行无头运行：  
  `xvfb-run -s "-screen 0 1024x768x24" ros2 run turtlesim turtlesim_node`
