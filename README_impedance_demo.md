# Z 轴阻抗控制演示程序

## 概述

本程序展示如何在 UR3 机械臂仿真环境中实现单方向（Z 轴）的阻抗控制效果。阻抗控制是一种柔顺控制方法，允许机械臂在特定方向上根据力反馈进行柔性响应，同时在其他方向保持位置控制。

## 控制原理

### 混合位置/力控制

程序使用混合位置/力控制器（Hybrid Position/Force Controller），通过选择矩阵（Selection Matrix）定义每个自由度的控制模式：

- **选择矩阵**: `[X, Y, Z, Rx, Ry, Rz] = [1.0, 1.0, 0.0, 1.0, 1.0, 1.0]`
  - `1.0` = 位置控制模式
  - `0.0` = 力控制模式
  
在本演示中：
- Z 方向：力控制（阻抗控制）
- X, Y, Rx, Ry, Rz 方向：位置控制

### 阻抗控制特性

阻抗控制模拟弹簧-质量-阻尼系统的行为：
- 机械臂会尝试在 Z 方向上施加目标力
- 当遇到阻力时，会产生柔性响应
- 在其他方向保持位置不变

## 文件说明

- `src/ur3/ur_control/scripts/impedance_z_demo.py`: Z 轴阻抗控制演示程序
- `src/ur3/ur_gripper_85_moveit_config/launch/demo_gazebo.launch`: Gazebo 仿真环境启动文件

## 运行步骤

### 1. 启动仿真环境

打开终端 1，启动 Gazebo 仿真和 MoveIt!：

```bash
cd /workspaces/ur3_sim
roslaunch ur_gripper_85_moveit_config demo_gazebo.launch
```

等待 Gazebo 和 RViz 完全启动（可能需要 10-30 秒）。

### 2. 赋予脚本执行权限

打开终端 2：

```bash
cd /workspaces/ur3_sim
chmod +x src/ur3/ur_control/scripts/impedance_z_demo.py
```

### 3. 运行阻抗控制演示

在终端 2 中运行：

```bash
rosrun ur_control impedance_z_demo.py
```

或者直接运行：

```bash
cd /workspaces/ur3_sim
python src/ur3/ur_control/scripts/impedance_z_demo.py
```

## 演示流程

程序会自动执行以下步骤：

1. **初始化**
   - 配置阻抗控制参数
   - 初始化力/位置混合控制器

2. **移动到初始位置**
   - 机械臂移动到预设的安全起始姿态

3. **多级力控制测试**（共 3 级）
   - 第 1 步：目标力 = -2.0 N（轻度下压）
   - 第 2 步：目标力 = -5.0 N（中等下压）
   - 第 3 步：目标力 = -8.0 N（较大下压）

每一步会运行约 8 秒，并在步骤之间等待 2 秒。

## 预期效果

### 在 RViz 中观察

- 机械臂会在 Z 方向上产生柔性运动
- X 和 Y 方向的位置保持不变
- 末端执行器会根据目标力进行微小的位移调整

### 终端输出

程序会实时输出：
- 当前控制步骤
- 目标力大小
- 实际位移量（XYZ 方向）
- Z 方向的位移（米 和 毫米）
- 最终测得的力

## 参数说明

### 控制器参数

在 `impedance_z_demo.py` 中定义：

```python
# 位置 PID 增益
Kp_pos = [2.0, 2.0, 2.0, 1.0, 1.0, 1.0]
Kd_pos = Kp_pos * 0.01
Ki_pos = Kp_pos * 0.0

# 力 PID 增益
Kp_force = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
Kd_force = Kp_force * 0.0
Ki_force = Kp_force * 0.01

# 选择矩阵（哪些方向是位置控制，哪些是力控制）
selection_matrix = [1.0, 1.0, 0.0, 1.0, 1.0, 1.0]

# 控制周期
dt = 0.02  # 20ms
```

### 安全参数

```python
# 最大允许力/力矩（防止过载）
max_force_torque = [50.0, 50.0, 50.0, 5.0, 5.0, 5.0]  # N 和 Nm

# 控制持续时间
duration = 8.0  # 秒
```

## 修改和扩展

### 改变控制方向

要实现 X 方向或 Y 方向的阻抗控制，修改选择矩阵：

```python
# X 方向力控制
selection_matrix = [0.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# Y 方向力控制
selection_matrix = [1.0, 0.0, 1.0, 1.0, 1.0, 1.0]

# XYZ 三个方向都力控制
selection_matrix = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
```

### 调整目标力

修改 `force_levels` 列表：

```python
# 更多力级别
force_levels = [-1.0, -2.0, -3.0, -5.0, -8.0, -10.0]

# 或单次测试
result = run_z_impedance_control(arm, target_force_z=-6.0, duration=10.0)
```

### 调整控制增益

如果响应太慢或太快，可以调整 PID 增益：

```python
# 更快响应（增大增益）
Kp_force = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# 更稳定但慢（减小增益）
Kp_force = [0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
```

## 故障排除

### 问题 1: 找不到 `ur_control` 包

**解决方案**：

```bash
cd /workspaces/ur3_sim
source devel/setup.bash
```

### 问题 2: 力传感器数据为零

**原因**: 仿真环境中可能没有配置力传感器。

**解决方案**: 确保使用的 URDF 文件包含力/力矩传感器配置。

### 问题 3: 机械臂不动

**检查**:
1. Gazebo 是否正常运行
2. 关节控制器是否已加载
3. 查看终端输出的错误信息

### 问题 4: IK 求解失败

**解决方案**: 调整初始位置或目标位置，确保在机械臂工作空间内。

## 相关文件

- `src/ur3/ur_control/src/ur_control/compliance_controller.py`: 柔顺控制器基类
- `src/ur3/ur_control/src/ur_control/hybrid_controller.py`: 混合位置/力控制器
- `src/ur3/ur_control/src/ur_control/impedance_control.py`: 阻抗控制模型
- `src/ur3/ur_control/scripts/compliance_controller_examples.py`: 其他柔顺控制示例

## 进一步学习

查看其他示例程序了解更多控制模式：

```bash
# 查看所有示例
ls src/ur3/ur_control/scripts/

# 运行其他示例
rosrun ur_control compliance_controller_examples.py --force
rosrun ur_control cartesian_compliance_controller_examples.py --free_drive
```

## 技术支持

如有问题，请查看：
1. ROS 日志: `~/.ros/log/`
2. Gazebo 日志
3. 项目文档: `src/readme_howto.md`

## 参考文献

- 阻抗控制理论: Hogan, N. (1985). "Impedance Control: An Approach to Manipulation"
- ROS 力控制: http://wiki.ros.org/ros_control
- UR 机器人文档: https://www.universal-robots.com/
