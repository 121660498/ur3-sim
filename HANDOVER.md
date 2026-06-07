# UR3 Z轴阻抗控制项目 - 工作交接文档

> 生成时间: 2026-06-07  
> 项目路径: `/workspaces/ur3_sim`  
> 当前状态: 代码已完成，仍有运行时错误待解决

---

## 一、项目概述

### 目标
实现 UR3 机械臂在 Gazebo 仿真环境中的 Z 轴阻抗控制（柔顺控制），并自动生成实验数据和结果曲线图。

### 核心功能
1. **Z 轴力控制**：Z 方向施加目标力（-2N、-5N、-8N），其他方向保持位置控制
2. **FT 传感器偏置补偿**：自动采集并减去静态偏置（手臂自重产生的虚假力）
3. **数据记录**：实时记录力、位置、时间戳
4. **自动绘图**：运行结束后生成 4 子图的结果曲线（力/位移随时间变化）

---

## 二、已完成的工作

### 1. 修复的 Bug

#### Bug #1: URDF 加载错误（已修复 ✅）
**问题**: `robot_state_publisher` 无法解析 `${-pi/2.0}` 等 xacro 表达式

**修复位置**: `/workspaces/ur3_sim/src/ur3/ur_gripper_85_moveit_config/launch/gazebo.launch`

```xml
<!-- 修复前 -->
<param name="robot_description" textfile="$(arg urdf_path)" />

<!-- 修复后 -->
<param name="robot_description" command="$(find xacro)/xacro '$(arg urdf_path)'
    joint_limit_params:='$(find ur_description)/config/$(arg ur_robot)/joint_limits.yaml'
    kinematics_params:='$(find ur_description)/config/$(arg ur_robot)/default_kinematics.yaml'
    physical_params:='$(find ur_description)/config/$(arg ur_robot)/physical_parameters.yaml'
    visual_params:='$(find ur_description)/config/$(arg ur_robot)/visual_parameters.yaml'
    transmission_hw_interface:=$(arg transmission_hw_interface)
    safety_limits:=$(arg safety_limits)
    safety_pos_margin:=$(arg safety_pos_margin)
    safety_k_position:=$(arg safety_k_position)
    grasp_plugin:=$(arg grasp_plugin)" />
```

#### Bug #2: `target_force` 未初始化（已修复 ✅）
**问题**: `ForcePositionController.reset()` 里 `target_force` 未初始化为默认值，导致计算时 `-1 * None` 产生错误

**修复位置**: `/workspaces/ur3_sim/src/ur3/ur_control/src/ur_control/hybrid_controller.py`

```python
def reset(self):
    """ reset targets and PID params """
    self.qc = None
    self.target_position = None
    self.target_force = np.zeros(6)  # ← 添加这一行
    self.safety_mode = False
    self.position_pd.reset()
    self.force_pd.reset()
    self.error_data = list()
    self.update_data = list()
```

**说明**: 第 55 行已经添加了 `self.target_force = np.zeros(6)`

#### Bug #3: Matplotlib 颜色代码错误（已修复 ✅）
**问题**: `'#555'` 三位 hex 颜色在旧版 matplotlib 不支持

**修复**: 所有脚本中 `'#555'` 改为 `'#555555'`

#### Bug #4: FT 传感器偏置问题（已修复 ✅）
**问题**: Gazebo FT 传感器读取到手臂自重产生的几十到几百牛的"虚假力"，导致安全限制立即触发

**解决方案**:
1. 提高安全限制：`max_force_torque` 从 `[50, 50, 50]` 提高到 `[500, 500, 500]`
2. 创建 `BiasCompensatedController` 子类，自动减去静态偏置

---

### 2. 创建的核心文件

#### 主演示脚本
**文件**: `/workspaces/ur3_sim/src/ur3/ur_control/scripts/impedance_z_demo.py` (402 行)

**核心组件**:
- `BiasCompensatedController` 类：继承 `CompliantController`，重写 `get_wrench()` 自动减去偏置
- `DataRecorder` 类：后台订阅 `/wrench`，线程安全地记录数据
- `run_z_impedance_control()` 函数：执行单次力控制阶段
- `plot_results()` 函数：生成 4 子图结果曲线（全英文标签）

**运行方式**:
```bash
# 终端 1: 启动仿真
roslaunch ur_gripper_85_moveit_config demo_gazebo.launch

# 终端 2: 运行演示
source devel/setup.bash
python src/ur3/ur_control/scripts/impedance_z_demo.py
```

**输出**:
- `/workspaces/ur3_sim/results/impedance_data_YYYYMMDD_HHMMSS.csv` - 原始数据
- `/workspaces/ur3_sim/results/impedance_result_YYYYMMDD_HHMMSS.png` - 结果图表

#### 独立绘图脚本
**文件**: `/workspaces/ur3_sim/src/ur3/ur_control/scripts/plot_impedance_results.py` (217 行)

**用途**: 从历史 CSV 文件重新生成图表

```bash
python src/ur3/ur_control/scripts/plot_impedance_results.py ~/results/impedance_data_20260607_120000.csv
```

#### 使用文档
**文件**: `/workspaces/ur3_sim/README_impedance_demo.md` (240 行)

包含完整的使用说明、参数调整指南、故障排除等（中文）

---

## 三、当前问题 ⚠️

### 运行时错误（未解决）

**错误信息**:
```
IndexError: invalid index to scalar variable.
File "/workspaces/ur3_sim/src/ur3/ur_control/src/ur_control/math_utils.py", line 307, in to_np_quaternion
    return np.quaternion(q[3], q[0], q[1], q[2])
```

**错误堆栈**:
```
impedance_z_demo.py:200 → run_z_impedance_control()
compliance_controller.py:129 → set_hybrid_control_trajectory()
transformations.py:1602 → pose_from_angular_velocity()
transformations.py:1620 → integrateUnitQuaternionEuler()
math_utils.py:307 → to_np_quaternion() ← 这里崩溃
```

**症状**:
- FT 偏置已成功设置（显示 `[-56.3, -35.1, -3.5] N`）
- 错误发生在控制循环的第一次迭代
- `to_np_quaternion(q)` 期望 `q` 是长度为 4 的数组 `[x, y, z, w]`，但收到了标量

### 已尝试的调试步骤

1. ✅ 修复了 `target_force` 未初始化问题（添加 `np.zeros(6)` 到 `reset()`）
2. ✅ 删除了 Python 缓存文件 `__pycache__/hybrid_controller*.pyc`
3. ❌ 错误依然存在

### 问题分析

**根本原因推测**:
`control_position_orientation()` 返回的 `dxf`（速度向量，应为 6 维 `[vx, vy, vz, wx, wy, wz]`）可能因某种原因变成了标量或错误维度，导致 `pose_from_angular_velocity(xb, dxf, ...)` 里的四元数操作崩溃。

**可能的触发点**:
1. `spalg.translation_rotation_error()` 返回值异常
2. `np.dot(self.alpha, dxf_pos)` 矩阵乘法产生标量（`alpha` 是 6×6 矩阵）
3. `get_wrench()` 的偏置补偿返回了错误的数据类型

**建议的下一步调试**:
1. 在 `control_position_orientation()` 的返回语句前添加日志，打印 `dxf_pos`、`dxf_force`、返回值的 `shape`
2. 在 `compliance_controller.py:129` 行前打印 `xb.shape`、`dxf.shape`、`xb` 和 `dxf` 的实际值
3. 检查 `BiasCompensatedController.get_wrench()` 是否正确返回 `np.ndarray`

---

## 四、代码结构

### 关键类继承关系
```
Arm (arm.py)
  └─ CompliantController (compliance_controller.py)
       └─ BiasCompensatedController (impedance_z_demo.py)
```

### 控制器参数
```python
# 选择矩阵（0=力控，1=位控）
selection_matrix = [1.0, 1.0, 0.0, 1.0, 1.0, 1.0]
#                    X    Y    Z   Rx   Ry   Rz

# 位置 PID
Kp_pos = [2.0, 2.0, 2.0, 1.0, 1.0, 1.0]
Kd_pos = Kp_pos * 0.01
Ki_pos = Kp_pos * 0.0

# 力 PID
Kp_force = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
Kd_force = Kp_force * 0.0
Ki_force = Kp_force * 0.01

# 安全限制
max_force_torque = [500.0, 500.0, 500.0, 50.0, 50.0, 50.0]  # N 和 Nm
```

### 数据流
```
/wrench (WrenchStamped)
  → DataRecorder._wrench_cb()
  → records 列表
  → save_csv()
  → plot_results()
  → PNG 图表
```

---

## 五、环境信息

### 系统
- OS: Linux 6.8.0-117-generic
- ROS: Noetic
- Python: 3.8
- Gazebo: (版本未知)

### 关键依赖
- `ur_control` 包（自定义）
- `ur_pykdl` - UR 机器人的 PyKDL 运动学
- `matplotlib` - 绘图
- `numpy` - 数值计算
- `quaternion` - 四元数运算

### 工作空间
```
/workspaces/ur3_sim/
├── src/
│   ├── ur3/
│   │   ├── ur_control/
│   │   │   ├── scripts/
│   │   │   │   ├── impedance_z_demo.py ← 主程序
│   │   │   │   └── plot_impedance_results.py ← 绘图工具
│   │   │   └── src/ur_control/
│   │   │       ├── arm.py
│   │   │       ├── compliance_controller.py
│   │   │       ├── hybrid_controller.py ← 已修复 target_force
│   │   │       └── ...
│   │   ├── ur_gripper_85_moveit_config/
│   │   │   └── launch/
│   │   │       ├── demo_gazebo.launch ← 启动入口
│   │   │       └── gazebo.launch ← 已修复 xacro
│   │   └── ur_gripper_gazebo/
│   │       └── urdf/
│   │           └── ur_gripper_hande.xacro
│   └── ...
├── results/ ← 输出目录
└── README_impedance_demo.md ← 用户文档
```

---

## 六、快速参考

### 启动仿真
```bash
cd /workspaces/ur3_sim
roslaunch ur_gripper_85_moveit_config demo_gazebo.launch
```

### 运行演示
```bash
cd /workspaces/ur3_sim
source devel/setup.bash
python src/ur3/ur_control/scripts/impedance_z_demo.py
```

### 清除 Python 缓存（如果修改了 `ur_control` 包）
```bash
find /workspaces/ur3_sim/src/ur3/ur_control/src/ur_control/__pycache__ -name "*.pyc" -delete
```

### 查看结果
```bash
ls -lh /workspaces/ur3_sim/results/
```

---

## 七、下一步建议

### 立即需要做的
1. **定位 `dxf` 维度异常的根源**
   - 在 `hybrid_controller.py` 的 `control_position_orientation()` 末尾添加调试日志
   - 打印 `dxf_pos.shape`、`dxf_force.shape`、`(dxf_pos + dxf_force).shape`
   
2. **检查 `alpha` 矩阵乘法**
   - `self.alpha` 是 6×6 对角矩阵
   - `dxf_pos` 和 `dxf_force` 应该都是 6 维向量
   - `np.dot(self.alpha, dxf_pos)` 应该返回 6 维向量，如果返回标量说明输入有问题

3. **验证 `get_wrench` 返回类型**
   - 在 `BiasCompensatedController.get_wrench()` 里添加 `assert` 检查返回值的 `shape`

### 可选改进
1. 添加更详细的日志输出（当前已有基本日志）
2. 支持 X/Y 方向的阻抗控制（修改 `selection_matrix`）
3. 实现视频录制功能（录制 Gazebo 窗口）
4. 添加单元测试

---

## 八、重要提示

### 缓存问题
修改 `/workspaces/ur3_sim/src/ur3/ur_control/src/ur_control/` 下的 Python 文件后，**必须**删除 `__pycache__` 目录，否则 Python 会加载旧的 `.pyc` 文件。

### FT 传感器特性
Gazebo 的 `libgazebo_ros_ft_sensor.so` 插件**不支持重力补偿**，静止状态下会读到几十到几百牛的力（手臂自重产生的关节力矩）。`BiasCompensatedController` 通过软件方式减去静态偏置来补偿。

### 控制循环逻辑
`set_hybrid_control_trajectory()` 内部会再次调用 `self.model.set_goals(position=trajectory)`，但不传递 `force` 参数，所以外部设置的 `target_force` 会被保留。

---

## 九、联系方式与资源

### 已创建的文档
- `/workspaces/ur3_sim/README_impedance_demo.md` - 完整用户文档（中文）
- 本文档 - 开发者交接文档

### 代码注释
核心文件都有详细的 docstring，可以直接阅读：
- `impedance_z_demo.py` - 每个函数都有参数和返回值说明
- `BiasCompensatedController` 类 - 有完整的类文档

### 调试技巧
- 使用 `rospy.loginfo()` 输出调试信息
- 查看终端输出：`/root/.cursor/projects/workspaces-ur3-sim/terminals/*.txt`
- ROS 日志：`~/.ros/log/`

---

## 十、总结

### 已完成 ✅
- URDF 加载修复
- `target_force` 初始化修复
- FT 偏置补偿实现
- 数据记录与自动绘图
- 全英文图表标签
- 完整的用户文档

### 待解决 ❌
- **运行时四元数错误** - `to_np_quaternion` 收到标量而非数组
- 需要深入调试 `control_position_orientation()` 的返回值

### 预期效果（修复后）
程序应该：
1. 成功启动并移动到初始位置
2. 采集 FT 偏置并开始力控制
3. 在 Z 方向产生柔性运动（位移几毫米）
4. 自动生成包含 4 个子图的结果曲线 PNG
5. 保存完整的时间序列数据到 CSV

---

**文档生成时间**: 2026-06-07  
**最后更新**: 修复 `hybrid_controller.py` 第 55 行，添加 `target_force = np.zeros(6)`  
**下一位开发者**: 请从"三、当前问题"部分开始，先解决四元数错误，然后测试完整流程
