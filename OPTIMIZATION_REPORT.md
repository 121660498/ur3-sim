# UR3 Z轴阻抗控制 —— 调试与优化过程报告

> 项目路径：`/workspaces/ur3_sim`  
> 报告时间：2026-06-08  
> 环境：ROS Noetic · Python 3.8 · Gazebo 仿真

---

## 一、项目背景

目标：实现 UR3 机械臂在 Gazebo 仿真中的 Z 轴阻抗（柔顺）控制，并自动记录数据和生成结果曲线图。

核心思路是混合力/位置控制（Hybrid Force-Position Control）：
- Z 轴：力控制，跟踪目标接触力（-2N、-5N、-8N）
- X/Y 轴及三个旋转轴：位置控制，保持初始姿态不变

---

## 二、代码 Bug 修复记录

### Bug 1：`IndexError: invalid index to scalar variable`

**文件**：`src/ur3/ur_control/src/ur_control/math_utils.py`

**根本原因**：`integrateUnitQuaternionEuler()` 先将四元数数组转换为 `np.quaternion` 对象，然后再次调用 `to_np_quaternion()` 试图对已经是 quaternion 标量的对象做索引，导致 `IndexError`。

调用链：
```
pose_from_angular_velocity()
  └─ integrateUnitQuaternionEuler(orientation, ang_vel, dt)
       └─ to_np_quaternion(q)  ← q 已是 np.quaternion，再做 q[3] 崩溃
```

**修复**：在 `to_np_quaternion()` 添加类型检查，如果输入已经是 `np.quaternion` 则直接返回。

```python
# 修复前
def to_np_quaternion(q: np.ndarray) -> np.quaternion:
    return np.quaternion(q[3], q[0], q[1], q[2])

# 修复后
def to_np_quaternion(q) -> np.quaternion:
    if isinstance(q, np.quaternion):
        return q
    return np.quaternion(q[3], q[0], q[1], q[2])
```

---

### Bug 2：`AttributeError: 'quaternion.quaternion' object has no attribute 'normalised'`

**文件**：`src/ur3/ur_control/src/ur_control/transformations.py`

**根本原因**：两个拼写错误叠加：
1. 英式拼写 `.normalised` 应为美式拼写 `.normalized`
2. `.normalized` 是**方法**而非属性，需要加括号调用

**修复**：

```python
# 修复前
return (q + 0.5*qw*dt*q).normalised

# 修复后
return (q + 0.5*qw*dt*q).normalized()
```

---

### Bug 3：`AttributeError: 'builtin_function_or_method' object has no attribute 'x'`

**文件**：`src/ur3/ur_control/src/ur_control/math_utils.py`

**根本原因**：Bug 2 未加括号时，`.normalized` 返回的是方法对象本身，该对象被传入 `to_np_array()`，对方法对象访问 `.x` 属性自然失败。修复 Bug 2（加括号）后，`to_np_array()` 恢复正常工作。

---

### Bug 4：`InverseKinematicsException` 未捕获导致程序崩溃

**文件**：`src/ur3/ur_control/src/ur_control/compliance_controller.py`

**根本原因**：`_actuate()` 方法仅处理 `inverse_kinematics()` 返回 `None` 的情况，但当目标位姿超出机械臂可达工作空间时，该函数会抛出 `InverseKinematicsException`，导致程序异常终止。

**修复**：用 `try/except` 捕获异常并将其转换为 `IK_NOT_FOUND` 返回码。

```python
# 修复前
q = self.inverse_kinematics(pose, attempts=0, verbose=False)

# 修复后
try:
    q = self.inverse_kinematics(pose, attempts=0, verbose=False)
except InverseKinematicsException:
    q = None
```

---

## 三、控制参数优化记录

### 优化 1：速度输出限制（Velocity Clamping）

**文件**：`src/ur3/ur_control/src/ur_control/compliance_controller.py`

**问题**：控制器计算出的速度指令高达 ±9000 deg/s，远超关节速度限制（100 deg/s），导致几乎所有控制指令都被丢弃（单次运行丢弃 200+ 次）。

**原因**：FT 传感器偏置补偿后的残余力仍有 100~300 N，乘以力 PID 增益 0.05 后产生 5~15 m/s 的速度指令。

**修复**：在控制循环中对速度输出进行硬限制：

```python
dxf = self.model.control_position_orientation(Fb, xb)

# 新增：速度输出限制
dxf[:3] = np.clip(dxf[:3], -0.1, 0.1)   # 线速度 ±0.1 m/s
dxf[3:] = np.clip(dxf[3:], -1.0, 1.0)   # 角速度 ±1.0 rad/s

xc = transformations.pose_from_angular_velocity(xb, dxf, dt=self.model.dt)
```

---

### 优化 2：力 PID 增益调整

**文件**：`src/ur3/ur_control/scripts/impedance_z_demo.py`

**问题**：力 PID 增益 `Kp_force = 0.05` 过大，对 Gazebo FT 传感器的大量噪声（±70 N）产生了过激响应。

**修改**：

| 参数 | 修改前 | 修改后 | 原因 |
|------|--------|--------|------|
| `Kp_force` | 0.05 | 0.001 | 100N 噪声 × 0.001 = 0.1 m/s，刚好在 clamp 范围内 |
| `Kp_pos` | 2.0 | 1.0 | 降低位置控制刚度，防止超调 |
| `dynamic_pid` | True | False | 关闭自适应增益，避免对噪声动态放大 |

---

### 优化 3：FT 偏置采样稳定性

**文件**：`src/ur3/ur_control/scripts/impedance_z_demo.py`

**问题**：每次阶段开始时采集的偏置值差异极大（从 -76N 到 +28N），说明采样窗口太短，受 Gazebo 传感器动态波动影响严重。

**修改**：

| 参数 | 修改前 | 修改后 |
|------|--------|--------|
| 采样前等待时间 | 1.5 s | 2.0 s |
| 采样点数 | 10 | 30 |
| 采样时间跨度 | 0.5 s | 1.5 s |

---

### 优化 4：初始关节角度调整

**文件**：`src/ur3/ur_control/scripts/impedance_z_demo.py`

**问题**：原始关节角 `[1.57, -1.57, 1.26, -1.57, -1.57, 0]` 接近奇异位形，IK 求解成功率低，力控制时容易偏离工作空间。

**修改**：

```python
# 修改前
[1.57, -1.57, 1.26, -1.57, -1.57, 0]

# 修改后
[1.57, -1.2, 1.0, -1.37, -1.57, 0]
```

---

### 优化 5：低通滤波器（方案二）

**文件**：`src/ur3/ur_control/scripts/impedance_z_demo.py`

**动机**：FT 传感器噪声标准差约 ±70 N，远大于目标力 2~8 N，控制器无法从噪声中提取有效的力误差信号。

**实现**：在 `BiasCompensatedController.get_wrench()` 中添加**滑动平均低通滤波器**：

```python
def get_wrench(self, base_frame_control=False, hand_frame_control=False):
    raw_wrench = super().get_wrench(...)
    compensated = raw_wrench - self.ft_bias  # 偏置补偿

    if self.filtering_enabled:
        self.ft_buffer.append(compensated)
        if len(self.ft_buffer) > self.filter_window_size:
            self.ft_buffer.pop(0)
        return np.asarray(np.mean(self.ft_buffer, axis=0), dtype=np.float64)
    else:
        return np.asarray(compensated, dtype=np.float64)
```

**参数**：窗口大小 100 点，控制循环 500 Hz → 约 0.2 秒时间平均。

同时将 `DataRecorder` 从直接订阅 `/wrench` 原始话题改为通过 `arm.get_wrench()` 以 50 Hz 定时采样，使记录数据与控制器实际使用的滤波信号一致。

---

## 四、实验结果对比

### 实验条件

- 目标力级别：-2 N、-5 N、-8 N（Z 轴方向，负值表示向下压）
- 每阶段持续时间：8 秒
- 初始 Z 轴高度：约 37~38 mm

### 量化对比

#### FT 力信号噪声（标准差）

| 阶段 | 滤波前 | 滤波后 | 降噪幅度 |
|------|--------|--------|----------|
| F = -2 N | ±70.8 N | ±17.8 N | **降低 75%** |
| F = -5 N | ±68.3 N | ±19.3 N | **降低 72%** |
| F = -8 N | ±55.5 N | ±13.0 N | **降低 77%** |

#### Z 轴净位移

| 阶段 | 滤波前 | 滤波后 |
|------|--------|--------|
| F = -2 N | 0.23 mm | 0.43 mm |
| F = -5 N | 0.04 mm | 0.16 mm |
| F = -8 N | 0.32 mm | -0.51 mm |

### 结果分析

**已改善的方面**：
- 程序不再崩溃，三个阶段均能完整运行到超时
- FT 信号噪声降低约 4 倍（从 ±70 N 降至 ±17 N）
- 速度指令不再超限（从 9000 deg/s 降至限制范围内）

**仍存在的根本问题**：
- 噪声（±17 N）相对于目标力（2~8 N）仍然过大，**信噪比约 0.1**，控制器无法可靠地感知目标力
- 各阶段的 FT 均值差异仍然很大（+75 N → -31 N → -5 N），说明静态偏置补偿不能消除 Gazebo 传感器的动态误差
- Z 轴位移没有体现出"力越大位移越大"的阻抗特性

---

## 五、根本原因分析

```
Gazebo FT 传感器输出 = 真实接触力 + 关节反作用力（手臂自重 + 惯性力）
                              ↑                      ↑
                          2~8 N（目标）          50~300 N（主导）
```

`libgazebo_ros_ft_sensor.so` 插件报告的是关节处的**总反作用力**，不仅包含接触力，还包含手臂自重产生的关节力矩分量。这个分量随机械臂姿态和运动状态**实时变化**，因此：

1. 每次采集的"偏置"本身不稳定，相同姿态下不同时刻的读数差异可达 100 N
2. 滑动平均滤波可以压制随机高频噪声，但无法消除这种低频漂移

---

## 六、后续优化建议

### 方案 A：位置柔顺控制模拟阻抗（推荐）

不依赖 FT 传感器，直接按阻抗公式 `F = K × Δx` 计算目标位移：

```python
K_stiffness = 1000.0  # N/m（可调刚度）
target_z_offset = target_force_z / K_stiffness
# -2 N → -2 mm，-5 N → -5 mm，-8 N → -8 mm
```

**优点**：结果确定可预测，曲线清晰，零噪声干扰；改动量最小。

### 方案 B：使用 Gazebo 接触力传感器

在 URDF 末端添加 `ContactSensor` 插件，只报告真实接触力，不包含关节内力。

**优点**：最接近真实传感器语义；**缺点**：需要修改 URDF，需要有物体与末端实际接触才能产生读数。

### 方案 C：力矩补偿（Gravity Compensation）

实现基于当前关节角度的重力补偿模型，从 FT 读数中实时减去手臂自重分量。

**优点**：最接近真实机器人的标准做法；**缺点**：需要精确的质量/惯量参数，实现复杂。

---

## 七、修改文件汇总

| 文件 | 修改类型 | 内容摘要 |
|------|----------|----------|
| `src/ur3/ur_control/src/ur_control/math_utils.py` | Bug 修复 | `to_np_quaternion()` 添加类型检查，防止重复转换 |
| `src/ur3/ur_control/src/ur_control/transformations.py` | Bug 修复 | `.normalised` 改为 `.normalized()` |
| `src/ur3/ur_control/src/ur_control/compliance_controller.py` | Bug 修复 + 优化 | 捕获 `InverseKinematicsException`；添加速度 clamp |
| `src/ur3/ur_control/src/ur_control/hybrid_controller.py` | Bug 修复 | `reset()` 中初始化 `target_force = np.zeros(6)` |
| `src/ur3/ur_control/scripts/impedance_z_demo.py` | 优化 | 降低 PID 增益；增加偏置采样；调整初始姿态；添加低通滤波器；改进 DataRecorder |
| `src/ur3/ur_gripper_85_moveit_config/launch/gazebo.launch` | Bug 修复（前任）| URDF xacro 解析修复 |

---

*报告生成时间：2026-06-08*
