# UR3 Z轴阻抗控制 - 优化总结（中文）

> 完整报告见 `OPTIMIZATION_REPORT.md`

---

## 修复的代码Bug（共4个）

| Bug | 文件 | 症状 | 根因 |
|-----|------|------|------|
| 1 | `math_utils.py` | `IndexError: invalid index to scalar variable` | `to_np_quaternion()` 对已经是quaternion的对象再次索引 |
| 2 | `transformations.py` | `'quaternion.quaternion' object has no attribute 'normalised'` | 拼写错误 + `.normalized` 是方法需要加括号 |
| 3 | `math_utils.py` | `'builtin_function_or_method' object has no attribute 'x'` | Bug 2的连锁反应，修复Bug 2后自动解决 |
| 4 | `compliance_controller.py` | `InverseKinematicsException` 未捕获导致崩溃 | IK失败时抛出异常但未捕获 |

**结果**: 程序从完全无法运行到能稳定运行完整个实验流程。

---

## 实施的参数优化（共5项）

### 1. 速度输出限制
```python
dxf[:3] = np.clip(dxf[:3], -0.1, 0.1)   # 线速度 ±0.1 m/s
dxf[3:] = np.clip(dxf[3:], -1.0, 1.0)   # 角速度 ±1.0 rad/s
```
- **效果**: 速度指令从 9000 deg/s 降至限制范围内，超限次数从 200+ 降至个位数

### 2. PID增益调整
| 参数 | 原值 | 新值 | 原因 |
|------|------|------|------|
| `Kp_force` | 0.05 | 0.001 | FT噪声太大（±70N），需要极小增益 |
| `Kp_pos` | 2.0 | 1.0 | 降低位置刚度，避免超调 |

### 3. FT偏置采样增强
- 采样前等待: 1.5s → 2.0s
- 采样点数: 10 → 30
- 采样时间: 0.5s → 1.5s

### 4. 初始姿态调整
```python
# 原: [1.57, -1.57, 1.26, -1.57, -1.57, 0]  # 接近奇异位形
# 新: [1.57, -1.2, 1.0, -1.37, -1.57, 0]    # 更好的工作空间
```

### 5. 低通滤波器（方案二）
- 类型: 滑动平均滤波器
- 窗口: 100点（约0.2秒）
- 效果: 噪声标准差降低约 **75%**

---

## 实验结果对比

### FT力信号噪声（标准差）
```
              滤波前      滤波后      降低幅度
F = -2N     ±70.8 N    ±17.8 N       75%
F = -5N     ±68.3 N    ±19.3 N       72%
F = -8N     ±55.5 N    ±13.0 N       77%
```

### 程序稳定性
```
滤波前: 第1阶段力超限后停止，第2阶段完成，第3阶段IK失败崩溃
滤波后: 三个阶段均完整运行，无崩溃
```

### Z轴位移
```
              滤波前      滤波后
F = -2N      0.23 mm    0.43 mm
F = -5N      0.04 mm    0.16 mm
F = -8N      0.32 mm   -0.51 mm
```

**结论**: 位移仍然无明显规律，说明控制器还是在噪声中随机游走。

---

## 根本问题（未解决）

Gazebo的 `libgazebo_ros_ft_sensor.so` 是**关节力矩传感器**，报告的是：

```
FT读数 = 真实接触力 + 手臂自重产生的关节反作用力
           (2~8 N)          (50~300 N, 随姿态动态变化)
```

**核心矛盾**:
- 目标力: 2~8 N
- 滤波后噪声: ±17 N
- **信噪比 ≈ 0.1~0.5**（信号被噪声完全淹没）

静态偏置补偿只能消除固定部分，无法消除随运动变化的动态成分。

---

## 后续建议（三选一）

### 方案A: 位置柔顺控制（推荐）⭐
```python
K_stiffness = 1000  # N/m
delta_z = target_force / K_stiffness
# -2N → -2mm, -5N → -5mm, -8N → -8mm
```
- ✅ 改动最小（10行代码）
- ✅ 结果确定可预测
- ✅ 图表清晰美观
- ❌ 不是真正的力反馈控制

### 方案B: Gazebo接触力传感器
在URDF末端添加 `ContactSensor` 插件
- ✅ 只报告真实接触力
- ❌ 需要修改URDF
- ❌ 需要有物体接触才有读数

### 方案C: 重力/动力学补偿
基于关节角度实时计算手臂自重分量并减去
- ✅ 最接近真实机器人做法
- ❌ 需要精确质量参数
- ❌ 实现复杂

---

## 修改文件清单

1. `src/ur3/ur_control/src/ur_control/math_utils.py`
2. `src/ur3/ur_control/src/ur_control/transformations.py`
3. `src/ur3/ur_control/src/ur_control/compliance_controller.py`
4. `src/ur3/ur_control/src/ur_control/hybrid_controller.py`
5. `src/ur3/ur_control/scripts/impedance_z_demo.py`

所有修改已提交，程序可正常运行。

---

**报告生成时间**: 2026-06-08  
**完整英文报告**: `OPTIMIZATION_REPORT.md`  
**实验数据**: `results/impedance_data_*.csv`  
**结果图表**: `results/impedance_result_*.png`
