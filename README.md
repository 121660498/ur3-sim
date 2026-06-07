# UR3 仿真环境快速上手指南

> 本文档从零开始，完整说明如何在 Windows 宿主机 + Docker 容器环境下运行 UR3 Z 轴阻抗控制演示程序。

---

## 第一步：修改 devcontainer 的显示 IP

在使用 Dev Container 打开项目之前，需要先把 `DISPLAY` 环境变量中的 IP 地址改成你 Windows 宿主机的实际 IP。

打开 `.devcontainer/devcontainer.json`，找到以下这行：

```json
"DISPLAY": "10.7.1.150:0.0"
```

把 `10.7.1.150` 替换成你 Windows 机器的 IP 地址。

查看 Windows IP 的方法：打开 PowerShell 或 CMD，运行：

```powershell
ipconfig
```

找到以太网或 Wi-Fi 适配器对应的 IPv4 地址（通常是 `192.168.x.x` 或 `10.x.x.x`）填入即可。

---

## 第二步：在 Windows 上启动 X Server

容器内的 GUI 程序（Gazebo、RViz）需要通过 X11 转发显示到 Windows 屏幕上，因此需要提前在 Windows 上启动 X Server。

推荐使用 **VcXsrv**（免费）：

1. 下载安装 [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. 启动 XLaunch，配置如下：
   - Display settings：选 "Multiple windows"
   - Display number：填 `0`
   - 勾选 "Disable access control"（重要，否则容器无法连接）
3. 点击 Finish，托盘区会出现 X 图标

> 每次重启 Windows 后都需要重新启动 VcXsrv。

---

## 第三步：用 Dev Container 打开项目

1. 用 Cursor 或 VS Code 打开项目文件夹 `ur3_sim`
2. 按 `Ctrl+Shift+P`，搜索并选择 `Dev Containers: Reopen in Container`
3. 等待容器启动完成（第一次需要拉取镜像，耐心等待）

容器启动后，左下角会显示 `>< Dev Container: ROS Noetic GPU Dev`，说明已成功进入容器环境。

> 下次打开同一工作区时，点击左下角图标选择 `Reopen in Container` 即可。

---

## 第四步：编译工作空间

进入容器后，打开终端（`Ctrl+` ` `），执行编译：

```bash
cd /workspaces/ur3_sim
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如果之前已经编译过，直接 `source devel/setup.bash` 即可，不需要重新编译。

如果编译报错缺少依赖，运行：

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-moveit ros-noetic-gazebo-ros-control \
  ros-noetic-gazebo-plugins ros-noetic-effort-controllers \
  ros-noetic-scaled-joint-trajectory-controller \
  ros-noetic-gripper-action-controller \
  ros-noetic-industrial-robot-status-interface \
  ros-noetic-speed-scaling-interface \
  ros-noetic-speed-scaling-state-controller \
  ros-noetic-pass-through-controllers \
  ros-noetic-ur-client-library ros-noetic-ur-msgs

python3 -m pip install numpy-quaternion pyquaternion
```

然后重新 `catkin_make`。

---

## 第五步：启动仿真环境（终端 1）

新开一个终端，启动 Gazebo 仿真 + MoveIt!：

```bash
cd /workspaces/ur3_sim
source devel/setup.bash
roslaunch ur_gripper_85_moveit_config demo_gazebo.launch
```

等待直到看到以下输出，说明仿真环境已就绪：

```
[ INFO] ... You can start planning now!
```

Gazebo 和 RViz 窗口会弹出（需要 Windows 上的 X Server 正在运行）。第一次启动 Gazebo 可能需要 30-60 秒加载模型资源。

---

## 第六步：运行 Z 轴阻抗控制演示（终端 2）

新开另一个终端，运行演示程序：

```bash
cd /workspaces/ur3_sim
source devel/setup.bash
rosrun ur_control impedance_z_demo.py
```

程序会自动执行以下流程：

1. 机械臂移动到初始位置
2. 采集力传感器偏置（补偿手臂自重）
3. 依次执行三级力控制测试：
   - 第 1 步：目标力 -2 N，持续 8 秒
   - 第 2 步：目标力 -5 N，持续 8 秒
   - 第 3 步：目标力 -8 N，持续 8 秒
4. 自动保存数据和图表到 `results/` 目录

终端输出示例：

```
[ INFO] 移动到初始位置...
[ INFO] 到达初始位置: [0.330 0.194 0.040]
[ INFO] 开始多级力控测试: [-2.0, -5.0, -8.0] N
[ INFO] >>> 第 1/3 步: F=-2.0N
[ INFO]   FT 偏置已设置: [57.0, -9.2, 123.5] N
[ INFO]   结果=ExecutionResult.DONE  Z位移=-0.309 mm
...
[ INFO] 结果图已保存: /workspaces/ur3_sim/results/impedance_result_YYYYMMDD_HHMMSS.png
```

---

## 查看结果

```bash
ls -lh /workspaces/ur3_sim/results/
```

输出目录中包含：

- `impedance_data_YYYYMMDD_HHMMSS.csv`：原始时间序列数据（力、位置、时间戳）
- `impedance_result_YYYYMMDD_HHMMSS.png`：4 子图结果曲线（Z 力、Z 位移、XY 位移、力矩）

如需从历史 CSV 重新绘图：

```bash
python src/ur3/ur_control/scripts/plot_impedance_results.py results/impedance_data_YYYYMMDD_HHMMSS.csv
```

---

## 常见问题排查

### Gazebo 窗口没有弹出

先在 Windows 宿主机的 PowerShell 中运行：

```powershell
# 确认 VcXsrv 已启动，然后在容器内执行：
```

```bash
export DISPLAY=你的Windows_IP:0.0
export QT_X11_NO_MITSHM=1
```

### Gazebo 启动卡住或闪退

上次可能没有正确退出，清理残留进程：

```bash
pkill -f gzserver; pkill -f gzclient; pkill -f rosmaster; pkill -f roslaunch
```

稍等几秒后重新启动。

### `source devel/setup.bash` 提示找不到文件

说明还没有编译，先执行：

```bash
cd /workspaces/ur3_sim
source /opt/ros/noetic/setup.bash
catkin_make
```

### 提示 `rosrun: command not found`

ROS 环境变量没有加载：

```bash
source /opt/ros/noetic/setup.bash
source /workspaces/ur3_sim/devel/setup.bash
```

### 修改了 Python 源码后演示行为没有变化

Python 字节码缓存可能是旧版本，清理后重试：

```bash
find /workspaces/ur3_sim/src/ur3/ur_control/src -name "*.pyc" -delete
find /workspaces/ur3_sim/src/ur3/ur_control/src -name "__pycache__" -type d -exec rm -rf {} +
```

---

## 快速命令速查

| 操作 | 命令 |
|------|------|
| 加载 ROS 环境 | `source /opt/ros/noetic/setup.bash` |
| 加载工作空间 | `source /workspaces/ur3_sim/devel/setup.bash` |
| 编译 | `cd /workspaces/ur3_sim && catkin_make` |
| 启动仿真 | `roslaunch ur_gripper_85_moveit_config demo_gazebo.launch` |
| 运行演示 | `rosrun ur_control impedance_z_demo.py` |
| 查看结果 | `ls -lh /workspaces/ur3_sim/results/` |
| 清理旧进程 | `pkill -f gzserver; pkill -f rosmaster` |
| 清理 Python 缓存 | `find src/ur3/ur_control/src -name "*.pyc" -delete` |
