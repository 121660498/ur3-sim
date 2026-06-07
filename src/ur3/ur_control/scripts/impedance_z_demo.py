#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Z轴阻抗控制演示程序（含数据记录与绘图）
运行方式:
  1. roslaunch ur_gripper_85_moveit_config demo_gazebo.launch
  2. source devel/setup.bash
  3. python src/ur3/ur_control/scripts/impedance_z_demo.py
"""

import sys
import os
import signal
import threading
import csv
from datetime import datetime

import rospy
import numpy as np
import matplotlib
matplotlib.use('Agg')   # 无显示器时也能保存图片
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

from ur_control import utils
from ur_control.compliance_controller import CompliantController
from ur_control.hybrid_controller import ForcePositionController

np.set_printoptions(suppress=True, precision=4)


# ---------------------------------------------------------------------------
# 带偏置补偿的柔顺控制器
# ---------------------------------------------------------------------------
class BiasCompensatedController(CompliantController):
    """自动进行 FT 传感器偏置补偿和低通滤波的柔顺控制器"""
    
    def __init__(self, *args, filter_window_size=100, **kwargs):
        super(BiasCompensatedController, self).__init__(*args, **kwargs)
        self.ft_bias = np.zeros(6)
        self.bias_compensation_enabled = False
        
        self.filter_window_size = filter_window_size
        self.ft_buffer = []
        self.filtering_enabled = False
    
    def set_ft_bias(self, bias):
        """设置 FT 传感器偏置"""
        self.ft_bias = np.array(bias)
        self.bias_compensation_enabled = True
        rospy.loginfo("  FT 偏置已设置: [%.1f, %.1f, %.1f] N", 
                     bias[0], bias[1], bias[2])
    
    def clear_ft_bias(self):
        """清除偏置补偿"""
        self.ft_bias = np.zeros(6)
        self.bias_compensation_enabled = False
        self.ft_buffer = []
        self.filtering_enabled = False
    
    def enable_filtering(self):
        """启用低通滤波（在偏置设置后调用）"""
        self.filtering_enabled = True
        self.ft_buffer = []
        rospy.loginfo("  低通滤波器已启用 (窗口大小: %d)", self.filter_window_size)
    
    def get_wrench(self, base_frame_control=False, hand_frame_control=False):
        """重写 get_wrench，自动减去偏置并进行低通滤波"""
        raw_wrench = super(BiasCompensatedController, self).get_wrench(
            base_frame_control=base_frame_control,
            hand_frame_control=hand_frame_control
        )
        
        if self.bias_compensation_enabled:
            compensated = raw_wrench - self.ft_bias
        else:
            compensated = raw_wrench
        
        if self.filtering_enabled:
            self.ft_buffer.append(compensated)
            
            if len(self.ft_buffer) > self.filter_window_size:
                self.ft_buffer.pop(0)
            
            smoothed = np.mean(self.ft_buffer, axis=0)
            return np.asarray(smoothed, dtype=np.float64)
        else:
            return np.asarray(compensated, dtype=np.float64)


# ---------------------------------------------------------------------------
# DataRecorder：后台订阅 wrench 和 ee_pose，线程安全地缓存数据
# ---------------------------------------------------------------------------
class DataRecorder:
    def __init__(self, arm):
        self.arm = arm
        self._lock = threading.Lock()
        self._recording = False

        # 每条记录: [t, fx, fy, fz, tx, ty, tz, x, y, z, phase_label]
        self.records = []
        self._phase_label = ""
        self._t0 = 0.0
        
        # 使用定时器周期性采样，记录滤波后的力数据（控制器实际看到的值）
        self._timer = None
        self._sample_rate = 50  # Hz

    def _sample_callback(self, event):
        """定时采样回调，记录滤波后的力数据"""
        if not self._recording:
            return
        
        t = rospy.get_time() - self._t0
        
        try:
            # 获取滤波后的力/力矩（控制器实际使用的值）
            wrench = self.arm.get_wrench(base_frame_control=True)
            fx, fy, fz = wrench[0], wrench[1], wrench[2]
            tx, ty, tz = wrench[3], wrench[4], wrench[5]
            
            # 获取当前末端位置
            pose = self.arm.end_effector()
            x, y, z = pose[0], pose[1], pose[2]
        except Exception as e:
            rospy.logwarn_once("DataRecorder采样失败: %s", e)
            return

        with self._lock:
            self.records.append([t, fx, fy, fz, tx, ty, tz, x, y, z, self._phase_label])

    def start(self, label=""):
        with self._lock:
            self._phase_label = label
            self._t0 = rospy.get_time()
            self._recording = True
        
        # 启动定时器进行周期性采样
        if self._timer is None:
            self._timer = rospy.Timer(rospy.Duration(1.0 / self._sample_rate), 
                                      self._sample_callback)

    def stop(self):
        with self._lock:
            self._recording = False

    def set_label(self, label):
        with self._lock:
            self._phase_label = label

    def save_csv(self, path):
        with self._lock:
            rows = list(self.records)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "fx", "fy", "fz",
                        "tx", "ty", "tz", "x", "y", "z", "phase"])
            w.writerows(rows)
        rospy.loginfo("数据已保存: %s  (%d 条)", path, len(rows))

    def get_arrays(self):
        with self._lock:
            rows = list(self.records)
        if not rows:
            return None
        arr = np.array([[r[0], r[1], r[2], r[3],
                         r[7], r[8], r[9]] for r in rows],
                       dtype=float)
        labels = [r[10] for r in rows]
        return arr, labels   # arr: [t, fx, fy, fz, x, y, z]


# ---------------------------------------------------------------------------
# 控制器初始化
# ---------------------------------------------------------------------------
def init_impedance_controller(selection_matrix, dt=0.02):
    # Position PID: moderate gains to hold XY/orientation while allowing Z compliance
    Kp_pos = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])
    position_pd = utils.PID(Kp=Kp_pos, Ki=Kp_pos * 0.0,
                            Kd=Kp_pos * 0.01, dynamic_pid=False)

    # Force PID: very small gain — Gazebo FT residuals are ~100-300 N even after bias
    # subtraction, so Kp=0.001 maps 100 N error → 0.1 m/s (within the clamp limit)
    Kp_force = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
    force_pd = utils.PID(Kp=Kp_force, Ki=Kp_force * 0.01, Kd=Kp_force * 0.0)

    return ForcePositionController(
        position_pd=position_pd,
        force_pd=force_pd,
        alpha=np.diag(selection_matrix),
        dt=dt
    )


def move_to_initial_pose(arm):
    rospy.loginfo("移动到初始位置...")
    # Adjusted joint angles to avoid singularities and provide better workspace
    # Positions robot with more vertical reach for Z-axis compliance
    arm.set_joint_positions(positions=[1.57, -1.2, 1.0, -1.37, -1.57, 0],
                            target_time=3.0, wait=True)
    rospy.sleep(1.0)
    rospy.loginfo("到达初始位置: %s", np.round(arm.end_effector()[:3], 4))


# ---------------------------------------------------------------------------
# 单次力控制阶段
# ---------------------------------------------------------------------------
def run_z_impedance_control(arm, recorder, target_force_z, duration, label):
    rospy.loginfo("--- %s  目标力=%.1f N  时长=%.1f s ---",
                  label, target_force_z, duration)

    arm.zero_ft_sensor()
    rospy.sleep(2.0)  # Longer wait for sensor to stabilize
    
    # 采集静态偏置（手臂自重产生的"虚假力"）
    # Use more samples over longer period for better stability
    ft_bias_samples = []
    for _ in range(30):  # 30 samples over 1.5 seconds
        ft_bias_samples.append(arm.get_wrench(base_frame_control=True))
        rospy.sleep(0.05)
    ft_bias = np.mean(ft_bias_samples, axis=0)
    
    # 启用偏置补偿和低通滤波
    arm.set_ft_bias(ft_bias)
    arm.enable_filtering()

    initial_pose = arm.end_effector()
    initial_pose = np.asarray(initial_pose, dtype=np.float64)

    target_force = np.array([0., 0., target_force_z, 0., 0., 0.], dtype=np.float64)
    max_ft = np.array([500., 500., 500., 50., 50., 50.], dtype=np.float64)

    arm.model.set_goals(position=initial_pose.copy(), force=target_force.copy())

    recorder.start(label=label)
    result = arm.set_hybrid_control_trajectory(
        initial_pose.copy(),
        max_force_torque=max_ft,
        timeout=duration,
        stop_on_target_force=False,
        termination_criteria=None
    )
    recorder.stop()
    
    # 清除偏置补偿
    arm.clear_ft_bias()

    final = arm.end_effector()
    disp = final[:3] - initial_pose[:3]
    rospy.loginfo("  结果=%s  Z位移=%.3f mm", result, disp[2] * 1000)
    return result, disp


# ---------------------------------------------------------------------------
# 绘图
# ---------------------------------------------------------------------------
def plot_results(recorder, save_dir, force_levels):
    data = recorder.get_arrays()
    if data is None:
        rospy.logwarn("无数据可绘图")
        return

    arr, labels = data
    t   = arr[:, 0]
    fx  = arr[:, 1]
    fy  = arr[:, 2]
    fz  = arr[:, 3]
    x   = arr[:, 4]
    y   = arr[:, 5]
    z   = arr[:, 6]

    # 以第一帧为基准计算位移 (mm)
    z_disp = (z - z[0]) * 1000.0
    x_disp = (x - x[0]) * 1000.0
    y_disp = (y - y[0]) * 1000.0

    # 为每个阶段找起止时刻，用于背景色区分
    phase_intervals = []
    cur_label = labels[0]
    t_start = t[0]
    for i in range(1, len(labels)):
        if labels[i] != cur_label:
            phase_intervals.append((cur_label, t_start, t[i - 1]))
            cur_label = labels[i]
            t_start = t[i]
    phase_intervals.append((cur_label, t_start, t[-1]))

    phase_colors = ["#e8f4fd", "#fef9e7", "#eafaf1"]

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("UR3 Z-axis Impedance Control Simulation Results", fontsize=14, fontweight="bold", y=0.98)
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    ax_fz   = fig.add_subplot(gs[0, :])
    ax_disp = fig.add_subplot(gs[1, :])
    ax_fx   = fig.add_subplot(gs[2, 0])
    ax_fy   = fig.add_subplot(gs[2, 1])

    axes = [ax_fz, ax_disp, ax_fx, ax_fy]

    # 背景色
    for ax in axes:
        for k, (lbl, ts, te) in enumerate(phase_intervals):
            c = phase_colors[k % len(phase_colors)]
            ax.axvspan(ts, te, alpha=0.4, color=c, zorder=0)

    ax_fz.plot(t, fz, color="#2980b9", linewidth=1.2, label="Fz Measured")
    for _, (lbl, ts, te) in enumerate(phase_intervals):
        for fl in force_levels:
            if str(fl) in lbl or ("%.1f" % fl) in lbl:
                ax_fz.hlines(fl, ts, te, colors="#e74c3c",
                             linewidths=1.5, linestyles="--",
                             label="Target Force" if _ == 0 else "")
                break
    ax_fz.set_ylabel("Force (N)")
    ax_fz.set_title("Z-axis Contact Force")
    ax_fz.legend(loc="upper right", fontsize=8)
    ax_fz.grid(True, alpha=0.3)

    # 阶段标签
    for lbl, ts, te in phase_intervals:
        ax_fz.text((ts + te) / 2, ax_fz.get_ylim()[0] * 0.95 if ax_fz.get_ylim()[0] < 0 else
                   ax_fz.get_ylim()[0] + 0.5,
                   lbl, ha="center", va="bottom", fontsize=7, color="#555555")

    ax_disp.plot(t, z_disp, color="#27ae60", linewidth=1.2, label="Z Displacement")
    ax_disp.plot(t, x_disp, color="#8e44ad", linewidth=0.8,
                 linestyle=":", alpha=0.7, label="X Displacement (should be ~0)")
    ax_disp.plot(t, y_disp, color="#f39c12", linewidth=0.8,
                 linestyle=":", alpha=0.7, label="Y Displacement (should be ~0)")
    ax_disp.axhline(0, color="gray", linewidth=0.5)
    ax_disp.set_ylabel("Displacement (mm)")
    ax_disp.set_title("End Effector Displacement (relative to initial position)")
    ax_disp.legend(loc="lower right", fontsize=8)
    ax_disp.grid(True, alpha=0.3)

    ax_fx.plot(t, fx, color="#e67e22", linewidth=1.0)
    ax_fx.axhline(0, color="gray", linewidth=0.5)
    ax_fx.set_xlabel("Time (s)")
    ax_fx.set_ylabel("Force (N)")
    ax_fx.set_title("X-axis Force (position controlled, should be ~0)")
    ax_fx.grid(True, alpha=0.3)

    ax_fy.plot(t, fy, color="#16a085", linewidth=1.0)
    ax_fy.axhline(0, color="gray", linewidth=0.5)
    ax_fy.set_xlabel("Time (s)")
    ax_fy.set_ylabel("Force (N)")
    ax_fy.set_title("Y-axis Force (position controlled, should be ~0)")
    ax_fy.grid(True, alpha=0.3)

    # 统一 x 轴范围
    for ax in axes:
        ax.set_xlim(t[0], t[-1])

    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    png_path = os.path.join(save_dir, "impedance_result_%s.png" % ts_str)
    fig.savefig(png_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    rospy.loginfo("结果图已保存: %s", png_path)
    return png_path


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------
def main():
    rospy.init_node("impedance_z_demo")

    save_dir = "/workspaces/ur3_sim/results"
    os.makedirs(save_dir, exist_ok=True)

    rospy.loginfo("=" * 60)
    rospy.loginfo("  Z 轴阻抗控制演示（含数据记录与绘图）")
    rospy.loginfo("  结果保存目录: %s", save_dir)
    rospy.loginfo("=" * 60)

    selection_matrix = [1.0, 1.0, 0.0, 1.0, 1.0, 1.0]
    rospy.loginfo("选择矩阵 [X,Y,Z,Rx,Ry,Rz] = %s  (0=力控, 1=位控)",
                  selection_matrix)

    impedance_model = init_impedance_controller(selection_matrix)

    arm = BiasCompensatedController(
        model=impedance_model,
        namespace=None,
        joint_names_prefix=None,
        gripper_type=None
    )

    recorder = DataRecorder(arm)

    move_to_initial_pose(arm)

    # 多级力控测试
    force_levels = [-2.0, -5.0, -8.0]
    rospy.loginfo("开始多级力控测试: %s N", force_levels)

    for i, fz in enumerate(force_levels):
        label = "F=%.1fN" % fz
        rospy.loginfo("\n>>> 第 %d/%d 步: %s", i + 1, len(force_levels), label)

        run_z_impedance_control(
            arm, recorder,
            target_force_z=fz,
            duration=8.0,
            label=label
        )

        if i < len(force_levels) - 1:
            rospy.sleep(1.5)

    # 保存 CSV
    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(save_dir, "impedance_data_%s.csv" % ts_str)
    recorder.save_csv(csv_path)

    # 生成并保存图表
    rospy.loginfo("生成结果曲线图...")
    png_path = plot_results(recorder, save_dir, force_levels)

    rospy.loginfo("=" * 60)
    rospy.loginfo("  全部完成!")
    rospy.loginfo("  CSV : %s", csv_path)
    rospy.loginfo("  图表: %s", png_path)
    rospy.loginfo("=" * 60)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("错误: %s", e)
        import traceback
        traceback.print_exc()
