#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
独立的阻抗控制结果绘图工具
可以读取历史 CSV 数据重新生成曲线图

使用方法:
  python plot_impedance_results.py <csv文件路径>
  
示例:
  python plot_impedance_results.py ~/impedance_results/impedance_data_20260607_120000.csv
"""

import sys
import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from datetime import datetime


def load_csv_data(csv_path):
    """Load data from CSV file."""
    if not os.path.exists(csv_path):
        print(f"Error: File not found - {csv_path}")
        return None
    
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    
    if not rows:
        print("Error: CSV file is empty")
        return None
    
    data = {
        't': np.array([float(r['time_s']) for r in rows]),
        'fx': np.array([float(r['fx']) for r in rows]),
        'fy': np.array([float(r['fy']) for r in rows]),
        'fz': np.array([float(r['fz']) for r in rows]),
        'x': np.array([float(r['x']) for r in rows]),
        'y': np.array([float(r['y']) for r in rows]),
        'z': np.array([float(r['z']) for r in rows]),
        'phase': [r['phase'] for r in rows]
    }
    
    print(f"Loaded {len(rows)} records")
    return data


def extract_force_levels(phases):
    """Extract target force values from phase labels."""
    force_levels = []
    for p in phases:
        if 'F=' in p:
            try:
                val = float(p.split('F=')[1].split('N')[0])
                if val not in force_levels:
                    force_levels.append(val)
            except:
                pass
    return sorted(force_levels)


def plot_data(data, output_path=None):
    """生成完整的结果曲线图"""
    t = data['t']
    fx = data['fx']
    fy = data['fy']
    fz = data['fz']
    x = data['x']
    y = data['y']
    z = data['z']
    labels = data['phase']
    
    # 计算位移 (mm)
    z_disp = (z - z[0]) * 1000.0
    x_disp = (x - x[0]) * 1000.0
    y_disp = (y - y[0]) * 1000.0
    
    # 提取阶段信息
    phase_intervals = []
    cur_label = labels[0]
    t_start = t[0]
    for i in range(1, len(labels)):
        if labels[i] != cur_label:
            phase_intervals.append((cur_label, t_start, t[i - 1]))
            cur_label = labels[i]
            t_start = t[i]
    phase_intervals.append((cur_label, t_start, t[-1]))
    
    force_levels = extract_force_levels(labels)
    phase_colors = ["#e8f4fd", "#fef9e7", "#eafaf1"]
    
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("UR3 Z-axis Impedance Control Simulation Results", fontsize=14, fontweight="bold", y=0.98)
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)
    
    ax_fz = fig.add_subplot(gs[0, :])
    ax_disp = fig.add_subplot(gs[1, :])
    ax_fx = fig.add_subplot(gs[2, 0])
    ax_fy = fig.add_subplot(gs[2, 1])
    
    axes = [ax_fz, ax_disp, ax_fx, ax_fy]
    
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
    
    for lbl, ts, te in phase_intervals:
        y_pos = ax_fz.get_ylim()[0] * 0.95 if ax_fz.get_ylim()[0] < 0 else ax_fz.get_ylim()[0] + 0.5
        ax_fz.text((ts + te) / 2, y_pos, lbl, ha="center", va="bottom", fontsize=7, color="#555555")
    
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
    
    for ax in axes:
        ax.set_xlim(t[0], t[-1])
    
    if output_path is None:
        base = os.path.splitext(os.path.basename(sys.argv[1]))[0]
        output_path = os.path.join(os.path.dirname(sys.argv[1]), f"{base}_plot.png")
    
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved: {output_path}")
    return output_path


def print_statistics(data):
    """Print data statistics."""
    print("\nData Statistics:")
    print(f"  Time range: {data['t'][0]:.2f} ~ {data['t'][-1]:.2f} s (total {data['t'][-1] - data['t'][0]:.2f} s)")
    print(f"  Force (N):")
    print(f"    Fx: min={data['fx'].min():.2f}, max={data['fx'].max():.2f}, mean={data['fx'].mean():.2f}")
    print(f"    Fy: min={data['fy'].min():.2f}, max={data['fy'].max():.2f}, mean={data['fy'].mean():.2f}")
    print(f"    Fz: min={data['fz'].min():.2f}, max={data['fz'].max():.2f}, mean={data['fz'].mean():.2f}")
    
    z_disp = (data['z'] - data['z'][0]) * 1000.0
    print(f"  Z displacement (mm): min={z_disp.min():.3f}, max={z_disp.max():.3f}, total={z_disp[-1]:.3f}")
    
    phases = list(set(data['phase']))
    print(f"  Phases: {len(phases)} - {phases}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_impedance_results.py <csv_file_path>")
        print("\nExample:")
        print("  python plot_impedance_results.py ~/impedance_results/impedance_data_20260607_120000.csv")
        sys.exit(1)
    
    csv_path = os.path.expanduser(sys.argv[1])
    
    print(f"Reading data file: {csv_path}")
    data = load_csv_data(csv_path)
    
    if data is None:
        sys.exit(1)
    
    print_statistics(data)
    
    print("\nGenerating plot...")
    output_path = plot_data(data)
    
    print("\nDone!")


if __name__ == "__main__":
    main()
