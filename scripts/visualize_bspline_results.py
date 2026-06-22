#!/usr/bin/env python3
"""
B-spline Trajectory Optimization Visualization Script.

Reads joint test data (JPS + B-spline) and generates:
  1. comparison maps (original vs smoothed path + control points)
  2. curvature profile overlays
  3. detail views for key scenarios
  4. summary infographic with benchmark table
"""

import sys
import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCENARIO_TITLES = {
    's1_empty_diag': 'S1: Empty Diagonal',
    's2_wall_gap': 'S2: Wall with Gap',
    's3_maze': 'S3: Maze Corridor',
    's4_obstaclefield': 'S4: Obstacle Field',
    's5_large_empty': 'S5: Large Empty (200x200)',
    's6_costband': 'S6: High-Cost Band',
}

SCENARIO_ORDER = ['s4_obstaclefield', 's3_maze', 's2_wall_gap',
                  's1_empty_diag', 's5_large_empty', 's6_costband']

COLOR_ORIG = '#00e676'    # green original path
COLOR_SMOOTH = '#2196f3'  # blue smoothed path
COLOR_CTRL = '#f44336'    # red control points
COLOR_START = '#4caf50'   # green start
COLOR_GOAL = '#ff5722'    # orange goal
COLOR_OBS = '#37474f'     # dark obstacle


def load_path(fname):
    if not os.path.exists(fname):
        return np.empty((0, 2))
    with open(fname) as f:
        lines = [l.strip().split() for l in f if l.strip() and not l.startswith('#')]
    pts = []
    for parts in lines:
        if len(parts) < 2: continue
        if parts[-1] in ('start', 'goal'):
            pts.append([float(parts[0]), float(parts[1])])
        else:
            try:
                pts.append([float(parts[0]), float(parts[1])])
            except ValueError:
                continue
    if not pts:
        return np.empty((0, 2))
    return np.array(pts)


def load_grid(fname):
    if not os.path.exists(fname):
        return None
    pts = np.loadtxt(fname, ndmin=2)
    if pts.size == 0:
        return None
    w = int(pts[:, 0].max()) + 1
    h = int(pts[:, 1].max()) + 1
    grid = np.full((h, w), 255, dtype=np.uint8)
    for x, y, c in pts.astype(int):
        if 0 <= y < h and 0 <= x < w:
            grid[y, x] = min(c, 255)
    return grid


def make_cost_image(grid):
    h, w = grid.shape
    img = np.ones((h, w, 4), dtype=np.float32)
    for y in range(h):
        for x in range(w):
            c = grid[y, x]
            if c == 254:
                img[y, x] = [0.22, 0.28, 0.31, 1.0]
            elif c >= 200:
                img[y, x] = [0.9, 0.5, 0.2, 1.0]
            elif c == 0:
                img[y, x] = [0.95, 0.95, 0.95, 1.0]
            else:
                r = min(1.0, c / 252.0 * 0.7)
                img[y, x] = [r, 0.8 - r * 0.5, 1.0 - r, 1.0]
    return img


def draw_comparison(ax, data_dir, scenario):
    pfx = os.path.join(data_dir, scenario)
    grid = load_grid(pfx + '_grid.dat')
    orig = load_path(pfx + '_orig_path.dat')
    smooth = load_path(pfx + '_smooth_path.dat')
    ctrl = load_path(pfx + '_ctrl_pts.dat')
    sg = load_path(pfx + '_startgoal.dat') if os.path.exists(pfx + '_startgoal.dat') else None

    if grid is not None:
        h, w = grid.shape
        ax.imshow(make_cost_image(grid), origin='lower', interpolation='nearest',
                  extent=[0, w, 0, h])
    else:
        ax.set_facecolor('#f5f5f5')

    if len(orig) > 1:
        ax.plot(orig[:, 0], orig[:, 1], '--', color=COLOR_ORIG, linewidth=1.5,
                alpha=0.7, label='JPS (original)')
    if len(smooth) > 1:
        ax.plot(smooth[:, 0], smooth[:, 1], '-', color=COLOR_SMOOTH, linewidth=2.5,
                label='B-spline (smoothed)')
    if len(ctrl) > 1:
        ax.scatter(ctrl[:, 0], ctrl[:, 1], s=30, c=COLOR_CTRL, marker='D',
                   edgecolors='white', linewidths=0.5, zorder=5, label='Control pts')
    if sg is not None and sg.shape[0] >= 2:
        ax.scatter(sg[0, 0], sg[0, 1], s=80, c=COLOR_START, marker='o',
                   edgecolors='white', linewidths=1.5, zorder=6, label='Start')
        ax.scatter(sg[1, 0], sg[1, 1], s=80, c=COLOR_GOAL, marker='*',
                   edgecolors='white', linewidths=1.5, zorder=6, label='Goal')

    ax.set_aspect('equal')
    ax.set_title(SCENARIO_TITLES.get(scenario, scenario), fontsize=10, fontweight='bold')
    ax.legend(loc='upper right', fontsize=6, framealpha=0.85)
    ax.grid(False)


def draw_curvature_profile(ax, data_dir, scenario):
    csv_path = os.path.join(data_dir, scenario + '_curvature.csv')
    if not os.path.exists(csv_path):
        ax.text(0.5, 0.5, 'No curvature data', transform=ax.transAxes, ha='center')
        return

    data = np.loadtxt(csv_path, delimiter=',', skiprows=1, ndmin=2)
    if data.size == 0 or data.shape[0] < 2:
        return

    ax.plot(data[:, 0], data[:, 1], '-', color=COLOR_CTRL, linewidth=1.5)
    ax.fill_between(data[:, 0], 0, data[:, 1], color=COLOR_CTRL, alpha=0.15)
    ax.set_xlabel('Arc parameter')
    ax.set_ylabel('Curvature κ')
    ax.set_title(f'{SCENARIO_TITLES.get(scenario, scenario)} — Curvature', fontsize=9,
                 fontweight='bold')
    ax.grid(True, alpha=0.3)


def draw_summary(ax, data_dir):
    bench_path = os.path.join(data_dir, 'benchmark.csv')
    if not os.path.exists(bench_path):
        return
    rows = []
    with open(bench_path) as f:
        for row in csv.DictReader(f):
            rows.append(row)

    ax.axis('tight')
    ax.axis('off')
    col_labels = ['Scenario', 'Pts (orig)', 'Max κ (orig)', 'Max κ (smooth)',
                  'Len (orig)', 'Len (smooth)', 'Converged']
    table_data = []
    for r in rows:
        name = SCENARIO_TITLES.get(r['scenario'], r['scenario'])
        table_data.append([
            name.split(':')[-1].strip(),
            r['orig_points'],
            f"{float(r['orig_max_curv']):.3f}",
            f"{float(r['smooth_max_curv']):.3f}",
            f"{float(r['orig_len']):.1f}",
            f"{float(r['smooth_len']):.1f}",
            'Yes' if r['converged'] == '1' else 'No',
        ])

    table = ax.table(cellText=table_data, colLabels=col_labels,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.3)
    for j in range(len(col_labels)):
        table[0, j].set_facecolor('#37474f')
        table[0, j].set_text_props(color='white', fontweight='bold')
    for i in range(len(table_data)):
        color = '#f5f5f5' if i % 2 == 0 else '#ffffff'
        for j in range(len(col_labels)):
            table[i + 1, j].set_facecolor(color)
    ax.set_title('Joint Test Results Summary', fontweight='bold', fontsize=12, pad=20)


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/bspline_viz_data'
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(data_dir, 'viz_output')
    os.makedirs(out_dir, exist_ok=True)

    plt.rcParams.update({
        'figure.dpi': 150, 'font.family': 'sans-serif', 'font.size': 9,
        'axes.titlesize': 11, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.1})

    scenarios = [s for s in SCENARIO_ORDER
                 if os.path.exists(os.path.join(data_dir, s + '_orig_path.dat'))]

    # Fig 1: Comparison maps (3x2)
    fig1, axes1 = plt.subplots(2, 3, figsize=(15, 9))
    for idx, sc in enumerate(scenarios):
        row, col = divmod(idx, 3)
        draw_comparison(axes1[row, col], data_dir, sc)
    for idx in range(len(scenarios), 6):
        row, col = divmod(idx, 3)
        axes1[row, col].set_visible(False)
    fig1.suptitle('B-spline Trajectory Optimization — Path Comparison', fontsize=14,
                  fontweight='bold', y=0.98)
    fig1.tight_layout()
    fig1.savefig(os.path.join(out_dir, 'bspline_01_comparison.png'))
    plt.close(fig1)
    print(f'Saved: bspline_01_comparison.png')

    # Fig 2: Curvature profiles (3x2)
    fig2, axes2 = plt.subplots(2, 3, figsize=(15, 8))
    for idx, sc in enumerate(scenarios):
        row, col = divmod(idx, 3)
        draw_curvature_profile(axes2[row, col], data_dir, sc)
    for idx in range(len(scenarios), 6):
        row, col = divmod(idx, 3)
        axes2[row, col].set_visible(False)
    fig2.suptitle('B-spline Curvature Profiles', fontsize=14, fontweight='bold', y=0.98)
    fig2.tight_layout()
    fig2.savefig(os.path.join(out_dir, 'bspline_02_curvature.png'))
    plt.close(fig2)
    print(f'Saved: bspline_02_curvature.png')

    # Fig 3: Detail view (S4 obstacle field)
    if 's4_obstaclefield' in scenarios:
        fig3, ax3 = plt.subplots(1, 1, figsize=(10, 8))
        draw_comparison(ax3, data_dir, 's4_obstaclefield')
        fig3.suptitle('B-spline Detail: Obstacle Field (23 JPS waypoints → smooth curve)',
                      fontsize=13, fontweight='bold')
        fig3.tight_layout()
        fig3.savefig(os.path.join(out_dir, 'bspline_03_detail.png'))
        plt.close(fig3)
        print(f'Saved: bspline_03_detail.png')

    # Fig 4: Summary table
    fig4, ax4 = plt.subplots(1, 1, figsize=(12, 5))
    draw_summary(ax4, data_dir)
    fig4.tight_layout()
    fig4.savefig(os.path.join(out_dir, 'bspline_04_summary.png'))
    plt.close(fig4)
    print(f'Saved: bspline_04_summary.png')

    print(f'\nAll visualization outputs saved to: {out_dir}/')


if __name__ == '__main__':
    main()
