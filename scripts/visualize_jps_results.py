#!/usr/bin/env python3
"""
JPS Planner Visualization Script.

Reads planning data exported by jps_viz_exporter and generates:
  1. Scenario maps (grid + path + jump points + expanded nodes)
  2. Cost surface 3D plot
  3. Benchmark performance charts
  4. Summary infographic

Usage:
  python3 visualize_jps_results.py <data_dir> [output_dir]
"""

import sys
import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib.colors import LinearSegmentedColormap

# ──────────────────────────────────────────────────────────
# Color schemes
# ──────────────────────────────────────────────────────────
CMAP_COST = LinearSegmentedColormap.from_list(
    'cost', ['#f0f0f0', '#ffe0b2', '#ff9800', '#d32f2f', '#212121'],
    N=256)
COLOR_PATH = '#00e676'        # green
COLOR_START = '#2196f3'       # blue
COLOR_GOAL = '#f44336'        # red
COLOR_JUMP = '#ffd600'        # yellow/gold
COLOR_EXPANDED = 'yellow'     # light yellow
COLOR_OBS = '#37474f'         # dark grey
COLOR_HIGH = '#ff7043'        # orange (high cost)
COLOR_UNK = '#b0bec5'         # grey (unknown)

SCENARIO_TITLES = {
    's1_empty_diag': 'S1: Empty Grid — Diagonal',
    's2_wall_gap': 'S2: Wall with Gap',
    's3_maze': 'S3: Maze Corridor',
    's4_obstaclefield': 'S4: Obstacle Field',
    's5_large_empty': 'S5: Large Empty Grid (200×200)',
    's6_costband': 'S6: High-Cost Diagonal Band',
}


def load_grid(path):
    """Load grid data (x y cost) into a 2D numpy array."""
    pts = np.loadtxt(path, dtype=np.float64, ndmin=2)
    if pts.size == 0:
        return None
    w = int(pts[:, 0].max()) + 1
    h = int(pts[:, 1].max()) + 1
    grid = np.full((h, w), 255, dtype=np.uint8)
    for x, y, c in pts.astype(int):
        if 0 <= y < h and 0 <= x < w:
            grid[y, x] = min(c, 255)
    return grid


def load_path(path):
    """Load path data (x y)."""
    pts = np.loadtxt(path, dtype=np.float64, ndmin=2)
    if pts.size == 0:
        return np.empty((0, 2))
    return pts


def load_intlist(path):
    """Load integer point list (x y)."""
    pts = np.loadtxt(path, dtype=np.int64, ndmin=2)
    if pts.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    return pts


def load_startgoal(path):
    """Load start/goal data (format: x y label)."""
    if not os.path.exists(path):
        return None, None
    with open(path) as f:
        lines = [l.strip().split() for l in f if l.strip() and not l.startswith('#')]
    start = None
    goal = None
    for parts in lines:
        if len(parts) < 2:
            continue
        x = float(parts[0])
        y = float(parts[1])
        label = parts[2] if len(parts) > 2 else ''
        if label == 'start':
            start = np.array([x, y])
        elif label == 'goal':
            goal = np.array([x, y])
    return start, goal


def make_cost_image(grid):
    """Convert a raw cost grid to an RGBA image."""
    h, w = grid.shape
    img = np.ones((h, w, 4), dtype=np.float32)

    for y in range(h):
        for x in range(w):
            c = grid[y, x]
            if c == 254:
                img[y, x] = [0.216, 0.278, 0.310, 1.0]  # dark grey obstacle
            elif c == 253:
                img[y, x] = [1.0, 0.267, 0.0, 1.0]  # orange inscribed
            elif c == 255:
                img[y, x] = [0.690, 0.741, 0.773, 1.0]  # grey unknown
            elif c == 0:
                img[y, x] = [0.95, 0.95, 0.95, 1.0]  # light free
            else:
                r = min(1.0, c / 252.0 * 0.7)
                img[y, x] = [r, 0.8 - r * 0.5, 1.0 - r, 1.0]
    return img


def draw_scenario(ax, data_dir, scenario, grid=None):
    """Draw one scenario map with path, jump points, expanded nodes."""
    pfx = os.path.join(data_dir, scenario)

    if grid is None:
        grid = load_grid(pfx + '_grid.dat')
    path = load_path(pfx + '_path.dat')
    expanded = load_intlist(pfx + '_expanded.dat')
    jumps = load_intlist(pfx + '_jumppoints.dat')
    start, goal = load_startgoal(pfx + '_startgoal.dat')

    if grid is None:
        ax.text(0.5, 0.5, 'No data', transform=ax.transAxes, ha='center')
        return

    h, w = grid.shape

    # Background cost image
    img = make_cost_image(grid)
    ax.imshow(img, origin='lower', interpolation='nearest',
              extent=[0, w, 0, h])

    # Expanded nodes
    if len(expanded) > 0:
        ax.scatter(expanded[:, 0] + 0.5, expanded[:, 1] + 0.5,
                   s=12, c=COLOR_EXPANDED, edgecolors='#f9a825', linewidths=0.3,
                   alpha=0.85, marker='o', zorder=3, label='Expanded')

    # Jump points
    if len(jumps) > 0:
        ax.scatter(jumps[:, 0] + 0.5, jumps[:, 1] + 0.5,
                   s=25, c=COLOR_JUMP, edgecolors='#e65100', linewidths=0.5,
                   marker='D', zorder=4, label='Jump Points')

    # Path
    if len(path) > 1:
        ax.plot(path[:, 0], path[:, 1], '-', color=COLOR_PATH, linewidth=2.5,
                zorder=5, label='Path', alpha=0.9)

    # Start & Goal
    if start is not None:
        ax.scatter(*start, s=100, c=COLOR_START, marker='o', edgecolors='white',
                   linewidths=1.5, zorder=6, label='Start')
    if goal is not None:
        ax.scatter(*goal, s=100, c=COLOR_GOAL, marker='*', edgecolors='white',
                   linewidths=1.5, zorder=6, label='Goal')

    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_aspect('equal')
    ax.set_title(SCENARIO_TITLES.get(scenario, scenario), fontsize=10,
                 fontweight='bold')
    ax.set_xlabel('x [cells]')
    ax.set_ylabel('y [cells]')
    ax.grid(False)

    # Legend
    ax.legend(loc='upper right', fontsize=6, ncol=2,
              framealpha=0.85, edgecolor='#666')


def draw_benchmark_plots(ax_time, ax_nodes, data_dir):
    """Draw benchmark charts: time & nodes vs grid size."""
    bench_path = os.path.join(data_dir, 'benchmark.csv')
    if not os.path.exists(bench_path):
        ax_time.text(0.5, 0.5, 'No benchmark data',
                     transform=ax_time.transAxes, ha='center')
        return

    scenarios = []
    scales = []
    with open(bench_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            s = dict(row)
            if s['scenario'].startswith('scale'):
                scales.append(s)
            else:
                scenarios.append(s)

    # ── Chart 1: Timing vs grid size ──
    if scales:
        sizes = [int(s['w']) for s in scales]
        times = [float(s['time_ms']) for s in scales]

        ax_time.plot(sizes, times, 'o-', color='#2196f3', linewidth=2,
                     markersize=8, label='JPS Planning Time')
        # Quadratic reference (scaled)
        ref = [t for t in times]
        ax_time.plot(sizes, ref, '--', color='#ff9800', linewidth=1.5,
                     alpha=0.6, label='O(n²) reference')
        ax_time.set_xlabel('Grid dimension (n × n)')
        ax_time.set_ylabel('Time [ms]')
        ax_time.set_title('Planning Time vs Grid Size', fontweight='bold')
        ax_time.legend(fontsize=8)
        ax_time.grid(True, alpha=0.3)

        # Annotate last point
        ax_time.annotate(f'{times[-1]:.2f} ms', (sizes[-1], times[-1]),
                         textcoords="offset points", xytext=(0, 12),
                         fontsize=8, ha='center')

    # ── Chart 2: Nodes expanded vs grid size ──
    if scales:
        expanded = [int(s['expanded']) for s in scales]
        ax_nodes.bar(range(len(sizes)), expanded, color='#4caf50',
                     edgecolor='#2e7d32', alpha=0.8)
        ax_nodes.set_xticks(range(len(sizes)))
        ax_nodes.set_xticklabels([f'{s}x{s}' for s in sizes], rotation=45)
        ax_nodes.set_ylabel('Expanded Nodes')
        ax_nodes.set_title('Node Expansions (JPS on Empty Grid)',
                           fontweight='bold')
        ax_nodes.grid(True, axis='y', alpha=0.3)

        # JPS efficiency annotation
        for i, e in enumerate(expanded):
            ax_nodes.text(i, e + 0.1, str(e), ha='center', fontsize=8)


def draw_costband_3d(ax, data_dir):
    """3D surface plot of the cost band scenario."""
    pfx = os.path.join(data_dir, 's6_costband')
    grid = load_grid(pfx + '_grid.dat')
    path = load_path(pfx + '_path.dat')

    if grid is None:
        ax.text2D(0.5, 0.5, 'No data', transform=ax.transAxes, ha='center')
        return

    # Downsample for 3D rendering
    step = max(1, min(grid.shape) // 80)
    gs = grid[::step, ::step]
    h, w = gs.shape
    X = np.arange(0, grid.shape[1], step)[:w]
    Y = np.arange(0, grid.shape[0], step)[:h]
    X, Y = np.meshgrid(X, Y)

    ax.plot_surface(X, Y, gs.astype(float), cmap='YlOrRd',
                    linewidth=0, antialiased=True, alpha=0.85)

    # Path overlay on surface
    if len(path) > 1:
        ax.plot(path[:, 0], path[:, 1],
                np.interp(path[:, 0], np.arange(grid.shape[1]),
                          grid[grid.shape[0]//2]),
                '-', color='#00e676', linewidth=3, zorder=10, label='Path')

    ax.set_xlabel('X [cells]')
    ax.set_ylabel('Y [cells]')
    ax.set_zlabel('Cost')
    ax.set_title('3D Cost Surface with JPS Path', fontweight='bold')
    ax.view_init(elev=35, azim=-60)


def draw_perf_summary(ax, data_dir):
    """Summary bar chart comparing all scenarios."""
    bench_path = os.path.join(data_dir, 'benchmark.csv')
    if not os.path.exists(bench_path):
        return

    scenarios = []
    with open(bench_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row['scenario'].startswith('scale'):
                scenarios.append(row)

    names = [s['scenario'].replace('_', '\n') for s in scenarios]
    times = [float(s['time_ms']) for s in scenarios]
    expanded = [int(s['expanded']) for s in scenarios]

    x = np.arange(len(names))
    width = 0.35

    bars1 = ax.bar(x - width/2, times, width, label='Time [ms]',
                   color='#2196f3', edgecolor='#1565c0', alpha=0.8)
    ax2 = ax.twinx()
    bars2 = ax2.bar(x + width/2, expanded, width, label='Expanded Nodes',
                    color='#4caf50', edgecolor='#2e7d32', alpha=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=7)
    ax.set_ylabel('Time [ms]', color='#2196f3')
    ax2.set_ylabel('Expanded Nodes', color='#4caf50')
    ax.set_title('Scenario Performance Summary', fontweight='bold')

    # Combined legend
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=7)

    # Value labels
    for bar, val in zip(bars1, times):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                f'{val:.1f}', ha='center', va='bottom', fontsize=6)
    for bar, val in zip(bars2, expanded):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                 str(val), ha='center', va='bottom', fontsize=6)


def draw_results_table(ax, data_dir):
    """Draw a formatted results summary table."""
    bench_path = os.path.join(data_dir, 'benchmark.csv')
    if not os.path.exists(bench_path):
        return

    all_rows = []
    with open(bench_path) as f:
        for row in csv.DictReader(f):
            if not row['scenario'].startswith('scale'):
                all_rows.append(row)

    ax.axis('tight')
    ax.axis('off')

    col_labels = ['Scenario', 'Grid', 'Time (ms)', 'Expanded', 'Path Len']
    table_data = []
    for r in all_rows:
        name = SCENARIO_TITLES.get(r['scenario'], r['scenario'])
        name = name.split(':')[1].strip() if ':' in name else name
        table_data.append([
            name,
            f"{r['w']}×{r['h']}",
            f"{float(r['time_ms']):.3f}",
            r['expanded'],
            r['path_len'],
        ])

    table = ax.table(cellText=table_data, colLabels=col_labels,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.4)

    # Style header
    for j in range(len(col_labels)):
        cell = table[0, j]
        cell.set_facecolor('#37474f')
        cell.set_text_props(color='white', fontweight='bold')

    # Alternate row colors
    for i in range(len(table_data)):
        color = '#f5f5f5' if i % 2 == 0 else '#ffffff'
        for j in range(len(col_labels)):
            table[i + 1, j].set_facecolor(color)

    ax.set_title('JPS Planning Results Summary', fontweight='bold',
                 fontsize=12, pad=20)


# ──────────────────────────────────────────────────────────
# Main layout — multi-panel figure
# ──────────────────────────────────────────────────────────

def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/jps_viz_data'
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(data_dir, 'viz_output')
    os.makedirs(out_dir, exist_ok=True)

    plt.rcParams.update({
        'figure.dpi': 150,
        'font.family': 'sans-serif',
        'font.size': 9,
        'axes.titlesize': 11,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.1,
    })

    scenarios_present = [s for s in SCENARIO_TITLES
                         if os.path.exists(os.path.join(data_dir, s + '_grid.dat'))]

    # ── Figure 1: Multi-panel scenario maps ──
    ncols = 3
    nrows = (len(scenarios_present) + ncols - 1) // ncols
    fig1, axes1 = plt.subplots(nrows, ncols, figsize=(5 * ncols, 5 * nrows))
    if nrows * ncols == 1:
        axes1 = np.array([[axes1]])
    elif nrows == 1:
        axes1 = axes1.reshape(1, -1)
    elif ncols == 1:
        axes1 = axes1.reshape(-1, 1)

    for idx, scenario in enumerate(scenarios_present):
        row, col = divmod(idx, ncols)
        draw_scenario(axes1[row, col], data_dir, scenario)

    # Hide unused subplots
    for idx in range(len(scenarios_present), nrows * ncols):
        row, col = divmod(idx, ncols)
        axes1[row, col].set_visible(False)

    fig1.suptitle('JPS Planner — Scenario Visualization',
                  fontsize=14, fontweight='bold', y=0.98)
    fig1.tight_layout()
    fig1.savefig(os.path.join(out_dir, '01_scenario_maps.png'))
    plt.close(fig1)
    print(f'Saved: 01_scenario_maps.png')

    # ── Figure 2: Large detail views (obstacle field + maze + wall gap) ──
    detail_scenarios = [s for s in ['s4_obstaclefield', 's3_maze', 's2_wall_gap']
                        if s in scenarios_present]
    if detail_scenarios:
        fig2, axes2 = plt.subplots(1, len(detail_scenarios),
                                   figsize=(6 * len(detail_scenarios), 5.5))
        if len(detail_scenarios) == 1:
            axes2 = [axes2]
        for ax, sc in zip(axes2, detail_scenarios):
            draw_scenario(ax, data_dir, sc)
        fig2.suptitle('JPS Planner — Detailed Scenario Views',
                      fontsize=13, fontweight='bold')
        fig2.tight_layout()
        fig2.savefig(os.path.join(out_dir, '02_detail_views.png'))
        plt.close(fig2)
        print(f'Saved: 02_detail_views.png')

    # ── Figure 3: Empty grid comparison (80×80 vs 200×200) ──
    empty_scenarios = [s for s in ['s1_empty_diag', 's5_large_empty']
                       if s in scenarios_present]
    if empty_scenarios:
        fig3, axes3 = plt.subplots(1, 2, figsize=(12, 5.5))
        for ax, sc in zip(axes3, empty_scenarios):
            draw_scenario(ax, data_dir, sc)
        fig3.suptitle('JPS on Empty Grids — O(1) Node Expansion',
                      fontsize=13, fontweight='bold')
        fig3.tight_layout()
        fig3.savefig(os.path.join(out_dir, '03_empty_grids.png'))
        plt.close(fig3)
        print(f'Saved: 03_empty_grids.png')

    # ── Figure 4: Benchmark charts ──
    fig4, (ax_time, ax_nodes) = plt.subplots(1, 2, figsize=(12, 5))
    draw_benchmark_plots(ax_time, ax_nodes, data_dir)
    fig4.suptitle('JPS Performance Benchmark', fontsize=13, fontweight='bold')
    fig4.tight_layout()
    fig4.savefig(os.path.join(out_dir, '04_benchmarks.png'))
    plt.close(fig4)
    print(f'Saved: 04_benchmarks.png')

    # ── Figure 5: Performance summary ──
    fig5, (ax_perf, ax_table) = plt.subplots(1, 2, figsize=(14, 5),
                                              gridspec_kw={'width_ratios': [1.2, 0.8]})
    draw_perf_summary(ax_perf, data_dir)
    draw_results_table(ax_table, data_dir)
    fig5.suptitle('JPS Planner — Results Summary', fontsize=13,
                  fontweight='bold')
    fig5.tight_layout()
    fig5.savefig(os.path.join(out_dir, '05_summary.png'))
    plt.close(fig5)
    print(f'Saved: 05_summary.png')

    # ── Figure 6: Cost band 3D view ──
    if 's6_costband' in scenarios_present:
        fig6 = plt.figure(figsize=(10, 7))
        ax3d = fig6.add_subplot(111, projection='3d')
        draw_costband_3d(ax3d, data_dir)
        fig6.suptitle('JPS Path on Non-Uniform Cost Surface',
                      fontsize=13, fontweight='bold')
        fig6.tight_layout()
        fig6.savefig(os.path.join(out_dir, '06_costband_3d.png'))
        plt.close(fig6)
        print(f'Saved: 06_costband_3d.png')

    print(f'\nAll visualizations saved to: {out_dir}/')
    print('Generated files:')
    for f in sorted(os.listdir(out_dir)):
        fpath = os.path.join(out_dir, f)
        size_kb = os.path.getsize(fpath) / 1024
        print(f'  {f} ({size_kb:.1f} KB)')


if __name__ == '__main__':
    main()
