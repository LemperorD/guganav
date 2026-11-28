# 导航栈 CPU 性能基线（`perf` Benchmark）

本文记录 `guganav` 导航栈在仿真环境下的 CPU / 微架构性能基线与优化演进，供后续改动做回归对比。

> 架构与采集口径：Intel 混合架构（P 核 `cpu_core` + E 核 `cpu_atom`）在 `perf` 中会拆为两张事件表，**所有跨版本对比必须合并 P+E 绝对 `cycles` 与 `CPUs utilized`**，不能只看单张表的相对百分比。

---

## 1. 测试环境与统一口径

- **硬件平台**：13th Gen Intel Core i9-13980HX（32 逻辑核 = 8 P-core × 2 HT + 16 E-core）
- **软件环境**：Ubuntu 22.04（Linux 6.8 HWE）、ROS 2 Humble
- **进程架构（当前基线）**：2 进程（`component_container_isolated` 承载 Nav2 / 感知 / 重定位，开启 `use_intra_process_comms: True`；`pointlio_mapping` 独立进程保持故障隔离）
- **受控无 RViz 基准协议**：
  - 启动参数：`world:=rmul_2025 slam:=False use_composition:=True use_rviz:=False show_prior_pcd:=False pcd_save:=False planner:=jps controller:=mppi`
  - 运动负载：通过 `NavigateToPose` Action 在 `(0.0, 0.0) ↔ (5.0, -4.0)` 之间连续往返（不订阅 `point_lio` 话题，避免引入额外进程间序列化）
  - 采样窗口：**20s `perf record --call-graph lbr -F 999 -p <PIDS>`**（约行驶 `20 m` / 完成 2 趟）+ **10s `perf stat -p <PIDS>`**（约行驶 `10 m` / 完成 1 趟）

---

## 2. 优化演进总览（静止 / 早期阶段，含 RViz）

| 阶段 / 提交 | 关键改动 | 20s P+E 总 cycles | 相对初始变化 |
| :--- | :--- | ---: | ---: |
| 初始状态（未优化，多进程） | OpenMP 默认自旋、`small_gicp` 16 线程、`use_composition:=False` | `176.32 B` | 基线 |
| `cc65dc9` | 永久注入 `OMP_WAIT_POLICY=PASSIVE` + `GOMP_SPINCOUNT=0`；`small_gicp.num_threads` `16 -> 4` 并移除冗余 `source_tree_`；启用 `use_composition:=True` 修复 `/cloud_pcd` `transient_local` 冲突 | `53.99 B`（静止待机） | **-69.4%** |

---

## 3. 受控满载往返 Benchmark（无 RViz，20s 行驶 ~20 m）

### 3.1 核心指标对比

| 基线版本 | 架构 & MPPI 参数 | 20s 里程 | 20s P+E 总 cycles | `libmppi_critics` | `libmppi_controller` | `libfastrtps` | 10s `CPUs utilized` | 10s IPC (E / P) |
| :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: | :--- |
| **`cc65dc9`（优化前基线）** | 2 进程，`30 Hz` / `batch=1800` / `steps=56` | `20.00 m` | **`82.39 B`** (`47.99B + 34.40B`) | `9.00 B` | `7.63 B` | `13.87 B` | `1.289` | `0.90 / 1.32` |
| 单容器试验（已归档未合入） | 1 进程（`point_lio` 入容器），`30 Hz` / `batch=1800` | `18.43 m` | **`76.36 B`** (`47.37B + 29.00B`) | `10.12 B` | `7.77 B` | `12.24 B` | `1.436` | `0.98 / 1.42` |
| **`1f60cbe`（MPPI 优化，30 Hz 基线）** | 2 进程，`30 Hz` / `batch=1800` / `steps=56` | `19.94 m` | **`75.48 B`** (`45.05B + 30.43B`) | **`3.87 B`**（**-57.0%**） | `7.51 B` | `12.21 B` | **`1.208`** | `1.02 / 1.42` |
| `ae59c12`（60 Hz / 2000 试验） | 2 进程，`60 Hz` / `batch=2000` / `steps=56` | `20.35 m` | `95.25 B` (`59.89B + 35.36B`) | `8.18 B` | `16.75 B` | `12.88 B` | `1.533` | `1.20 / 1.69` |
| **当前工作区（60 Hz / 1800 基线）** | 2 进程，`60 Hz` / `batch=1800` / `steps=56` | `23.95 m` | **`93.08 B`** (`55.34B + 37.74B`) | **`7.00 B`** | **`15.17 B`** | `13.33 B` | **`1.557`** | **`1.21 / 1.65`** |

> 说明：
> - **`1f60cbe`**：通过 `critic_function.hpp::accumulateCostWithPower` 与 `xt::square` 消除 12 个 Critic 中的 `xt::pow` `double` 提升及通用幂函数（`vdivpd`/`vsqrtpd`）展开，`GoalCritic::score` 由 `5.08 B` 降至 `0.13 B`（**-97.4%，约 39× 提速**），单帧 MPPI 耗时约 `5–6 ms`。
> - **当前 60 Hz / `batch=1800`**：相比 `batch=2000`，MPPI 周期降低 `-11.1%`（`24.93 B -> 22.17 B`），30s 导航窗口内 `Control loop missed its desired rate` 警告由 `15` 次（`0.83%`）降至 **`2` 次（`0.11%`）**，且 30s 行驶里程提升至 `35.17 m`；实机部署时仍需在目标工控机复测 60 Hz 尾延迟。
> - **为何不将 `point_lio` 并入单容器**：无 RViz 受控对比显示合入 `point_lio` 仅节省约 `4.6%–7.3%`（`PointCloud2` 反序列化本身仅占 `0.51%`），且失去 LIO 故障隔离，因此保持 2 进程架构。

---

### 3.2 当前 60 Hz / `batch=1800` 基线 DSO 分布（20s P+E 合并，总计 `93.08 B`）

| 共享库 / 模块 | P+E cycles | 占比 |
| :--- | ---: | ---: |
| `libmppi_controller.so` | `15.17 B` | 16.3% |
| `libfastrtps.so.2.6.12` | `13.33 B` | 14.3% |
| `libc.so.6` | `12.07 B` | 13.0% |
| `[kernel.kallsyms]` | `8.46 B` | 9.1% |
| `libsmall_gicp_relocalization.so` | `7.30 B` | 7.8% |
| `libmppi_critics.so` | `7.00 B` | 7.5% |
| `librclcpp.so` | `6.49 B` | 7.0% |
| `pointlio_mapping` | `4.78 B` | 5.1% |
| `librcl.so` | `2.60 B` | 2.8% |
| `libterrain_analysis_lib.so` | `1.93 B` | 2.1% |

---

## 4. 复现与后续回归采样方法

1. 开启非 root `perf` 权限（重启失效）：
   ```bash
   sudo sysctl -w kernel.perf_event_paranoid=-1 kernel.kptr_restrict=0
   ```
2. 启动无 RViz 仿真导航后，获取存活导航进程 PID（`component_container_isolated` 与 `pointlio_mapping`），在连续导航期间采集：
   ```bash
   perf record --call-graph lbr -F 999 -p <PIDS> -o nav_bench.data -- sleep 20
   perf stat -p <PIDS> -- sleep 10
   ```
3. 查看 P+E 分库热点（注意 Linux 6.8 `perf report --stdio` 的 `--symbol-filter` 在混合架构第二事件上存在失效 Bug，过滤符号请用 `grep`，过滤库可用 `--dso=`）：
   ```bash
   perf --no-pager report --stdio --no-children --no-call-graph -s dso --percent-limit 0.5 -i nav_bench.data
   ```
