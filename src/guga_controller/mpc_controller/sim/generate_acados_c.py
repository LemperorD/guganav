#!/usr/bin/env python3
"""
生成 acados Model A + Model B 的 C 代码并打包到 src/acados/ 目录。
"""
import sys, os, shutil, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from mpc_solver import MpcSolver
from mpc_solver_augmented import AugmentedMpcSolver

sim_dir = os.path.dirname(__file__)
target_dir = os.path.join(os.path.dirname(sim_dir), 'src', 'acados')
os.makedirs(target_dir, exist_ok=True)

N, dt = 15, 0.05
Q, R, Rd = [10.0, 10.0, 2.0], [0.1, 0.1, 0.05], [0.5, 0.5, 0.3]
u_min, u_max = [-3.0, -3.0, -6.0], [3.0, 3.0, 6.0]

# Generate Model A
print("[Model A] Generating...")
_ma = MpcSolver(N, dt, Q, R, Rd, u_min, u_max)
model_a_dir = os.path.join(target_dir, 'model_a')
os.makedirs(model_a_dir, exist_ok=True)
for sub in ['c_generated_code', 'omni_mpc_cost', 'omni_mpc_model']:
    src = os.path.join(sim_dir, sub)
    dst = os.path.join(model_a_dir, sub)
    if os.path.exists(src):
        if os.path.exists(dst): shutil.rmtree(dst)
        shutil.copytree(src, dst)
        print(f"  {sub} -> {dst}")

# Generate Model B
print("[Model B] Generating...")
w_err, w_ctrl, w_dctrl, w_term = [10.0, 2.0], [0.1, 0.1, 0.05], [0.5, 0.5, 0.3], 100.0
_mb = AugmentedMpcSolver(N, dt, w_err, w_ctrl, w_dctrl, w_term, u_min, u_max)
model_b_dir = os.path.join(target_dir, 'model_b')
os.makedirs(model_b_dir, exist_ok=True)
for sub in ['c_generated_code', 'omni_mpc_aug_cost', 'omni_mpc_aug_model']:
    src = os.path.join(sim_dir, sub)
    dst = os.path.join(model_b_dir, sub)
    if os.path.exists(src):
        if os.path.exists(dst): shutil.rmtree(dst)
        shutil.copytree(src, dst)
        print(f"  {sub} -> {dst}")

print(f"\nDone! C code in {target_dir}")
