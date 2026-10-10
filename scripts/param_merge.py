#!/usr/bin/env python3
"""打印某个模式下三层参数文件（base → controller → planner）合并后的结果。

用于结构性重构时的等价性检查：合并语义与 launch 中的 RewrittenYaml 一致——
后加载的文件按键覆盖先加载的文件，嵌套字典递归合并。忽略 <robot_namespace>
替换与 use_sim_time 重写，因为两侧处理方式相同。

用法：
    python3 scripts/param_merge.py reality mppi jps            # 读工作区文件
    python3 scripts/param_merge.py reality mppi jps --rev HEAD  # 读某个 git 版本
    python3 scripts/param_merge.py reality mppi jps --node global_costmap

    # 等价性比较
    python3 scripts/param_merge.py reality mppi jps > /tmp/a.json
    python3 scripts/param_merge.py reality mppi jps --rev HEAD > /tmp/b.json
    diff /tmp/a.json /tmp/b.json
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_ROOT = "src/guga_bringup/config"


def read_yaml(rel_path: str, rev: str | None) -> dict:
    if rev is None:
        return yaml.safe_load((REPO_ROOT / rel_path).read_text()) or {}
    text = subprocess.run(
        ["git", "show", f"{rev}:{rel_path}"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return yaml.safe_load(text) or {}


def deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def merge_layers(mode: str, controller: str, planner: str, rev: str | None) -> dict:
    files = [
        f"{CONFIG_ROOT}/{mode}/base.yaml",
        f"{CONFIG_ROOT}/{mode}/controller/{controller}.yaml",
        f"{CONFIG_ROOT}/{mode}/planner/{planner}.yaml",
    ]
    merged: dict = {}
    for rel in files:
        merged = deep_merge(merged, read_yaml(rel, rev))
    return merged


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["reality", "simulation"])
    parser.add_argument("controller")
    parser.add_argument("planner")
    parser.add_argument("--rev", default=None, help="从该 git 版本读取三个参数文件")
    parser.add_argument("--node", default=None, help="只打印该节点段")
    args = parser.parse_args()

    merged = merge_layers(args.mode, args.controller, args.planner, args.rev)
    if args.node:
        node = None
        for value in merged.values():
            if isinstance(value, dict) and args.node in value:
                node = value[args.node]
                break
        if node is None:
            print(f"未找到节点段：{args.node}", file=sys.stderr)
            return 1
        merged = {args.node: node}

    print(json.dumps(merged, indent=2, sort_keys=True, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
