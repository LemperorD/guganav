#!/usr/bin/env python3
"""把各包的编译数据库合并成工作区级别的 compile_commands.json。

colcon 的 compile_commands 事件处理器只汇总本次构建涉及的包。选择性构建
（例如测试脚本里的 --packages-select）会把 build/compile_commands.json
覆盖成子集，clangd 随后就找不到其他包的头文件与编译参数。这个脚本把
build/*/compile_commands.json 合并回去，按源文件路径去重。

用法：python3 scripts/ci/merge_compile_commands.py
"""

import json
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
BUILD_DIR = WORKSPACE / "build"
OUTPUT = BUILD_DIR / "compile_commands.json"


def main() -> int:
    merged = {}
    package_dbs = sorted(BUILD_DIR.glob("*/compile_commands.json"))
    for db in package_dbs:
        try:
            entries = json.loads(db.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        for entry in entries:
            merged[entry["file"]] = entry

    OUTPUT.write_text(
        json.dumps(
            sorted(merged.values(), key=lambda entry: entry["file"]),
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    print(
        f"已合并 {len(merged)} 条编译命令（{len(package_dbs)} 个包）到 {OUTPUT}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
