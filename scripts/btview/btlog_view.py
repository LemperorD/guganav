#!/usr/bin/env python3
"""解析 Groot2 保存的 .btlog 记录，打印行为树结构与每次 tick 的执行路径。

用途是回答"这一轮 tick 到底走了哪一支"。行为树的节点在几微秒内跑完就回到
IDLE，实时界面看到的永远是某一瞬间的静态状态，看不出执行顺序；而 btlog 里
记的是带时间戳的状态迁移，按时间排开就是完整路径。

用法：
    scripts/btview/btlog_view.py <file.btlog>              树结构 + 每次 tick 一行摘要
    scripts/btview/btlog_view.py <file.btlog> --tick 7     第 7 次 tick 的完整事件序列
    scripts/btview/btlog_view.py <file.btlog> --verbose    所有 tick 都展开
    scripts/btview/btlog_view.py <file.btlog> --tree-only  只看树结构
    scripts/btview/btlog_view.py <file.btlog> --no-color   不着色（重定向到文件时用）

文件格式（BT.CPP v4 的 FileLogger2）：
    魔数 "BTCPP4-FileLogger2"(18) + 版本(1) + XML 长度(4, 小端)
    + 树 XML + 时间基准(8) + 若干 9 字节记录：时间戳(6) + 节点UID(2) + 状态(1)
XML 里每个节点带 _uid 属性，SubTree 节点另有 _fullpath，两者合起来足以还原
展开后的整棵树。
"""

import argparse
import struct
import sys
import xml.etree.ElementTree as ET

MAGIC = b"BTCPP4-FileLogger2"
STATUS_NAMES = {0: "IDLE", 1: "RUNNING", 2: "SUCCESS", 3: "FAILURE"}
STATUS_COLORS = {
    0: "\033[90m",   # 灰
    1: "\033[33m",   # 黄：运行中
    2: "\033[32m",   # 绿：成功
    3: "\033[31m",   # 红：失败
}
RESET = "\033[0m"
PORT_TAGS = {"input_port", "output_port", "inout_port"}


def parse_btlog(path):
    """返回 (xml_text, time_base, records)；records 为 (ts_us, uid, status)。"""
    with open(path, "rb") as handle:
        data = handle.read()

    if data[: len(MAGIC)] != MAGIC:
        raise SystemExit(f"不是 FileLogger2 记录：文件头为 {data[:18]!r}")

    xml_len = struct.unpack_from("<I", data, len(MAGIC) + 1)[0]
    xml_start = len(MAGIC) + 1 + 4
    xml_text = data[xml_start : xml_start + xml_len].decode("utf-8", "replace")

    rec_start = xml_start + xml_len
    time_base = struct.unpack_from("<Q", data, rec_start)[0]
    body = data[rec_start + 8 :]
    count = len(body) // 9
    if len(body) % 9:
        print(f"警告：记录区有 {len(body) % 9} 字节余数，可能有新字段", file=sys.stderr)

    records = []
    for i in range(count):
        chunk = body[i * 9 : (i + 1) * 9]
        records.append(
            (
                int.from_bytes(chunk[0:6], "little"),
                int.from_bytes(chunk[6:8], "little"),
                chunk[8],
            )
        )
    return xml_text, time_base, records


def build_tree(xml_text):
    """从 XML 还原节点表与各 BehaviorTree 的根 uid。

    nodes[uid] = {"uid","label","tag","subtree","path","children","leaf"}
    leaf 表示这个节点是实际干活的叶子（动作或条件）：既没有 XML 子元素，
    也不是 SubTree。摘要里只列这些，才不会把控制节点也报一遍。

    同一个子行为树被多处调用时，日志 XML 里会出现多个同名 <BehaviorTree>，
    只有 _fullpath 能区分（例如 Decide::3/CaptureCenterArea::7/NavigateAndTrigger::13
    与 Decide::3/FallBack::19/NavigateAndTrigger::20）。所以 SubTree 节点与子树的
    连线必须按 _fullpath 匹配：按 ID 匹配会让所有同名调用点连到同一份定义上，
    另一份里的节点就与树脱开，界面上表现为"同名节点聚成一堆"、光条走不到。
    """
    root = ET.fromstring(xml_text)
    nodes = {}
    roots = {}
    instances = {}

    for behavior_tree in root.findall("BehaviorTree"):
        tree_id = behavior_tree.get("ID")
        fullpath = behavior_tree.get("_fullpath") or tree_id
        first = None
        for child in behavior_tree:
            if child.tag in PORT_TAGS:
                continue
            if first is None:
                first = int(child.get("_uid"))
            _collect(child, nodes)
        # 同名时后者覆盖前者：roots 只用于按 ID 找入口树（MainLoop 是唯一的），
        # 子树连接一律走 instances。
        roots[tree_id] = first
        instances[fullpath] = first

    # SubTree 节点没有 XML 子元素，它的唯一子节点按自己的 _fullpath 找。
    for info in nodes.values():
        if info["subtree"] and not info["children"]:
            target = instances.get(info["path"])
            if target is not None:
                info["children"] = [target]

    return nodes, roots


def _collect(element, nodes):
    uid = int(element.get("_uid"))
    children = [c for c in element if c.tag not in PORT_TAGS]
    is_subtree = element.tag == "SubTree"
    nodes[uid] = {
        "uid": uid,
        "label": element.get("ID") if is_subtree else (element.get("name") or element.tag),
        "tag": element.tag,
        "subtree": element.get("ID") if is_subtree else None,
        "path": element.get("_fullpath"),
        "children": [int(c.get("_uid")) for c in children],
        "leaf": not children and not is_subtree,
    }
    for child in children:
        _collect(child, nodes)


def reachable(nodes, root_uid):
    """从入口根节点出发能走到的 uid 集合，顺序与树一致。

    日志里所有节点都在一起，而已展开的树是从入口根连通的。用不到这个集合的
    话，脱开的节点在界面上会因为没有坐标而堆在左上角。
    """
    seen, stack, order = set(), [root_uid], []
    while stack:
        uid = stack.pop()
        if uid in seen or uid not in nodes:
            continue
        seen.add(uid)
        order.append(uid)
        stack.extend(reversed(nodes[uid]["children"]))
    return order



def tick_marker(nodes, root_uid):
    """返回入口根节点的第一个孩子：每轮第一个被求值的节点。

    只有根节点不够用：根节点报告 RUNNING 后会跨 tick 一直保持这个状态（分支里
    有节点在等，例如巡逻等超时），不再产生边界。第一个孩子每轮都会重新求值，
    状态必然从 IDLE 出发，两种情况下都能当边界。没有孩子时返回根节点自己。
    """
    info = nodes.get(root_uid)
    if info and info["children"]:
        return info["children"][0]
    return root_uid


def split_ticks(records, root_uid, marker_uid=None):
    """按 tick 切分事件序列。

    边界取"根节点或它的第一个孩子离开 IDLE"的那一刻：
      - 立即返回终态的动作（例如 ROS2Wrapper，onStart 里就 SUCCESS）每轮只记
        SUCCESS、IDLE，没有 RUNNING 记录，只能靠"离开 IDLE"认出来；
      - 控制节点每轮记 RUNNING、终态、IDLE，两种都能认。
    根节点跨 tick 停在 RUNNING 时（有节点在等）只有第一个孩子会产生边界；正常
    情况下两者都会产生，于是在一轮的开头会多出一个"只有根节点自己的事件"的片段，
    这种片段并回后一轮，不算独立的一次 tick。
    """
    if not records:
        return []
    if marker_uid is None:
        marker_uid = root_uid

    IDLE = 0
    ticks, current = [], []
    statuses = {root_uid: IDLE, marker_uid: IDLE}
    for ts, uid, status in records:
        if uid in statuses:
            started = statuses[uid] == IDLE and status != IDLE
            statuses[uid] = status
            if started and current:
                only_root = all(u == root_uid for _t, u, _s in current)
                if not only_root:
                    ticks.append(current)
                    current = []
        current.append((ts, uid, status))
    if current:
        ticks.append(current)
    return ticks


def summarize(tick, nodes):
    """概括一次 tick：走到的叶子节点及其结果，以及经过的 SubTree 链。

    同一节点在一轮里可能出现两次（先 SUCCESS 再 IDLE），只取第一次。
    """
    leaves, subtrees = [], []
    seen = set()
    for _, uid, status in tick:
        info = nodes.get(uid)
        if info is None or uid in seen:
            continue
        seen.add(uid)
        if info["leaf"]:
            leaves.append((uid, info["label"], status))
        if info["subtree"]:
            # 子树节点在出栈时才记终态，事件顺序是从里往外的，按树深排回
            # 从外到里，读起来才是"经过哪几层子树"。
            subtrees.append(((info["path"] or "").count("/"), info["subtree"]))

    subtrees.sort(key=lambda item: item[0])
    chain = []
    for _depth, name in subtrees:
        if name not in chain:
            chain.append(name)
    return leaves, chain


def format_summary(leaves, subtrees, color):
    counts = {}
    for _uid, label, _status in leaves:
        counts[label] = counts.get(label, 0) + 1

    parts = []
    for uid, label, status in leaves:
        shown = f"{label}#{uid}" if counts[label] > 1 else label
        name = STATUS_NAMES.get(status, str(status))
        paint = STATUS_COLORS.get(status, "") if color else ""
        reset = RESET if color else ""
        parts.append(f"{shown}={paint}{name}{reset}")

    path = " > ".join(subtrees) if subtrees else "-"
    return f"{'  '.join(parts) or '-'}   [{path}]"


def print_tree(nodes, roots, color, main_tree):
    visited = set()

    def walk(uid, prefix, is_last, is_root=False):
        info = nodes.get(uid)
        if info is None:
            return
        connector = "" if is_root else ("└─ " if is_last else "├─ ")
        label = info["label"]
        if info["subtree"]:
            label = f"SubTree ──→ {info['subtree']}"
        print(f"{prefix}{connector}{uid:>3} {label}")

        if uid in visited:
            print(f"{prefix}{'   ' if (is_last or is_root) else '│  '}(已展开，略)")
            return
        visited.add(uid)

        next_prefix = prefix if is_root else prefix + ("   " if is_last else "│  ")
        children = info["children"]
        for index, child in enumerate(children):
            walk(child, next_prefix, index == len(children) - 1)

    entry = roots.get(main_tree)
    if entry is None:
        print(f"未找到入口树 {main_tree}，可选：{', '.join(roots)}", file=sys.stderr)
        return
    walk(entry, "", True, is_root=True)


def print_tick_detail(index, tick, nodes, color):
    first_ts = tick[0][0]
    span = tick[-1][0] - first_ts
    leaves, subtrees = summarize(tick, nodes)
    print(f"tick {index}: 起于 t+{first_ts / 1000:.3f}ms，跨度 {span}us，共 {len(tick)} 条事件")
    print(f"  {format_summary(leaves, subtrees, color)}")
    print("  完整事件序列：")
    for ts, uid, status in tick:
        info = nodes.get(uid, {})
        label = info.get("label", "?")
        name = STATUS_NAMES.get(status, str(status))
        paint = STATUS_COLORS.get(status, "") if color else ""
        reset = RESET if color else ""
        print(f"    +{ts - first_ts:>5}us  uid={uid:>3}  {label:<26}{paint}{name}{reset}")


def main():
    parser = argparse.ArgumentParser(
        description="解析 .btlog 记录，查看树结构与每次 tick 的执行路径",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("logfile", help=".btlog 文件路径")
    parser.add_argument("--tick", type=int, help="只详细显示第 N 次 tick")
    parser.add_argument("--verbose", action="store_true", help="每个 tick 都展开事件序列")
    parser.add_argument("--tree-only", action="store_true", help="只打印树结构")
    parser.add_argument("--no-color", action="store_true", help="不着色")
    parser.add_argument("--main-tree", default="MainLoop", help="入口树 ID，默认 MainLoop")
    args = parser.parse_args()

    xml_text, _time_base, records = parse_btlog(args.logfile)
    nodes, roots = build_tree(xml_text)
    entry_root = roots.get(args.main_tree)
    if entry_root is None:
        # 找不到入口树时退回"出现过的最小 uid"，至少还能切出东西来。
        entry_root = min(nodes) if nodes else None
    ticks = split_ticks(records, entry_root, tick_marker(nodes, entry_root))
    color = not args.no_color and sys.stdout.isatty()

    print(f"记录文件 : {args.logfile}")
    print(f"节点数量 : {len(nodes)}   行为树: {', '.join(roots)}")
    if entry_root is not None:
        linked = reachable(nodes, entry_root)
        if len(linked) != len(nodes):
            # 日志里节点是平铺的：同名子行为树的每一份实例都在。连不上的说明
            # 子树连接出了问题，下面的树里看不到它们。
            print(f"注意     : {len(nodes) - len(linked)} 个节点连不到入口树，不会出现在树结构里")
    print(f"事件数量 : {len(records)}   切分出 {len(ticks)} 次 tick")
    if ticks:
        span = ticks[-1][-1][0] - ticks[0][0][0]
        print(f"覆盖时长 : {span / 1000:.1f} ms")
    print()

    print("=== 树结构 ===")
    print_tree(nodes, roots, color, args.main_tree)

    if args.tree_only:
        return

    print()
    if args.tick is not None:
        if not 1 <= args.tick <= len(ticks):
            raise SystemExit(f"--tick 取值范围 1..{len(ticks)}")
        print(f"=== 第 {args.tick} 次 tick ===")
        print_tick_detail(args.tick, ticks[args.tick - 1], nodes, color)
        return

    print("=== 每次 tick 的执行路径 ===")
    if args.verbose:
        for index, tick in enumerate(ticks, 1):
            print_tick_detail(index, tick, nodes, color)
        return

    print("  （连续走同一支的 tick 合并为一行；只列实际干活的叶子节点）")
    runs = []
    for index, tick in enumerate(ticks, 1):
        leaves, subtrees = summarize(tick, nodes)
        key = (tuple((label, status) for _uid, label, status in leaves), tuple(subtrees))
        if runs and runs[-1][0] == key:
            runs[-1][2] = index
        else:
            runs.append([key, index, index, tick, leaves, subtrees])

    for _key, start, end, tick, leaves, subtrees in runs:
        span_label = f"{start}" if start == end else f"{start}-{end}"
        count = end - start + 1
        times = f"{count:>3} 次" if count > 1 else "      "
        print(f"tick {span_label:>7}  t+{tick[0][0] / 1000:>9.3f}ms  {times}  "
              f"{format_summary(leaves, subtrees, color)}")


if __name__ == "__main__":
    main()
