#!/usr/bin/env python3
"""动态调参 Web UI 的服务端。

与 btview 同一套做法：aiohttp 提供页面与接口，rclpy 直接调
<node>/get_parameters 与 <node>/set_parameters 服务，不走 ros2 CLI
（CLI 每次都要做 node graph 查询，逐个读取很慢，某些环境还会报 Node not found）。

读取策略是针对速度做的：
  * 每个目标节点一次 ListParameters + 一次 GetParameters，批量拿回全部参数，
    不做逐参数往返；
  * 结果缓存在服务端，页面加载与刷新只读缓存，不等服务往返；
  * 周期刷新在后台线程里跑（默认 2 Hz），通过 WebSocket 推给页面。

用法：
    scripts/paramview/paramview_server.py                        # 默认 controller_server:FollowPath
    scripts/paramview/paramview_server.py --list scripts/paramview/params_list/pid_para.txt \\
        --target controller_server:FollowPath --port 8090
    scripts/paramview/paramview_server.py --target global_costmap/global_costmap:inflation_layer \\
        --target planner_server:GridBased

浏览器打开 http://127.0.0.1:8090。

接口：
    GET  /api/params            返回缓存快照（不触发服务调用）
    POST /api/set                {"target":..., "name":..., "value":...} 写单个参数
    POST /api/save              把与配置不同的参数写回 yaml（下次启动生效）
    POST /api/refresh           立即重读一次
    WS   /ws                    周期性推送 {"type":"params", ...}，接收 {"type":"set_param", ...}
"""

import argparse
import asyncio
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

from aiohttp import web

HERE = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))


def parse_param_lists(patterns):
    """读取 params_list/*.txt，返回 {参数名: 说明}。行格式：  Name|说明"""
    descriptions = {}
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            with open(path, "r", encoding="utf-8") as handle:
                for line in handle:
                    line = line.strip()
                    if not line or "|" not in line:
                        continue
                    name, desc = line.split("|", 1)
                    name = name.strip().lstrip("/")
                    if name:
                        descriptions[name] = desc.strip()
    return descriptions


def load_baseline(mode, controller, planner):
    """用 scripts/paramview/param_merge.py 的合并结果作为"配置值"基线。

    返回 {节点路径: {参数名: 值}}，节点路径形如 controller_server、
    global_costmap/global_costmap —— 后者的 yaml 是两层同名嵌套，这里拍平。
    """
    if not (mode and controller and planner):
        return {}
    script = os.path.join(WS_ROOT, "scripts", "paramview", "param_merge.py")
    if not os.path.exists(script):
        return {}
    proc = subprocess.run(
        [sys.executable, script, mode, controller, planner],
        capture_output=True, text=True,
    )
    if proc.returncode != 0:
        print(f"读取参数基线失败：{proc.stderr.strip()[:200]}", file=sys.stderr)
        return {}
    try:
        merged = json.loads(proc.stdout)
    except json.JSONDecodeError:
        return {}

    def flatten(prefix, mapping, out):
        """yaml 的嵌套映射对应带点的参数名：FollowPath: {vx_max: 2.5} → FollowPath.vx_max"""
        for name, value in mapping.items():
            full = f"{prefix}.{name}" if prefix else name
            if isinstance(value, dict):
                flatten(full, value, out)
            else:
                out[full] = value

    flat = {}
    for key, value in merged.items():
        if not isinstance(value, dict):
            continue
        if "ros__parameters" in value:
            params = {}
            flatten("", value["ros__parameters"], params)
            flat[key] = params
            continue
        for inner_key, inner in value.items():
            if isinstance(inner, dict) and "ros__parameters" in inner:
                params = {}
                flatten("", inner["ros__parameters"], params)
                flat[f"{key}/{inner_key}"] = params
    return flat


# ── 回写配置文件 ─────────────────────────────────────────────────
# 调好的参数要保留到下次启动，就得写回 yaml。这里用行级替换：只改目标行
# "键: 值" 里的值，缩进、键名与行内注释都原样保留。改用 yaml.safe_dump 重新
# 序列化会丢掉文件里的说明性注释（reality/controller/mppi.yaml 有 60 多处），
# 那些注释记录了取值理由，不能丢。写回前每个文件都备份一份。

CONFIG_ROOT = "src/guga_bringup/config"
PLACEHOLDER = "<robot_namespace>"
_YAML_KEYWORDS = {"true", "false", "yes", "no", "on", "off", "null", "none", "~"}
_PLAIN_SCALAR_RE = re.compile(r"^[A-Za-z0-9_./<>@+-]+$")
_NUMBER_RE = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$")
_KEY_RE = re.compile(r"^(?P<indent> *)(?P<key>[^:#\n]+?):(?P<rest>.*)$")


def config_layers(mode, controller, planner):
    """三层参数文件，顺序与 scripts/paramview/param_merge.py 的合并顺序一致。"""
    if not (mode and controller and planner):
        return []
    return [
        os.path.join(WS_ROOT, CONFIG_ROOT, mode, "base.yaml"),
        os.path.join(WS_ROOT, CONFIG_ROOT, mode, "controller", f"{controller}.yaml"),
        os.path.join(WS_ROOT, CONFIG_ROOT, mode, "planner", f"{planner}.yaml"),
    ]


def yaml_paths(text):
    """返回 {键路径: 行号}，用于定位要替换的那一行。

    只按缩进与 "键:" 判断层级，不解析 yaml 语义：解释性注释、行内注释与
    空行都保留在原位，行号也不会因嵌套结构而错位。以 - 开头的列表项不入栈，
    所以列表内部的键不会出现在结果里（这些参数不参与回写）。
    """
    found = {}
    stack = []
    for lineno, line in enumerate(text.splitlines()):
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("-"):
            continue
        match = _KEY_RE.match(line)
        if match is None:
            continue
        indent = len(match.group("indent"))
        key = match.group("key").strip().strip("\"'")
        rest = match.group("rest")
        while stack and stack[-1][0] >= indent:
            stack.pop()
        path = tuple(item[1] for item in stack) + (key,)
        found[path] = lineno
        # 值部分为空说明下面是嵌套映射，这个键要压栈
        if not rest.strip() or rest.lstrip().startswith("#"):
            stack.append((indent, key))
    return found


def split_comment(rest):
    """把 " 值  # 注释" 拆成值与该注释（含值后面原有的空白），引号内的 # 不算注释。"""
    quote = ""
    for index, char in enumerate(rest):
        if quote:
            if char == quote:
                quote = ""
        elif char in "\"'":
            quote = char
        elif char == "#" and index and rest[index - 1] in " \t":
            value = rest[:index].rstrip()
            return value, rest[len(value):]
    return rest.rstrip(), ""


def replace_scalar(line, literal):
    """把 "  键: 旧值  # 注释" 里的值换成 literal，保留缩进、键名与注释。"""
    match = _KEY_RE.match(line)
    if match is None:
        return line
    rest = match.group("rest")
    _, comment = split_comment(rest)
    head = line[: len(line) - len(rest)]
    return f"{head} {literal}{comment}"


def format_scalar(value):
    """按 yaml 标量写法输出当前值。布尔沿用文件里 True/False 的写法。"""
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(format_scalar(item) for item in value) + "]"
    text = str(value)
    # 像数字或 yaml 关键字的字符串必须加引号，否则写回去后类型就变了
    if (
        _PLAIN_SCALAR_RE.match(text)
        and not _NUMBER_RE.match(text)
        and text.lower() not in _YAML_KEYWORDS
    ):
        return text
    return json.dumps(text, ensure_ascii=False)


def namespace_for(graph, node_name):
    """从 ROS 图里取该节点的命名空间。

    yaml 里的 topic 写成 <robot_namespace>/terrain_map，节点加载时会换成带
    命名空间的完整名字；比较与回写都要把这一步还原，否则每个模板参数都会被
    误判成"偏离配置"，保存时还会把实际命名空间写死进配置文件。
    """
    suffix = "/" + node_name.strip("/")
    for full in graph.get("nodes", []):
        if full == suffix or full.endswith(suffix):
            return full[: -len(suffix)]
    return ""


def canonical(value, base, namespace):
    """把运行时值里已展开的命名空间还原成 <robot_namespace> 占位符。

    配置里用占位符的参数，运行时值可能已被节点替换
    （/red_standard_robot1/terrain_map），也可能仍是字面量；两种写法都归一成
    占位符后即可与基线直接比较，回写时也保持模板不被写死。
    """
    if not isinstance(value, str) or not isinstance(base, str) or PLACEHOLDER not in base:
        return value
    if namespace and value.startswith(namespace):
        return PLACEHOLDER + value[len(namespace):]
    return value


def same_value(left, right):
    """比较运行时值与配置值。数值按数值比，避免 1 与 1.0 被判成不同。"""
    if isinstance(left, bool) != isinstance(right, bool):
        return False
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return float(left) == float(right)
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        return len(left) == len(right) and all(
            same_value(a, b) for a, b in zip(left, right)
        )
    return str(left) == str(right)


def detect_controller_profile(plugin_name):
    """根据 controller_server 的 FollowPath.plugin 推断控制器 profile。"""
    if not isinstance(plugin_name, str) or not plugin_name.strip():
        return None
    low = plugin_name.lower()
    if "mppi" in low:
        return "mppi"
    if "pid" in low:
        return "pid"
    if "mpc" in low:
        return "mpc"
    return None


def detect_planner_profile(plugin_name):
    """根据 planner_server 的 GridBased.plugin 推断规划器 profile。"""
    if not isinstance(plugin_name, str) or not plugin_name.strip():
        return None
    low = plugin_name.lower()
    if "jps" in low:
        return "jps"
    if "smacplanner2d" in low or "smac2d" in low:
        return "smac2d"
    if "smacplannerhybrid" in low or "smachybrid" in low:
        return "smachybrid"
    return None


def param_list_for_profile(profile):
    """返回指定控制器 profile 对应的参数说明文件路径（不存在则返回 None）。"""
    if not profile:
        return None
    path = os.path.join(WS_ROOT, "scripts", "paramview", "params_list", f"{profile}_para.txt")
    return path if os.path.exists(path) else None


def resolve_node_in_graph(graph, spec_node):
    """在 ROS 图里解析节点全路径（自动适配仿真命名空间，如 /red_standard_robot1/controller_server）。

    返回 (resolved_full_path, ambiguous_matches)。
    """
    nodes = (graph or {}).get("nodes", [])
    clean = spec_node.strip("/")
    exact = "/" + clean
    if exact in nodes:
        return exact, []
    suffix = "/" + clean
    matched = [n for n in nodes if n.endswith(suffix)]
    if len(matched) == 1:
        return matched[0], []
    if len(matched) > 1:
        return None, matched
    return None, []


class ParamTarget:
    """一个被调参的节点：一次批量读，一次写一个。"""

    def __init__(self, spec, node, descriptions, graph=None):
        if ":" in spec:
            node_name, prefix = spec.split(":", 1)
        else:
            node_name, prefix = spec, ""
        self.spec_node = node_name.strip("/")
        self.node_name = self.spec_node
        self.prefix = prefix.strip()
        self.descriptions = descriptions
        self.graph = graph
        self._node = node
        self.resolved_node = "/" + self.spec_node
        self.resolved_namespace = ""
        self.use_sim_time = None
        self.names = []
        self.values = {}
        self.read_ms = None
        self.error = None
        self.get_client = None
        self.set_client = None
        self.list_client = None
        self._bind_clients(self.resolved_node, wait=False)

    def _bind_clients(self, full_node_path, wait=True):
        if self.get_client is not None and full_node_path == self.resolved_node:
            return
        for client in (self.get_client, self.set_client, self.list_client):
            if client is not None:
                try:
                    self._node.destroy_client(client)
                except Exception:
                    pass
        self.resolved_node = full_node_path
        suffix = "/" + self.node_name
        if full_node_path.endswith(suffix):
            self.resolved_namespace = full_node_path[: -len(suffix)]
        else:
            self.resolved_namespace = ""
        self.get_client = self._node.create_client(
            rcl_interfaces_srv_get(), f"{self.resolved_node}/get_parameters"
        )
        self.set_client = self._node.create_client(
            rcl_interfaces_srv_set(), f"{self.resolved_node}/set_parameters"
        )
        self.list_client = self._node.create_client(
            rcl_interfaces_srv_list(), f"{self.resolved_node}/list_parameters"
        )
        if wait:
            self.get_client.wait_for_service(timeout_sec=0.4)
            self.list_client.wait_for_service(timeout_sec=0.2)

    def config_node_key(self, baseline):
        """用于匹配 baseline 与写回 yaml 的节点键（去掉仿真命名空间前缀）。"""
        if not baseline or self.node_name in baseline:
            return self.node_name
        # 用户显式传入 red_standard_robot1/controller_server 时，还原成 controller_server
        parts = self.node_name.split("/")
        for i in range(1, len(parts)):
            candidate = "/".join(parts[i:])
            if candidate in baseline:
                return candidate
        return self.node_name

    # ── 读取 ─────────────────────────────────────────────────────
    def refresh(self, descriptions_all):
        import rclpy
        from rcl_interfaces.srv import ListParameters, GetParameters

        resolved, ambiguous = resolve_node_in_graph(self.graph, self.spec_node)
        if ambiguous:
            self.error = (
                f"发现多个同名节点：{', '.join(ambiguous[:3])}，"
                "请在 --target 里指定命名空间前缀"
            )
            return
        if resolved and resolved != self.resolved_node:
            self._bind_clients(resolved, wait=True)

        if not self.get_client.service_is_ready():
            hint = ""
            if self.graph is not None:
                matched = [n for n in self.graph.get("nodes", []) if self.spec_node in n]
                if matched:
                    hint = "；图里有同名节点：" + ", ".join(matched[:3]) + "（可能是命名空间不同，--target 里带上前缀）"
                else:
                    hint = (
                        f"；图里共 {len(self.graph.get('nodes', []))} 个节点，没有名字含 "
                        f"{self.spec_node} 的节点（ROS_DOMAIN_ID={self.graph['domain_id']}、"
                        f"ROS_LOCALHOST_ONLY={self.graph['localhost_only']}）"
                    )
            self.error = f"未发现 {self.resolved_node}/get_parameters{hint}"
            return
        started = time.time()
        names = set()
        if self.list_client.service_is_ready():
            request = ListParameters.Request()
            request.prefixes = [self.prefix] if self.prefix else []
            # depth 要够深：nav2 的参数名带点是层级名，最深到 3~4 段
            # （FollowPath.CostCritic.cost_weight、local_costmap.obstacle_layer.terrain_map.topic），
            # depth=2 会漏掉第三段以后的参数
            request.depth = 10
            future = self.list_client.call_async(request)
            if self._wait(future):
                result = future.result().result
                names.update(result.names)
                # 节点把更深的层级当子前缀返回时，再按子前缀取一次
                for sub in result.prefixes:
                    if not sub.startswith(self.prefix):
                        continue
                    request2 = ListParameters.Request()
                    request2.prefixes = [sub]
                    request2.depth = 10
                    future2 = self.list_client.call_async(request2)
                    if self._wait(future2):
                        names.update(future2.result().result.names)
        if not names and descriptions_all:
            # 列表服务拿不到时退回说明文件里声明的名字（节点未声明的会在下面被滤掉）
            for name in descriptions_all:
                if self.prefix and not name.startswith(self.prefix + "."):
                    continue
                if not self.prefix and "." in name:
                    continue
                names.add(name)
        if not names:
            self.error = "该前缀下没有参数（列表服务无返回，且说明文件里没有该前缀的名字）"
            return

        sorted_names = sorted(names)
        query_sim_time = "use_sim_time" not in names
        request = GetParameters.Request()
        request.names = sorted_names + (["use_sim_time"] if query_sim_time else [])
        future = self.get_client.call_async(request)
        if not self._wait(future):
            self.error = "get_parameters 超时"
            return
        values = future.result().values
        if query_sim_time and len(values) == len(sorted_names) + 1:
            sim_val = values[-1]
            if sim_val.type == 1:
                self.use_sim_time = sim_val.bool_value
            values = values[:-1]
        self.values = {}
        kept = []
        for name, value in zip(sorted_names, values):
            if value.type == 0:  # NOT_SET：节点没声明这个名字
                continue
            kept.append(name)
            py_val = parameter_value_to_python(value)
            self.values[name] = py_val
            if name == "use_sim_time" and isinstance(py_val, bool):
                self.use_sim_time = py_val
        self.names = kept
        self.read_ms = (time.time() - started) * 1000.0
        self.error = None

    @staticmethod
    def _wait(future, timeout=2.0):
        import rclpy
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.005)
        return False

    # ── 写入 ─────────────────────────────────────────────────────
    def set_value(self, name, raw):
        from rcl_interfaces.msg import Parameter, ParameterValue
        from rcl_interfaces.srv import SetParameters

        if not self.set_client.service_is_ready():
            return False, f"未发现 {self.resolved_node}/set_parameters"
        if name not in self.names:
            return False, f"{name} 不在本目标的参数表里"

        current = self.values.get(name)
        value = ParameterValue()
        try:
            if isinstance(current, bool):
                value.type = 1
                value.bool_value = str(raw).strip().lower() in ("1", "true", "yes", "on")
            elif isinstance(current, int):
                value.type = 2
                value.integer_value = int(float(raw))
            elif isinstance(current, float):
                value.type = 3
                value.double_value = float(raw)
            elif isinstance(current, str):
                value.type = 4
                value.string_value = str(raw)
            else:
                return False, f"不支持的类型：{type(current).__name__}"
        except (TypeError, ValueError):
            # 输入非法时给可读反馈，不要把异常抛到接口层
            return False, f"无法把 '{raw}' 解析成 {type(current).__name__}"

        request = SetParameters.Request()
        request.parameters = [Parameter(name=name, value=value)]
        future = self.set_client.call_async(request)
        if not self._wait(future):
            return False, "set_parameters 超时"
        result = future.result().results[0]
        if not result.successful:
            return False, result.reason or "节点拒绝了该值"
        if isinstance(current, (int, float)) and not isinstance(current, bool):
            # 立刻用新值更新缓存，避免界面在下一个刷新周期之前回跳
            self.values[name] = value.double_value if value.type == 3 else value.integer_value
        else:
            self.values[name] = (
                value.bool_value if value.type == 1
                else value.string_value if value.type == 4
                else current
            )
        return True, "已写入"

    def snapshot(self, baseline=None):
        entries = []
        for name in self.names:
            entries.append(
                {
                    "name": name,
                    "short": name[len(self.prefix) + 1:] if self.prefix else name,
                    "value": self.values.get(name),
                    "description": self.descriptions.get(name, ""),
                }
            )
        config_key = self.config_node_key(baseline)
        ns = self.resolved_namespace or namespace_for(self.graph or {}, config_key)
        display_node = self.resolved_node.lstrip("/")
        return {
            "target": f"{self.spec_node}:{self.prefix}" if self.prefix else self.spec_node,
            "display_target": f"{display_node}:{self.prefix}" if self.prefix else display_node,
            "node": config_key,
            "resolved_node": self.resolved_node,
            "prefix": self.prefix,
            "namespace": ns,
            "read_ms": self.read_ms,
            "error": self.error,
            "params": entries,
        }


def parameter_value_to_python(value):
    kind = value.type
    if kind == 1:
        return value.bool_value
    if kind == 2:
        return value.integer_value
    if kind == 3:
        return value.double_value
    if kind == 4:
        return value.string_value
    if kind == 5:
        return list(value.byte_array_value)
    if kind == 6:
        return list(value.bool_array_value)
    if kind == 7:
        return list(value.integer_array_value)
    if kind == 8:
        return list(value.double_array_value)
    if kind == 9:
        return list(value.string_array_value)
    return None


_RCLPY_NODE = None


def make_node():
    global _RCLPY_NODE
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=[])
    _RCLPY_NODE = Node("paramview")
    thread = threading.Thread(target=rclpy.spin, args=(_RCLPY_NODE,), daemon=True)
    thread.start()
    return _RCLPY_NODE


def rcl_interfaces_srv_get():
    from rcl_interfaces.srv import GetParameters

    return GetParameters


def rcl_interfaces_srv_set():
    from rcl_interfaces.srv import SetParameters

    return SetParameters


def rcl_interfaces_srv_list():
    from rcl_interfaces.srv import ListParameters

    return ListParameters


def graph_summary(node):
    """服务端当前能看到的 ROS 图。用于把"未发现服务"分成
    发现不到节点（DDS/域/网络）与名字写错（命名空间）两种。"""
    nodes = []
    try:
        for name, namespace in node.get_node_names_and_namespaces():
            full = f"{namespace.rstrip('/')}/{name}" if namespace not in ("", "/") else f"/{name}"
            nodes.append(full)
    except Exception:
        pass
    nodes.sort()
    return {
        "nodes": nodes,
        "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "默认"),
    }


class App:
    def __init__(self, targets, node, hz, baseline_args=None, writable=True, auto_list=False):
        self.targets = targets
        self.node = node
        self.hz = hz
        raw_mode, raw_ctrl, raw_planner = baseline_args or (None, None, None)
        self.auto_mode = raw_mode == "auto"
        self.auto_controller = raw_ctrl == "auto"
        self.auto_planner = raw_planner == "auto"
        self.auto_list = auto_list
        self.baseline_args = (
            None if self.auto_mode else raw_mode,
            None if self.auto_controller else raw_ctrl,
            None if self.auto_planner else raw_planner,
        )
        self.detected_profile = None if self.auto_controller else raw_ctrl
        self._planner_client = None
        self._planner_node_path = None
        self._layer_mtimes = None
        self.baseline_version = 1
        self.baseline = load_baseline(*self.baseline_args) if all(self.baseline_args) else {}
        self.layer_files = config_layers(*self.baseline_args) if all(self.baseline_args) else []
        self.writable = writable
        self.clients = set()
        self.lock = threading.Lock()
        self.last_refresh = 0.0

    def _query_planner_plugin(self, graph):
        from rcl_interfaces.srv import GetParameters

        resolved, ambiguous = resolve_node_in_graph(graph, "planner_server")
        if not resolved or ambiguous:
            return None
        if self._planner_client is None or self._planner_node_path != resolved:
            if self._planner_client is not None:
                try:
                    self.node.destroy_client(self._planner_client)
                except Exception:
                    pass
            self._planner_node_path = resolved
            self._planner_client = self.node.create_client(
                GetParameters, f"{resolved}/get_parameters"
            )
            self._planner_client.wait_for_service(timeout_sec=0.2)
        if not self._planner_client.service_is_ready():
            return None
        req = GetParameters.Request()
        req.names = ["GridBased.plugin"]
        future = self._planner_client.call_async(req)
        if not ParamTarget._wait(future, timeout=0.5):
            return None
        vals = future.result().values
        if vals and vals[0].type == 4:
            return detect_planner_profile(vals[0].string_value)
        return None

    def _layers_mtime_signature(self, files):
        sig = []
        for path in files:
            try:
                sig.append((path, os.path.getmtime(path)))
            except OSError:
                sig.append((path, None))
        return tuple(sig)

    def _auto_detect_runtime(self, graph):
        mode, ctrl, planner = self.baseline_args
        new_mode, new_ctrl, new_planner = mode, ctrl, planner

        for target in self.targets:
            if target.error:
                continue
            if self.auto_mode:
                if target.use_sim_time is True:
                    new_mode = "simulation"
                elif target.use_sim_time is False:
                    new_mode = "reality"
                elif target.resolved_namespace:
                    new_mode = "simulation"
            if self.auto_controller:
                plugin = target.values.get("FollowPath.plugin")
                detected = detect_controller_profile(plugin)
                if detected:
                    new_ctrl = detected
            if self.auto_planner:
                plugin = target.values.get("GridBased.plugin")
                detected_p = detect_planner_profile(plugin)
                if detected_p:
                    new_planner = detected_p

        if self.auto_planner and not new_planner:
            detected_p = self._query_planner_plugin(graph)
            if detected_p:
                new_planner = detected_p
            elif new_mode and new_ctrl:
                # planner_server 尚未就绪时先回落默认 jps，待就绪后再更新
                new_planner = "jps"

        # 控制器 profile 识别后自动切换参数说明文件
        if self.auto_list and new_ctrl and new_ctrl != self.detected_profile:
            list_path = param_list_for_profile(new_ctrl)
            if list_path:
                descs = parse_param_lists([list_path])
                for target in self.targets:
                    target.descriptions = descs
                self.detected_profile = new_ctrl
                print(
                    f"自动加载参数说明: {os.path.relpath(list_path, WS_ROOT)} ({len(descs)} 项)",
                    flush=True,
                )

        new_args = (new_mode, new_ctrl, new_planner)
        changed = new_args != self.baseline_args
        if changed:
            self.baseline_args = new_args
            self.layer_files = config_layers(*new_args) if all(new_args) else []
            if all(new_args):
                self.baseline = load_baseline(*new_args)
                self._layer_mtimes = self._layers_mtime_signature(self.layer_files)
                self.baseline_version += 1
                print(
                    f"自动识别配置基线: mode={new_mode}, controller={new_ctrl}, planner={new_planner} "
                    f"-> {'、'.join(os.path.relpath(p, WS_ROOT) for p in self.layer_files)}",
                    flush=True,
                )
        elif all(self.baseline_args):
            sig = self._layers_mtime_signature(self.layer_files)
            if sig != self._layer_mtimes:
                self.baseline = load_baseline(*self.baseline_args)
                self._layer_mtimes = sig
                self.baseline_version += 1

    def snapshot(self):
        with self.lock:
            mode, ctrl, planner = self.baseline_args
            return {
                "type": "params",
                "stamp": self.last_refresh,
                "writable": self.writable,
                "baseline_version": self.baseline_version,
                "meta": {"mode": mode, "controller": ctrl, "planner": planner},
                "graph": graph_summary(self.node),
                "targets": [target.snapshot(self.baseline) for target in self.targets],
            }

    def refresh_once(self):
        graph = graph_summary(self.node)
        for target in self.targets:
            target.graph = graph
            try:
                target.refresh(target.descriptions)
            except Exception as exc:  # 单个目标出错不应影响其它目标
                target.error = f"{type(exc).__name__}: {exc}"
        with self.lock:
            self._auto_detect_runtime(graph)
        self.last_refresh = time.time()

    def save_params(self):
        """把与配置基线不同的参数写回 yaml，返回接口用的结果字典。

        只处理"配置文件里有、且当前值不同"的参数：节点声明了但配置文件没有
        的名字用的是节点默认值，写进去会改变文件结构，一律跳过并报告。
        """
        if not self.writable:
            return {"ok": False, "msg": "只读模式启动，未写回配置文件"}
        if not self.layer_files:
            return {
                "ok": False,
                "msg": "尚未识别到配置模式/控制器/规划器，请先等待节点连接或手动指定 --mode/--profile/--planner",
            }

        with self.lock:
            texts = {}
            index = {}
            for path in self.layer_files:
                if not os.path.exists(path):
                    return {"ok": False, "msg": f"参数文件不存在：{path}"}
                with open(path, encoding="utf-8") as handle:
                    texts[path] = handle.read()
                index[path] = yaml_paths(texts[path])

            edits = {}
            skipped = []
            for target in self.targets:
                config_key = target.config_node_key(self.baseline)
                namespace = target.resolved_namespace or namespace_for(target.graph or {}, config_key)
                node_base = self.baseline.get(config_key) or {}
                for name in target.names:
                    if name not in node_base:
                        continue
                    base = node_base[name]
                    current = canonical(target.values.get(name), base, namespace)
                    if same_value(canonical(base, base, namespace), current):
                        continue
                    # 键路径：节点段 + ros__parameters + 参数名的层级段
                    key_path = (
                        tuple(config_key.split("/"))
                        + ("ros__parameters",)
                        + tuple(name.split("."))
                    )
                    # 合并语义是后者覆盖前者，改动就写到最后出现该参数的文件里
                    path = None
                    for candidate in reversed(self.layer_files):
                        if key_path in index[candidate]:
                            path = candidate
                            break
                    if path is None:
                        skipped.append(
                            {"name": name, "reason": "参数文件里没有这个名字，写回会改变文件结构"}
                        )
                        continue
                    edits.setdefault(path, {})[key_path] = (name, current)

            if not edits:
                return {
                    "ok": True,
                    "msg": "当前值与配置一致，没有需要写回的参数",
                    "files": [],
                    "skipped": skipped,
                }

            stamp = time.strftime("%Y%m%d-%H%M%S")
            # 同一秒内保存两次时另起一个目录，避免后一次覆盖前一次的备份
            while os.path.exists(os.path.join(WS_ROOT, "log", "paramview_backup", stamp)):
                stamp += "-1"
            written = []
            for path, items in edits.items():
                lines = texts[path].splitlines(keepends=True)
                for key_path, (_, value) in items.items():
                    lineno = index[path][key_path]
                    line = lines[lineno]
                    newline = "\n" if line.endswith("\n") else ""
                    lines[lineno] = replace_scalar(line.rstrip("\n"), format_scalar(value)) + newline
                backup = os.path.join(
                    WS_ROOT, "log", "paramview_backup", stamp,
                    os.path.relpath(path, WS_ROOT),
                )
                os.makedirs(os.path.dirname(backup), exist_ok=True)
                shutil.copy2(path, backup)
                with open(path, "w", encoding="utf-8") as handle:
                    handle.write("".join(lines))
                written.append(
                    {
                        "path": os.path.relpath(path, WS_ROOT),
                        "count": len(items),
                        "backup": os.path.relpath(backup, WS_ROOT),
                        "params": sorted(name for name, _ in items.values()),
                    }
                )
            # 基线跟着更新：写回之后这些参数就不再是"偏离配置"了
            self.baseline = load_baseline(*self.baseline_args)
            self._layer_mtimes = self._layers_mtime_signature(self.layer_files)
            self.baseline_version += 1

        total = sum(item["count"] for item in written)
        msg = "已写回 {} 个参数（{}），下次启动生效".format(
            total, "；".join(f"{item['path']} {item['count']} 项" for item in written)
        )
        if skipped:
            msg += "。跳过 {} 个：{}".format(
                len(skipped), "；".join(f"{item['name']}（{item['reason']}）" for item in skipped)
            )
        return {"ok": True, "msg": msg, "files": written, "skipped": skipped}

    def refresh_loop(self):
        period = 1.0 / self.hz if self.hz > 0 else 0.5
        while True:
            self.refresh_once()
            time.sleep(period)


def make_app(args):
    auto_list = not bool(args.list)
    initial_list = args.list or [
        os.path.join(WS_ROOT, "scripts/paramview/params_list/mppi_para.txt")
    ]
    descriptions_all = parse_param_lists(initial_list)
    node = make_node()
    targets = [ParamTarget(spec, node, descriptions_all) for spec in args.target]
    app_state = App(
        targets, node, args.hz,
        (args.baseline_mode, args.baseline_controller, args.baseline_planner),
        writable=not args.no_save,
        auto_list=auto_list,
    )

    # 不在这里等每个服务就绪：目标节点的 3 个服务各等一次会让页面迟迟打不开，
    # 而"读取慢"正是要解决的问题。先绑端口、后台刷新线程自己发现服务，
    # 未就绪时界面上显示"未发现服务"，服务出现后自动填充。
    # 首次读取交给后台线程：绑定端口不受服务发现或超时影响
    threading.Thread(target=app_state.refresh_loop, daemon=True).start()

    routes = web.RouteTableDef()
    html_path = os.path.join(HERE, "paramview.html")

    @routes.get("/")
    async def index(_request):
        return web.FileResponse(html_path)

    @routes.get("/api/graph")
    async def api_graph(_request):
        return web.json_response(graph_summary(node))

    @routes.get("/api/baseline")
    async def api_baseline(_request):
        return web.json_response(app_state.baseline)

    @routes.get("/api/params")
    async def api_params(_request):
        return web.json_response(app_state.snapshot())

    @routes.post("/api/refresh")
    async def api_refresh(_request):
        await asyncio.get_event_loop().run_in_executor(None, app_state.refresh_once)
        return web.json_response(app_state.snapshot())

    @routes.post("/api/save")
    async def api_save(_request):
        result = await asyncio.get_event_loop().run_in_executor(None, app_state.save_params)
        if result.get("ok"):
            # 写回后基线已变，界面上的"偏离配置"高亮要跟着刷新
            result["baseline"] = app_state.baseline
        return web.json_response(result)

    @routes.post("/api/set")
    async def api_set(request):
        body = await request.json()
        target = next(
            (t for t in targets if t.snapshot()["target"] == body.get("target")), None
        )
        if target is None:
            return web.json_response({"ok": False, "msg": "未知目标"}, status=400)
        ok, msg = await asyncio.get_event_loop().run_in_executor(
            None, target.set_value, body.get("name"), body.get("value")
        )
        return web.json_response(
            {"ok": ok, "msg": msg, "params": target.snapshot(app_state.baseline)}
        )

    @routes.get("/ws")
    async def websocket(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        app_state.clients.add(ws)
        await ws.send_json(app_state.snapshot())
        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if payload.get("type") == "set_param":
                    target = next(
                        (t for t in targets if t.snapshot()["target"] == payload.get("target")),
                        None,
                    )
                    if target is None:
                        await ws.send_json({"type": "result", "ok": False, "msg": "未知目标"})
                        continue
                    ok, text = await asyncio.get_event_loop().run_in_executor(
                        None, target.set_value, payload.get("name"), payload.get("value")
                    )
                    await ws.send_json(
                        {
                            "type": "result",
                            "ok": ok,
                            "msg": text,
                            "params": target.snapshot(app_state.baseline),
                        }
                    )
        finally:
            app_state.clients.discard(ws)
        return ws

    async def push_loop(_app):
        while True:
            snapshot = app_state.snapshot()
            for ws in list(app_state.clients):
                try:
                    await ws.send_json(snapshot)
                except Exception:
                    app_state.clients.discard(ws)
            await asyncio.sleep(1.0 / app_state.hz if app_state.hz > 0 else 0.5)

    application = web.Application()
    application.add_routes(routes)

    # 注意：on_startup 的回调会在开始监听之前被 await，直接把 push_loop 挂上去会让
    # "永不返回的循环"把启动过程卡住，端口永远不会被监听（表现为页面打不开、
    # 但命令行已经打印了访问地址）。这里只建任务，不 await。
    async def start_push(app):
        app["push_task"] = asyncio.create_task(push_loop(app))

    async def stop_push(app):
        task = app.get("push_task")
        if task is not None:
            task.cancel()

    application.on_startup.append(start_push)
    application.on_cleanup.append(stop_push)
    return application, node


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--target",
        action="append",
        default=None,
        help="node:prefix，可重复。默认 controller_server:FollowPath",
    )
    parser.add_argument(
        "--list",
        action="append",
        default=None,
        help="参数说明文件（paramview/params_list/*.txt 格式），可重复。默认按 profile 推断",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--hz", type=float, default=2.0, help="后台重读频率")
    parser.add_argument(
        "--wait-service", type=float, default=0.0,
        help="启动时每个目标等待服务的秒数（默认 0，交给后台刷新线程发现）",
    )
    parser.add_argument(
        "--baseline-mode", default="auto", choices=[None, "auto", "reality", "simulation"],
        help="参数基线：模式（auto 按运行节点的 use_sim_time 自动识别）",
    )
    parser.add_argument(
        "--baseline-controller", default="auto",
        help="参数基线：控制器 profile（auto 按 FollowPath.plugin 自动识别）",
    )
    parser.add_argument(
        "--baseline-planner", default="auto",
        help="参数基线：规划器 profile（auto 按 planner_server 的 GridBased.plugin 自动识别）",
    )
    parser.add_argument(
        "--no-save", action="store_true",
        help="只读模式：禁止把参数写回配置文件（界面上的保存按钮会失效）",
    )
    args = parser.parse_args()

    if not args.target:
        args.target = ["controller_server:FollowPath"]

    application, node = make_app(args)
    print(f"打开浏览器访问: http://{args.host}:{args.port}", flush=True)
    print("目标: " + ", ".join(args.target), flush=True)
    env = graph_summary(node)
    print(
        f"ROS 环境: ROS_DOMAIN_ID={env['domain_id']} ROS_LOCALHOST_ONLY={env['localhost_only']} "
        f"RMW={env['rmw']}；当前可见节点 {len(env['nodes'])} 个",
        flush=True,
    )
    if "auto" in (args.baseline_mode, args.baseline_controller, args.baseline_planner):
        print(
            f"自动识别: mode={args.baseline_mode}, controller={args.baseline_controller}, "
            f"planner={args.baseline_planner}（节点就绪后自动匹配命名空间与配置基线）",
            flush=True,
        )
    else:
        layers = config_layers(args.baseline_mode, args.baseline_controller, args.baseline_planner)
        if layers and args.no_save:
            print("只读模式：参数不会写回配置文件", flush=True)
        elif layers:
            print(
                "保存写回: " + "、".join(os.path.relpath(path, WS_ROOT) for path in layers),
                flush=True,
            )
    web.run_app(application, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    sys.exit(main())
