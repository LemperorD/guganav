#!/usr/bin/env python3
"""行为树 Web UI 的服务端：一边 tail 执行记录，一边转发假数据源的参数。

数据来源是决策节点写出的 .btlog（BT.CPP v4 的 FileLogger2 格式）。选它而不是
自己发 ROS 话题，是因为 C++ 侧只需两行就能启用，而解析逻辑已经在
btlog_view.py 里写好了。

参数读写通过 rclpy 直接调假数据源的 get_parameters / set_parameters 服务，
不经过 ros2 CLI——CLI 依赖 node graph 查询，某些环境（含开发沙箱）会报
"Node not found"，而服务发现是好的。

用法：
    scripts/btview/btview_server.py                          # 默认读 /tmp/bt_trace.btlog
    scripts/btview/btview_server.py --log 路径.btlog
    scripts/btview/btview_server.py --source /fake_msg_source --port 8080

浏览器打开 http://localhost:8080。

消息协议：
    服务器 -> 浏览器
        {"type":"tree"}                    树结构，连接时发一次
        {"type":"tick"}                    每完成一次 tick 发一条
        {"type":"params"}                  假数据源参数，定期刷新
        {"type":"result","ok":bool,"msg"}  参数写入的反馈
    浏览器 -> 服务器
        {"type":"set_param","name":...,"value":...}
        {"type":"get_params"}
"""

import argparse
import asyncio
import os
import sys
import threading
import time
from collections import deque

from aiohttp import web

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from btlog_view import build_tree, summarize, tick_marker  # noqa: E402

STATUS_NAMES = {0: "IDLE", 1: "RUNNING", 2: "SUCCESS", 3: "FAILURE"}
MAGIC = b"BTCPP4-FileLogger2"
HEADER_LEN = len(MAGIC) + 1 + 4

# 假数据源的可调参数：名称、类型、说明。与 fake_msg_source_node.cpp 里的声明对应。
SOURCE_PARAMS = [
    ("current_hp", "integer", "当前血量"),
    ("maximum_hp", "integer", "血量上限"),
    ("projectile_allowance_17mm", "integer", "允许发弹量"),
    ("shooter_17mm_1_barrel_heat", "integer", "当前枪管热量"),
    ("shooter_barrel_heat_limit", "integer", "枪管热量上限"),
    ("shooter_barrel_cooling_value", "integer", "枪管冷却/秒"),
    ("remaining_gold_coin", "integer", "剩余金币"),
    ("robot_id", "integer", "机器人 ID"),
    ("robot_level", "integer", "机器人等级"),
    ("publish_rate", "double", "发布频率"),
    ("enemy_count", "integer", "敌人数量（>0 即有敌）"),
    ("vision_rate", "double", "视觉发布频率"),
    ("rfid_center", "bool", "中心增益点 RFID（占点）"),
    ("rfid_base", "bool", "基地增益点 RFID（家）"),
    ("robot_x", "double", "假机器人 x（map 系，移动到哪就看到哪）"),
    ("robot_y", "double", "假机器人 y（map 系）"),
    ("sim_speed", "double", "假机器人速度 m/s（0 = 钉住不动）"),
]


class BtLogTailer:
    """增量读取 .btlog，按 tick 切分。

    文件头（含树 XML）只解析一次，之后只读记录区增量。记录区里不足 9 字节的
    尾巴说明写入还没完成，留到下次再读。
    """

    def __init__(self, path):
        self.path = path
        self.nodes = None
        self.roots = None
        self.rec_offset = None
        self.parsed = 0
        self.pending = []
        self.tick_index = 0
        # 切分 tick 的两个边界节点（根节点与它的第一个孩子）及其上次状态，
        # 规则与 btlog_view.split_ticks 一致：见那里的说明。
        self.entry_uid = None
        self.marker_uid = None
        self.boundary_status = {}

    def _load_header(self, data):
        if data[: len(MAGIC)] != MAGIC:
            raise ValueError(f"不是 FileLogger2 记录：{data[:18]!r}")
        xml_len = int.from_bytes(data[len(MAGIC) + 1 : HEADER_LEN], "little")
        xml_end = HEADER_LEN + xml_len
        if len(data) < xml_end + 8:
            return False
        xml_text = data[HEADER_LEN:xml_end].decode("utf-8", "replace")
        self.nodes, self.roots = build_tree(xml_text)
        self.rec_offset = xml_end + 8
        # tick 边界用入口根节点的第一个孩子，理由见 btlog_view.tick_marker：
        # 根节点跨 tick 停在 RUNNING 时不会有"再次进入 RUNNING"的记录。
        main = self._main_tree()
        if main is not None:
            self.entry_uid = self.roots[main]
            self.marker_uid = tick_marker(self.nodes, self.entry_uid)
            self.boundary_status = {self.entry_uid: 0, self.marker_uid: 0}
        return True

    def _main_tree(self):
        """入口树：根 uid 最小的那棵（uid 按实例化顺序分配，主树排在最前）。"""
        candidates = {k: v for k, v in (self.roots or {}).items() if v is not None}
        return min(candidates, key=lambda tid: candidates[tid]) if candidates else None

    def poll(self):
        """返回自上次调用以来完成的 tick 列表。"""
        try:
            with open(self.path, "rb") as handle:
                data = handle.read()
        except FileNotFoundError:
            return []

        if self.nodes is None:
            if not self._load_header(data):
                return []
        if len(data) <= self.rec_offset:
            return []

        body = data[self.rec_offset :]
        total = len(body) // 9
        if total == self.parsed:
            return []

        finished = []
        for i in range(self.parsed, total):
            chunk = body[i * 9 : (i + 1) * 9]
            ts = int.from_bytes(chunk[0:6], "little")
            uid = int.from_bytes(chunk[6:8], "little")
            status = chunk[8]
            # 边界规则与 btlog_view.split_ticks 保持一致：两个边界节点里有任意
            # 一个"离开 IDLE"就是新的一轮；若当前攒下的还只有根节点自己的事件，
            # 说明这一轮才刚开始，不算独立的一次 tick。
            if uid in self.boundary_status:
                started = self.boundary_status[uid] == 0 and status != 0
                self.boundary_status[uid] = status
                only_root = all(u == self.entry_uid for _t, u, _s in self.pending)
                if started and self.pending and not only_root:
                    finished.append(self._finish(self.pending))
                    self.pending = []
            self.pending.append((ts, uid, status))
        self.parsed = total
        return finished

    def _finish(self, tick):
        self.tick_index += 1
        leaves, subtrees = summarize(tick, self.nodes)
        first = tick[0][0]
        return {
            "type": "tick",
            "index": self.tick_index,
            "t_start_ms": first / 1000.0,
            "span_us": tick[-1][0] - first,
            "path": subtrees,
            "leaves": [
                {"uid": uid, "label": label, "status": STATUS_NAMES.get(status, str(status))}
                for uid, label, status in leaves
            ],
            "events": [
                {"uid": uid, "status": STATUS_NAMES.get(status, str(status)), "dt_us": ts - first}
                for ts, uid, status in tick
            ],
        }

    def tree_message(self):
        if self.nodes is None:
            return None
        # 界面上必须从入口根往下画：日志里所有节点是平铺的，只有连通的那部分
        # 属于展开后的树。
        return {
            "type": "tree",
            "main": self._main_tree(),
            "roots": self.roots,
            "nodes": [
                {
                    "uid": info["uid"],
                    "label": info["label"],
                    "tag": info["tag"],
                    "subtree": info["subtree"],
                    "path": info["path"],
                    "children": info["children"],
                    "leaf": info["leaf"],
                }
                for info in self.nodes.values()
            ],
        }


def _parse_bool(text):
    """把界面上输入的文本转成布尔量，认不出来就抛 ValueError。"""
    shown = text.strip().lower()
    if shown in ("1", "true", "on", "yes"):
        return True
    if shown in ("0", "false", "off", "no"):
        return False
    raise ValueError(f"值不是布尔量（用 true/false 或 1/0）: {text}")


class SourceBridge:
    """读写假数据源参数的 ROS 2 侧封装。

    rclpy 在专用线程上 spin，主线程只投递请求再轮询 future——这样 asyncio 的
    事件循环不会被阻塞，也不需要在两处各自 spin。
    """

    def __init__(self, target_node):
        import rclpy
        from rclpy.node import Node
        from rcl_interfaces.srv import GetParameters, SetParameters

        self._rclpy = rclpy
        self._target = target_node
        self._get_srv = GetParameters
        self._set_srv = SetParameters

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node("btview_server")
        self.get_cli = self.node.create_client(GetParameters, f"{target_node}/get_parameters")
        self.set_cli = self.node.create_client(SetParameters, f"{target_node}/set_parameters")

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while not self._stop.is_set():
            try:
                self._rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception:
                break

    def _await(self, future, timeout=3.0):
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.01)
        return future.result()

    def read_all(self):
        """返回 [(name, kind, desc, value)]，服务不可用时返回 None。"""
        if not self.get_cli.wait_for_service(timeout_sec=1.0):
            return None
        from rcl_interfaces.msg import ParameterType

        req = self._get_srv.Request()
        req.names = [name for name, _kind, _desc in SOURCE_PARAMS]
        res = self._await(self.get_cli.call_async(req))
        if res is None:
            return None

        items = []
        for (name, kind, desc), value in zip(SOURCE_PARAMS, res.values):
            if value.type == ParameterType.PARAMETER_DOUBLE:
                shown = value.double_value
            elif value.type == ParameterType.PARAMETER_BOOL:
                shown = value.bool_value
            else:
                shown = value.integer_value
            items.append({"name": name, "kind": kind, "desc": desc, "value": shown})
        return items

    def write(self, name, text):
        """把界面上输入的文本按参数类型写回。返回 (ok, message)。"""
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        kind = dict((n, k) for n, k, _d in SOURCE_PARAMS).get(name, "integer")
        if not self.set_cli.wait_for_service(timeout_sec=1.0):
            return False, f"{self._target} 的 set_parameters 服务不可用"

        # 按声明类型构造 ParameterValue：类型不匹配会直接被 set_parameters 拒绝，
        # 所以不能用"整型也塞给布尔参数"这种偷懒写法。
        try:
            if kind == "double":
                value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                       double_value=float(text))
            elif kind == "bool":
                value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                       bool_value=_parse_bool(text))
            else:
                value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                       integer_value=int(float(text)))
        except ValueError as exc:
            return False, f"{name}: {exc}"

        req = self._set_srv.Request()
        req.parameters = [Parameter(name=name, value=value)]
        res = self._await(self.set_cli.call_async(req))
        if res is None:
            return False, "写入超时"
        ok = all(r.successful for r in res.results)
        reason = "; ".join(r.reason for r in res.results if r.reason)
        return ok, reason or ("已写入" if ok else "写入被拒绝")

    def shutdown(self):
        self._stop.set()
        # spin 线程要先收回来再拆节点：先关 rclpy 的话，线程可能还停在
        # spin_once 里，进程退出时会直接 abort 并打印 "terminate called"。
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass


class BtViewServer:
    def __init__(self, log_path, html_path, source_node, poll_interval=0.1,
                 history=10):
        self.tailer = BtLogTailer(log_path)
        self.html_path = html_path
        self.poll_interval = poll_interval
        self.clients = set()
        self.last_params = None
        # 最近几次 tick 留一份：轮询是增量的，页面晚连上来就再也收不到已经解析过
        # 的 tick，界面上会一直空着，直到下一个 tick 边界。留一份补发给新连接。
        self.recent = deque(maxlen=history)
        try:
            self.source = SourceBridge(source_node)
        except Exception as exc:
            print(f"参数桥接未启用: {exc}", file=sys.stderr)
            self.source = None

    # ---- HTTP ----
    async def index(self, request):
        with open(self.html_path, "r", encoding="utf-8") as handle:
            return web.Response(text=handle.read(), content_type="text/html")

    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        self.clients.add(ws)
        try:
            tree = self.tailer.tree_message()
            if tree:
                await ws.send_json(tree)
            # 按时间顺序补发最近的 tick，前端把它当实时消息处理即可。
            for tick in self.recent:
                await ws.send_json(tick)
            if self.last_params:
                await ws.send_json(self.last_params)
            async for msg in ws:
                if msg.type == web.WSMsgType.ERROR:
                    break
                if msg.type == web.WSMsgType.TEXT:
                    await self.handle_client(ws, msg.json())
        finally:
            self.clients.discard(ws)
        return ws

    async def handle_client(self, ws, msg):
        kind = msg.get("type")
        if kind == "get_params":
            await self.push_params()
        elif kind == "set_param":
            if not self.source:
                await ws.send_json({"type": "result", "ok": False,
                                    "msg": "参数桥接未启用"})
                return
            loop = asyncio.get_running_loop()
            ok, text = await loop.run_in_executor(
                None, self.source.write, msg["name"], str(msg["value"])
            )
            await ws.send_json({"type": "result", "ok": ok,
                                "msg": f"{msg['name']}: {text}"})
            await self.push_params()

    async def push_params(self):
        if not self.source:
            return
        loop = asyncio.get_running_loop()
        items = await loop.run_in_executor(None, self.source.read_all)
        if items is None:
            return
        self.last_params = {"type": "params", "items": items}
        await self.broadcast(self.last_params)

    async def broadcast(self, payload):
        dead = []
        for ws in self.clients:
            try:
                await ws.send_json(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def poll_loop(self):
        sent_tree = self.tailer.nodes is not None
        last_params_at = 0.0
        while True:
            await asyncio.sleep(self.poll_interval)
            try:
                ticks = self.tailer.poll()
            except Exception as exc:
                print(f"读取记录失败: {exc}", file=sys.stderr)
                ticks = []

            if not sent_tree and self.tailer.nodes is not None:
                tree = self.tailer.tree_message()
                if tree:
                    await self.broadcast(tree)
                    sent_tree = True

            for tick in ticks:
                self.recent.append(tick)
                await self.broadcast(tick)

            # 参数每秒刷新一次，够用又不会刷屏
            now = time.time()
            if self.clients and now - last_params_at > 1.0:
                last_params_at = now
                await self.push_params()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description="行为树 Web UI 服务端")
    parser.add_argument("--log", default="/tmp/bt_trace.btlog", help=".btlog 文件路径")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址")
    parser.add_argument("--port", type=int, default=8080, help="监听端口")
    parser.add_argument("--source", default="/fake_msg_source", help="假数据源节点名")
    parser.add_argument("--html", default=os.path.join(here, "btview.html"))
    parser.add_argument("--interval", type=float, default=0.1, help="轮询间隔（秒）")
    args = parser.parse_args()

    if not os.path.exists(args.html):
        raise SystemExit(f"前端页面不存在：{args.html}")

    server = BtViewServer(args.log, args.html, args.source, args.interval)

    app = web.Application()
    app.router.add_get("/", server.index)
    app.router.add_get("/ws", server.websocket)

    async def start_background(app):
        app["poller"] = asyncio.create_task(server.poll_loop())

    async def stop_background(app):
        app["poller"].cancel()
        if server.source:
            server.source.shutdown()

    app.on_startup.append(start_background)
    app.on_cleanup.append(stop_background)

    print(f"记录文件: {args.log}")
    print(f"假数据源节点: {args.source}")
    print(f"打开浏览器访问: http://localhost:{args.port}")
    web.run_app(app, host=args.host, port=args.port, print=None)


if __name__ == "__main__":
    main()
