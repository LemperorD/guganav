# 语义地图 TODO

只有一条隧道。yaml 的读取与校验放在 launch 里，节点只接收已经校验过的参数，
对外提供两个查询。

## 1. 文件

- [x] `include/guga_decision_common/semantic_map.hpp`：结构体、几何声明、类声明
- [x] `src/semantic_map_node.cpp`：节点函数（构造、`publishMarkers`），编进库
- [x] `src/semantic_map.cpp`：成员函数（`near`、`next_morph_trigger`）与几何
      实现，编进库
- [x] `src/semantic_map_server.cpp`：只有 `main`，编成可执行文件
- [x] `launch/semantic_map.launch.py`：读 yaml、校验、起节点
- [x] `src/guga_bringup/map/simulation/<world>_tunnel.yaml`

## 2. launch 负责的事

- [x] 读 `tunnel_yaml` 参数指定的文件，路径缺省按 `world` 拼，与栅格地图同前缀
      （[simulation_launch.py:74](../../guga_bringup/launch/simulation_launch.py#L74)）。
- [x] `load_tunnel(path)`：`yaml.safe_load` → 校验 → 返回参数字典。
  - `p0`、`p1` 必须是两个数
  - `half_width` 必须为正
  - `p0` 与 `p1` 不能重合
  - `approach_margin` 不能为负（缺省 0）
- [x] 校验失败 `raise RuntimeError`，launch 直接失败，不放半套参数进导航。
- [x] 通过 `parameters=[...]` 把四个值传给节点，节点不再读文件。

## 3. 节点负责的事

- [x] 构造：`declare_parameter` 读 `p0`、`p1`、`half_width`、`approach_margin`。
- [x] 只挡“点数不对”（`p0`/`p1` 不是两个数时 `valid_` 为 false），其余交给 launch。
- [x] `near(x, y, radius) -> vector<const SemanticRegion*>`
  - 区域距离 = 到中线距离减半宽（最小 0），小于等于半径即命中。
- [x] `next_morph_trigger(path, lookahead) -> optional<Trigger>`
  - 沿路径每 5 厘米采样，返回首次进入隧道的点与弧长；已在隧道内返回 0。
- [x] `publishMarkers()`：发黄色 `LINE_STRIP`，`transient_local`。

## 4. CMake 与依赖

- [x] `add_executable(semantic_map_server src/semantic_map_server.cpp)`
- [x] `install(TARGETS semantic_map_server RUNTIME DESTINATION bin)`
- [x] `install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)`
- [x] 去掉 `yaml-cpp`：节点不再读 yaml，依赖只剩 rclcpp 与 visualization_msgs。

## 5. 标注与验收

- [ ] RViz 用 Publish Point 点隧道两端，确认 Fixed Frame 是 `map`，填进 yaml。
- [ ] `ros2 launch guga_decision_common semantic_map.launch.py` 能起节点，RViz 看到线段。
- [ ] 故意写坏 yaml（缺 `p1`、半宽为 0、两点重合），launch 三种都报错退出。
- [ ] 单测覆盖 `next_morph_trigger`：路径未命中、命中、已在隧道内三种情况。
