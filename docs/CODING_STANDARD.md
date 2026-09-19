# 编码规范


## 通用规范

以下不区分模式，全项目统一。

### 命名

| 元素               | 约定                                                        |
| ------------------ | ----------------------------------------------------------- |
| class/struct       | PascalCase                                                  |
| 函数/方法          | camelCase                                                   |
| compile-time 常量  | UPPER_CASE `static constexpr int TERRAIN_VOXEL_WIDTH = 21;` |
| 变量（局部/参数）  | snake_case，全拼不简写                                      |
| 私有成员（模式 B） | 尾随 `_`：`config_`, `node_`                                |
| enum class 值      | UPPER_CASE：`UNINITIALIZED`, `RECORDING`                    |
| bool               | `is_`/`has_`/`should_`/`can_` 前缀                          |

### 类型

| 场景           | 使用类型                                      |
| -------------- | --------------------------------------------- |
| 坐标/距离/阈值 | `double`（不用 `float`）                      |
| PCL 点坐标     | `float`（PCL 库要求）                         |
| 数组索引       | `size_t`，非负                                |
| 网格坐标       | `int`，可为负                                 |
| 枚举底层类型   | `uint8_t`（值域小）                           |
| 时间相关       | raw nanoseconds: `int64_t`，seconds: `double` |

### 代码组织

| 规则               | 说明                                                              |
| ------------------ | ----------------------------------------------------------------- |
| `#pragma once`     | 所有头文件                                                        |
| 范围for            | 遍历点云、vector 优先用 `for (const auto& point : cloud->points)` |
| 局部变量提取       | 避免 `state.lidar_x` 在循环内重复读取，提到循环外               |
| `static constexpr` | 编译期可确定的常量放在 struct 或函数内                            |
| 值初始化           | 用 `{}` 统一初始化：`int x{};` / `double t{};` / `bool flag{};`。禁止 `= 0` / `= 0.0` / `= false` |
| 结构体默认值       | 字段默认值用 `{}`（值初始化）或 `{value}`（带初值）              |
| `auto`             | 类型从上下文明显可推断时用，否则显式写类型                        |
| 显式 cast          | 符号/窄化转换显式 `static_cast<T>()`，不隐式                      |
| `[[nodiscard]]`    | 返回值的查询方法标记                                              |

### 禁止

- 全局变量
- 逗号连续声明 `int x, y, z;` — 每个变量独占一行
- 魔法数字 — `0`/`1`/`-1` 以外的字面量提为命名常量 `constexpr` / `static constexpr`

### 注释

- 默认不写注释，只在原因非明显时添加
- 不使用 Doxygen `///`，不使用 `// ...` 分区注释段

## 保存时自动格式化（clangd）

格式只由仓库根的 `.clang-format` 决定，pre-commit 钩子与编辑器用的是同一份，因此保存过的
文件在提交时不会再被钩子改写。clangd 另外读 `.clangd`（编译参数与 clang-tidy 检查项），
编译数据库是仓库根的 `compile_commands.json`（软链到 `build/compile_commands.json`）。

- **VS Code**：装 clangd 扩展，并在 `.vscode/settings.json` 里给 C/C++ 指定它：

  ```json
  "[cpp] { "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd",
           "editor.formatOnSave": true },
  "[c]":  { "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd",
           "editor.formatOnSave": true }
  ```

  注意 `.vscode/` 被本地 exclude（`.git/info/exclude`）排除、不入库，所以这台机器上配好
  之后，换机器要再配一次。若同时装了 `ms-vscode.cpptools`，把它的 IntelliSense 关掉
  （`"C_Cpp.intelliSenseEngine": "disabled"`），否则两套诊断会重复。
- **CLion**：Settings → Tools → Actions on Save 勾选 “Reformat code”，并在
  Settings → Editor → Code Style → C/C++ 里启用 clang-format（读同一份 `.clang-format`）。
- **Neovim**：在写入前调用 LSP 的格式化：

  ```lua
  vim.api.nvim_create_autocmd("BufWritePre", {
    pattern = { "*.cpp", "*.hpp", "*.h" },
    callback = function() vim.lsp.buf.format({ async = false }) end,
  })
  ```

`.clangd` 的 `CompileFlags.CompilationDatabase: build` 指向 colcon 的构建目录；某个包的
编译数据库没生成时，clangd 会因为拿不到编译参数而解析错乱，重新构建该包即可。
