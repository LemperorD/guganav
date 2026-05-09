#!/bin/bash

# ===============================
# 配置工作空间根目录和代理
# ===============================
WORKSPACE_DIR=~/guganav
HTTPS_PROXY=http://127.0.0.1:7897
HTTP_PROXY=http://127.0.0.1:7897
ALL_PROXY=socks5://127.0.0.1:7897

cd "$WORKSPACE_DIR" || { echo "无法进入工作目录 $WORKSPACE_DIR"; exit 1; }

# ===============================
# 扫描 Git 仓库（主仓库 + 子模块）
# ===============================
echo "正在扫描 Git 仓库..."
repos=()

# 包括主仓库
if [ -d ".git" ]; then
    repos+=(".")
fi

# 扫描子模块
while IFS= read -r line; do
    path=$(echo "$line" | awk '{print $2}')
    if [ -d "$path/.git" ]; then
        repos+=("$path")
    fi
done < <(git submodule status)

# 显示仓库表格
echo "找到以下 Git 仓库："
for i in "${!repos[@]}"; do
    repo_name=$(basename "${repos[$i]}")
    echo "$i) $repo_name -> ${repos[$i]}"
done

# ===============================
# 用户选择仓库
# ===============================
read -rp "请输入要 push 的仓库序号（多个用空格分隔）： " -a choices

# ===============================
# 对选择的仓库执行 push
# ===============================
for index in "${choices[@]}"; do
    repo="${repos[$index]}"
    repo_name=$(basename "$repo")
    echo "-------------------------------"
    echo "正在处理仓库: $repo_name (路径: $repo)"
    cd "$WORKSPACE_DIR/$repo" || continue

    # 设置代理
    export https_proxy=$HTTPS_PROXY
    export http_proxy=$HTTP_PROXY
    export all_proxy=$ALL_PROXY

    # 获取当前分支
    current_branch=$(git symbolic-ref --short HEAD)
    echo "当前分支: $current_branch"

    # 检查是否有未提交的修改
    if git diff --quiet && git diff --cached --quiet; then
        echo "仓库 $repo_name 没有未提交的修改，执行 push..."
    else
        echo "仓库 $repo_name 有未提交的修改，请先在 VSCode 完成 commit!"
        continue
    fi

    # push 到远程同名分支
    git push origin "$current_branch"
done

echo "所有选择的仓库已 push 完成!"