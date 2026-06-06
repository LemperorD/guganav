#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
cd "$ROOT"

push_after_commit=false

usage() {
  cat <<'EOF'
Usage:
  scripts/gitPush.sh [--push]

Create a commit with the project message format. With --push, push the current
branch after a successful commit.
EOF
}

for arg in "$@"; do
  case "$arg" in
    -p|--push)
      push_after_commit=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_git_repo() {
  git rev-parse --show-toplevel >/dev/null
}

has_staged_changes() {
  ! git diff --cached --quiet
}

has_unstaged_changes() {
  ! git diff --quiet || [ -n "$(git ls-files --others --exclude-standard)" ]
}

prompt_required() {
  local label=$1
  local value
  while true; do
    read -r -p "$label" value
    if [ -n "$value" ]; then
      printf '%s' "$value"
      return
    fi
    echo "不能为空。" >&2
  done
}

edit_field() {
  local name=$1
  local current=$2
  local value

  read -r -p "$name [$current]: " value
  if [ -n "$value" ]; then
    printf '%s' "$value"
  else
    printf '%s' "$current"
  fi
}

print_commit_message() {
  echo
  echo "commit message:"
  echo "$title"
  echo
  echo "范围：$scope"
  echo
  echo "行为：$behavior"
  echo
  echo "验证：$validation"
  echo
}

edit_commit_message() {
  local choice

  while true; do
    echo "修改哪一项？"
    echo "1) 标题"
    echo "2) 范围"
    echo "3) 行为"
    echo "4) 验证"
    echo "5) 返回预览"
    read -r -p "请选择 [1-5]: " choice

    case "$choice" in
      1)
        title=$(edit_field "标题" "$title")
        ;;
      2)
        scope=$(edit_field "范围" "$scope")
        ;;
      3)
        behavior=$(edit_field "行为" "$behavior")
        ;;
      4)
        validation=$(edit_field "验证" "$validation")
        ;;
      5)
        return
        ;;
      *)
        echo "请输入 1-5。"
        ;;
    esac
  done
}

confirm_commit_message() {
  local confirm

  while true; do
    print_commit_message
    read -r -p "确认提交？[y 提交 / e 修改 / n 取消] " confirm
    case "$confirm" in
      y|Y)
        return 0
        ;;
      e|E)
        edit_commit_message
        ;;
      n|N|"")
        echo "已取消。"
        return 1
        ;;
      *)
        echo "请输入 y、e 或 n。"
        ;;
    esac
  done
}

ensure_staged_changes() {
  if has_staged_changes; then
    return
  fi

  if ! has_unstaged_changes; then
    echo "没有可提交的改动。"
    exit 1
  fi

  git status --short
  read -r -p "当前没有暂存改动，是否 git add -A 暂存全部改动？[y/N] " answer
  case "$answer" in
    y|Y)
      git add -A
      ;;
    *)
      echo "请先 git add 需要提交的文件。"
      exit 1
      ;;
  esac

  if ! has_staged_changes; then
    echo "暂存区仍然为空。"
    exit 1
  fi
}

push_current_branch() {
  local branch
  branch=$(git symbolic-ref --short HEAD)

  if git rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
    git push
  else
    git push -u origin "$branch"
  fi
}

require_git_repo
ensure_staged_changes

echo "即将提交以下暂存改动："
git diff --cached --name-status
echo

title=$(prompt_required "标题（例：测试：整理 PID 控制器单元测试）: ")
scope=$(prompt_required "范围：")
behavior=$(prompt_required "行为：")
validation=$(prompt_required "验证：")

if ! confirm_commit_message; then
  exit 1
fi

git commit \
  -m "$title" \
  -m "范围：$scope" \
  -m "行为：$behavior" \
  -m "验证：$validation"

if [ "$push_after_commit" = true ]; then
  push_current_branch
fi
