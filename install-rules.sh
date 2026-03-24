#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET_DIR="${HOME}/.claude/rules"

usage() {
  echo "Usage: $0 [general|edge-robotics|all] [--dry-run]"
  echo ""
  echo "Install rules to ~/.claude/rules/ with namespace prefix."
  echo "Files are named {plugin}--{rule}.md to avoid conflicts."
  exit 1
}

PLUGIN="${1:-all}"
DRY_RUN=false
for arg in "$@"; do
  if [ "$arg" = "--dry-run" ]; then
    DRY_RUN=true
  fi
  if [ "$arg" = "--help" ] || [ "$arg" = "-h" ]; then
    usage
  fi
done

install_rules() {
  local plugin_name="$1"
  local src_dir="${SCRIPT_DIR}/${plugin_name}/rules"

  if [ ! -d "$src_dir" ]; then
    echo "  no rules directory for ${plugin_name}, skipping"
    return
  fi

  mkdir -p "$TARGET_DIR"

  local count=0
  for rule in "$src_dir"/*.md; do
    [ -f "$rule" ] || continue
    local basename
    basename=$(basename "$rule")
    local target="${TARGET_DIR}/${plugin_name}--${basename}"

    if [ -f "$target" ] && diff -q "$rule" "$target" > /dev/null 2>&1; then
      echo "  skip (unchanged): ${plugin_name}--${basename}"
    else
      if [ "$DRY_RUN" = true ]; then
        echo "  would install: ${plugin_name}--${basename}"
      else
        cp "$rule" "$target"
        echo "  installed: ${plugin_name}--${basename}"
      fi
      count=$((count + 1))
    fi
  done

  if [ "$count" -eq 0 ]; then
    echo "  ${plugin_name}: all rules up to date"
  fi
}

echo "Installing rules to ${TARGET_DIR}/"
if [ "$DRY_RUN" = true ]; then
  echo "(dry run - no files will be modified)"
fi
echo ""

case "$PLUGIN" in
  general|edge-robotics)
    install_rules "$PLUGIN"
    ;;
  all)
    install_rules "general"
    install_rules "edge-robotics"
    ;;
  --dry-run)
    install_rules "general"
    install_rules "edge-robotics"
    ;;
  *)
    usage
    ;;
esac

echo ""
echo "Done."
