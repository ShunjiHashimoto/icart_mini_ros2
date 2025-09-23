#!/usr/bin/env bash
set -euo pipefail

# Generate a single consolidated Markdown from wiki pages.
# Output: All-in-One.md (repo root)

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
OUT_FILE="$ROOT_DIR/Wiki.md"

declare -a ORDER=(
  "Year-In-Review.md:Year-In-Review"
  "Roadmap.md:Roadmap"
  "Changelog.md:Changelog"
)

WIKI_DIR="$ROOT_DIR/.docs/wiki"

{
  echo "# icart_mini_ros2 — All-in-One Wiki"
  echo
  echo "## Table of Contents"
  for item in "${ORDER[@]}"; do
    file="${item%%:*}"; anchor="${item##*:}";
    echo "- [${anchor}](#${anchor,,})"
  done
  echo

  for item in "${ORDER[@]}"; do
    file="${item%%:*}"; anchor="${item##*:}";
    path="$WIKI_DIR/$file"
    if [[ ! -f "$path" ]]; then
      echo "<!-- Skipped missing: $file -->"
      continue
    fi
    echo
    echo "---"
    echo
    echo "## ${anchor}"
    echo
    sed -e 's#\.\./imgs/#.docs/imgs/#g' \
        -e 's#\.\./videos/#.docs/videos/#g' "$path"
    echo
  done
} > "$OUT_FILE"

echo "Generated: $OUT_FILE"
