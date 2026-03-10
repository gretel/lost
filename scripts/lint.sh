#!/usr/bin/env bash
# lint.sh — run clang-tidy and/or cppcheck on changed or specified C++ files
#
# Usage:
#   scripts/lint.sh                    # lint C++ files changed vs HEAD
#   scripts/lint.sh --staged           # lint staged files only (pre-commit)
#   scripts/lint.sh --diff             # lint only changed lines (fastest)
#   scripts/lint.sh file1.hpp file2.cpp # lint specific files
#   scripts/lint.sh --all              # lint all project C++ files
#
# Requires: build/compile_commands.json (cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
set -euo pipefail

LLVM_PREFIX="${LLVM_PREFIX:-$(brew --prefix llvm 2>/dev/null || echo /opt/homebrew/opt/llvm)}"
CLANG_TIDY="${LLVM_PREFIX}/bin/clang-tidy"
CLANG_TIDY_DIFF="${LLVM_PREFIX}/share/clang/clang-tidy-diff.py"
BUILD_DIR="${BUILD_DIR:-build}"
PROJECT_DIRS="blocks/include apps"

die() { echo "error: $*" >&2; exit 1; }
info() { echo "=== $* ==="; }

[[ -x "$CLANG_TIDY" ]] || die "clang-tidy not found at $CLANG_TIDY"
[[ -f "$BUILD_DIR/compile_commands.json" ]] || die \
    "compile_commands.json not found. Reconfigure with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"

collect_changed_files() {
    git diff $1 --name-only --diff-filter=d -- '*.cpp' '*.hpp' '*.h' 2>/dev/null || true
}

collect_all_files() {
    find $PROJECT_DIRS -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) 2>/dev/null || true
}

run_clang_tidy() {
    local files="$1"
    [[ -z "$files" ]] && { echo "no files to check"; return 0; }
    local count
    count=$(echo "$files" | wc -l | tr -d ' ')
    info "clang-tidy ($count file(s))"
    echo "$files" | xargs -P4 "$CLANG_TIDY" -p "$BUILD_DIR" --quiet 2>&1 || true
}

run_clang_tidy_diff() {
    info "clang-tidy (changed lines only)"
    git diff -U0 HEAD | python3 "$CLANG_TIDY_DIFF" \
        -p1 -path "$BUILD_DIR" -clang-tidy-binary "$CLANG_TIDY" -j4 2>&1
}

run_cppcheck() {
    local files="$1"
    [[ -z "$files" ]] && return 0
    command -v cppcheck &>/dev/null || return 0
    info "cppcheck"
    echo "$files" | xargs cppcheck --std=c++23 \
        --enable=warning,performance,portability \
        --suppress=missingIncludeSystem --suppress=unmatchedSuppression \
        --quiet 2>&1
}

mode="${1:-changed}"
shift 2>/dev/null || true

case "$mode" in
    --staged)  files=$(collect_changed_files "--cached") ;;
    --diff)    run_clang_tidy_diff; exit ;;
    --all)     files=$(collect_all_files) ;;
    changed)   files=$(collect_changed_files "HEAD") ;;
    *)         files="$mode"; for f in "$@"; do files="$files"$'\n'"$f"; done ;;
esac

run_clang_tidy "$files"
run_cppcheck "$files"
