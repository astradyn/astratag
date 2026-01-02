#!/bin/bash
# Run static analysis on AstraTag

set -e

# Check if clang-tidy is installed
command -v clang-tidy >/dev/null 2>&1 || { echo "clang-tidy not found. Install with: sudo apt install clang-tidy"; exit 1; }

chmod +x clang-tidy.sh

echo "Running clang-tidy..."
./clang-tidy.sh

echo ""
echo "Analysis complete."
