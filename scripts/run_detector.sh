#!/bin/bash
# Script to run the quadrilateral detector from the correct directory

# Get the directory where this script is located (scripts folder)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Go up one level to the project root directory
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Run the detector
./build/quadrilateral_detector "$@"
