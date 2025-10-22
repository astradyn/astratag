#!/bin/bash
# Script to run the quadrilateral detector from the correct directory

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Change to the project root directory
cd "$SCRIPT_DIR"

# Run the detector
./build/quadrilateral_detector "$@"
