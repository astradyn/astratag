#!/bin/bash
# Run clang-tidy static analysis on AstraTag

# Ensure build directory exists with compile_commands.json
if [ ! -f build/compile_commands.json ]; then
    echo "Error: compile_commands.json not found!"
    echo "Please run: cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -B build"
    exit 1
fi

# Find all source files
SOURCE_FILES=$(find src include examples -name "*.cpp" -o -name "*.hpp" | grep -v "nlohmann")

# Run clang-tidy
echo "$SOURCE_FILES" | xargs -I {} clang-tidy \
    -p build \
    --config-file=.clang-tidy \
    {}
