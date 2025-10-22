cmake_minimum_required(VERSION 3.16)
project(QuadrilateralDetector)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Method 1: Set OpenCV_DIR to your OpenCV root folder
set(OpenCV_DIR "path/to/opencv")
# Examples:
# Windows: "C:/opencv"
# Linux: "/home/username/opencv"
# Mac: "/usr/local/opt/opencv"

find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Create executable
add_executable(quadrilateral_detector
    src/main.cpp
    src/quadrilateral.cpp
)

# Link OpenCV libraries
target_link_libraries(quadrilateral_detector ${OpenCV_LIBS})

# Output in build directory
set_target_properties(quadrilateral_detector PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Set working directory to project root for debugging/running
set_target_properties(quadrilateral_detector PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
