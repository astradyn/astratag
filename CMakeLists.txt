cmake_minimum_required(VERSION 3.16)
project(AstraTagDetector)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(ASTRATAG_EMBED_DATA "Embed dictionary and keypoints into library" OFF)
option(ASTRATAG_ENABLE_CLANG_TIDY "Enable clang-tidy during build" OFF)
option(ASTRATAG_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# Method 1: Set OpenCV_DIR to your OpenCV root folder
set(OpenCV_DIR "path/to/your/opencv/build")  # <-- Change this path accordingly
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

# Source files
set(SOURCES
    src/quadrilateral.cpp
    src/signature.cpp
    src/detector.cpp
    src/visualization.cpp
)

# Generate embedded data if requested
if(ASTRATAG_EMBED_DATA)
    message(STATUS "Embedding data files into library")
    
    set(EMBEDDED_DATA_FILE "${CMAKE_BINARY_DIR}/embedded_data.cpp")
    
    add_custom_command(
        OUTPUT "${EMBEDDED_DATA_FILE}"
        COMMAND ${CMAKE_COMMAND}
            -DINPUT_FILE="${CMAKE_SOURCE_DIR}/src/embedded_data.cpp.in"
            -DOUTPUT_FILE="${EMBEDDED_DATA_FILE}"
            -DDICTIONARY_FILE="${CMAKE_SOURCE_DIR}/data/new_dictionary.json"
            -DKEYPOINTS_FILE="${CMAKE_SOURCE_DIR}/data/keypoints.txt"
            -P "${CMAKE_SOURCE_DIR}/cmake/generate_embedded_data.cmake"
        DEPENDS 
            "${CMAKE_SOURCE_DIR}/src/embedded_data.cpp.in"
            "${CMAKE_SOURCE_DIR}/data/new_dictionary.json"
            "${CMAKE_SOURCE_DIR}/data/keypoints.txt"
        COMMENT "Generating embedded data file"
    )
    
    list(APPEND SOURCES "${EMBEDDED_DATA_FILE}")
    add_compile_definitions(ASTRATAG_USE_EMBEDDED_DATA)
endif()

# Create library
add_library(astratag_lib ${SOURCES})
target_include_directories(astratag_lib PUBLIC 
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(astratag_lib PUBLIC 
    ${OpenCV_LIBS}
)

# Examples
add_executable(astratag_detect
    examples/detect_tags.cpp
)
target_link_libraries(astratag_detect PRIVATE astratag_lib)

# Output in build directory
set_target_properties(astratag_detect PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Enhanced compiler warnings
set(ASTRATAG_WARNING_FLAGS
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wconversion
    -Wsign-conversion
    -Wformat=2
)

if(ASTRATAG_WARNINGS_AS_ERRORS)
    list(APPEND ASTRATAG_WARNING_FLAGS -Werror)
endif()

target_compile_options(astratag_lib PRIVATE ${ASTRATAG_WARNING_FLAGS})
target_compile_options(astratag_detect PRIVATE ${ASTRATAG_WARNING_FLAGS})

# Static analysis integration
if(ASTRATAG_ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES clang-tidy)
    if(CLANG_TIDY_EXE)
        message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE};--config-file=${CMAKE_SOURCE_DIR}/.clang-tidy")
    else()
        message(WARNING "clang-tidy not found!")
    endif()
endif()

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "Building AstraTag Detection System")
message(STATUS "  Main executable: astratag_detect")
message(STATUS "  Library: astratag_lib")