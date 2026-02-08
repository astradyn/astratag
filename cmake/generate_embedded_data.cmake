# CMake script to generate embedded data files

# Read dictionary file
file(READ "${DICTIONARY_FILE}" DICTIONARY_CONTENT)
# Read keypoints file  
file(READ "${KEYPOINTS_FILE}" KEYPOINTS_CONTENT)

# Escape special characters for C++ raw string literal
# Raw strings handle most cases, but we need to escape the delimiter
string(REPLACE ")" "\\)" DICTIONARY_CONTENT "${DICTIONARY_CONTENT}")
string(REPLACE ")" "\\)" KEYPOINTS_CONTENT "${KEYPOINTS_CONTENT}")

# Configure the output file
configure_file("${INPUT_FILE}" "${OUTPUT_FILE}" @ONLY)

message(STATUS "Generated embedded data file: ${OUTPUT_FILE}")
message(STATUS "  Dictionary size: ${CMAKE_MATCH_0} bytes")
message(STATUS "  Keypoints size: ${CMAKE_MATCH_1} bytes")