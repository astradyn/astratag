# AstraTag Examples

This directory contains example applications demonstrating the AstraTag detection library.

## Examples

### detect_tags.cpp

Batch processing example that detects AstraTag markers in a directory of images.

**Usage:**
```bash
cd build
./astratag_detect
```

**Features:**
- Processes all images in `astratag_data/` directory
- Draws pose visualization with 3D cube overlay
- Saves results to `results/` directory
- Reports detection statistics and timing

**Input Requirements:**
- Images in `astratag_data/` folder
- Marker dictionary: `new_dictionary.json`
- Camera intrinsics: `intrinsic_astratag.json`

**Output:**
- Annotated images in `results/` folder
- Console statistics: detection rate, FPS, timing per image

## Building Additional Examples

To add your own example:

1. Create a new `.cpp` file in this directory
2. Add to CMakeLists.txt:
```cmake
add_executable(my_example examples/my_example.cpp)
target_link_libraries(my_example PRIVATE astratag_lib)
```
3. Rebuild with `make`

## Example Template

```cpp
#include "detector.hpp"
#include "visualization.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace astratag;

int main() {
    // Load dictionary and camera intrinsics
    auto dictionary = load_tag_dictionary("new_dictionary.json");
    auto intrinsics = load_camera_intrinsics("intrinsic_astratag.json");
    
    // Load image
    cv::Mat image = cv::imread("test.jpg");
    if (image.empty()) {
        std::cerr << "Failed to load image\n";
        return 1;
    }
    
    // Detect tags
    auto result = detect_tag(image, dictionary);
    
    std::cout << "Detected " << result.count << " markers\n";
    
    // Draw results
    for (size_t i = 0; i < result.count; ++i) {
        draw_tag(image, result.corners[i], result.indices[i]);
        auto pose = estimate_pose(result.corners[i], 
                                  result.world_locs[i], 
                                  intrinsics);
        if (pose.success) {
            draw_pose_cube(image, pose, result.corners[i], intrinsics);
        }
    }
    
    // Display result
    cv::imshow("Detection", image);
    cv::waitKey(0);
    
    return 0;
}
```
