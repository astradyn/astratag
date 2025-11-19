# Building and Using AstraTag

Quick start guide for building and integrating the AstraTag detection library.

## Quick Build

```bash
git clone <repository-url>
cd astratag
mkdir build && cd build
cmake ..
make -j4
```

This produces:
- `libastratag_lib.a` - Static library (2.4 MB)
- `astratag_detect` - Example application (782 KB)

## Running the Example

```bash
./build/astratag_detect
```

This will:
1. Load marker dictionary from `new_dictionary.json`
2. Load camera calibration from `intrinsic_astratag.json`
3. Process all images in `astratag_data/` directory
4. Save annotated results to `results/` directory
5. Print detection statistics

Expected output:
```
Loaded dictionary with 20 markers
Processing images from: astratag_data
Processing: frame_0145.jpg
  Status: DETECTED 1 marker(s) (1.26 seconds)
...
============================================================
Detection Statistics:
  Total images: 68
  Successfully detected: 67
  Detection rate: 98.53%
  Average processing time: 1.2643 seconds/image
  Average FPS: 0.79
============================================================
```

## Integrating into Your Project

### Option 1: Copy Header + Link Library

1. Copy headers to your include path:
```bash
cp -r include/astratag /your/project/include/
```

2. Copy library:
```bash
cp build/libastratag_lib.a /your/project/lib/
```

3. In your CMakeLists.txt:
```cmake
find_package(OpenCV REQUIRED)

include_directories(include/astratag ${OpenCV_INCLUDE_DIRS})
link_libraries(astratag_lib ${OpenCV_LIBS})

add_executable(your_app your_app.cpp)
target_link_libraries(your_app astratag_lib ${OpenCV_LIBS})
```

### Option 2: Add as Subdirectory

1. Add AstraTag as git submodule:
```bash
cd your_project
git submodule add <astratag-repo-url> external/astratag
```

2. In your CMakeLists.txt:
```cmake
add_subdirectory(external/astratag)

add_executable(your_app your_app.cpp)
target_link_libraries(your_app PRIVATE astratag_lib)
```

### Option 3: Install System-Wide

```bash
cd astratag/build
sudo make install  # (after adding install() commands to CMakeLists.txt)
```

## Minimal Example

Create `my_detector.cpp`:

```cpp
#include "detector.hpp"
#include "visualization.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace astratag;

int main() {
    try {
        // Load resources
        auto dictionary = load_tag_dictionary("data/new_dictionary.json");
        auto intrinsics = load_camera_intrinsics("config/my_camera.json");
        
        // Open camera or load image
        cv::VideoCapture cap(0);  // or cv::imread("image.jpg")
        cv::Mat frame;
        
        while (cap.read(frame)) {
            // Detect markers
            auto result = detect_tag(frame, dictionary);
            
            // Visualize results
            for (size_t i = 0; i < result.count; ++i) {
                draw_tag(frame, result.corners[i], result.indices[i]);
                
                auto pose = estimate_pose(result.corners[i], 
                                          result.world_locs[i], 
                                          intrinsics);
                if (pose.success) {
                    draw_pose_cube(frame, pose, result.corners[i], intrinsics);
                    
                    std::cout << "Marker " << result.indices[i] 
                              << " at distance: " << cv::norm(pose.tvec) 
                              << " meters\n";
                }
            }
            
            cv::imshow("Detection", frame);
            if (cv::waitKey(1) == 27) break;  // ESC to exit
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
```

Build and run:
```bash
g++ -std=c++17 my_detector.cpp -o my_detector \
    -I/path/to/astratag/include \
    -L/path/to/astratag/build \
    -lastratag_lib \
    $(pkg-config --cflags --libs opencv4)

./my_detector
```

## CMake Configuration

If you need custom OpenCV path, set it during cmake:

```bash
cmake -DOpenCV_DIR=/custom/opencv/path ..
```

Or modify CMakeLists.txt:
```cmake
set(OpenCV_DIR "/home/user/dev/opencv5")
find_package(OpenCV REQUIRED)
```

## Troubleshooting

### "Cannot find OpenCV"
```bash
export OpenCV_DIR=/usr/local
# or
cmake -DOpenCV_DIR=/path/to/opencv ..
```

### "Undefined reference to cv::..."
Ensure OpenCV is linked:
```cmake
target_link_libraries(your_app PRIVATE astratag_lib ${OpenCV_LIBS})
```

### "Cannot open dictionary file"
Ensure JSON files are in the working directory:
```bash
cp -r data config /path/to/working/dir/
```

Or use absolute paths in your code:
```cpp
auto dict = load_tag_dictionary("/absolute/path/to/data/new_dictionary.json");
```

### "No markers detected"
- Check image quality (blur, lighting, occlusion)
- Verify markers are in dictionary
- Try relaxing Hamming threshold: `detect_tag(image, dictionary, 8)`
- Ensure camera calibration matches your camera

## Performance Optimization

### Compile with optimizations:
```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

### Reduce image resolution:
```cpp
cv::Mat small;
cv::resize(image, small, cv::Size(), 0.5, 0.5);
auto result = detect_tag(small, dictionary);
```

### Use ROI (Region of Interest):
```cpp
cv::Rect roi(x, y, width, height);
cv::Mat cropped = image(roi);
auto result = detect_tag(cropped, dictionary);
```

### Multi-threading:
```cpp
#pragma omp parallel for
for (int i = 0; i < images.size(); ++i) {
    auto result = detect_tag(images[i], dictionary);
    // Process result...
}
```

## Required Files

For detection to work, you need:

1. **Dictionary** (`data/new_dictionary.json`) - Marker definitions
2. **Keypoints** (`data/keypoints.txt`) - Sampling regions for signatures
3. **Camera Intrinsics** (`config/camera_example.json`) - Only for pose estimation

Example directory structure:
```
your_app/
├── your_app_executable
├── data/
│   ├── new_dictionary.json
│   └── keypoints.txt
├── config/
│   └── my_camera.json  (your calibration)
└── images/
    └── test.jpg
```

See [docs/FILE_ORGANIZATION.md](FILE_ORGANIZATION.md) for detailed explanation of file organization.


## Support

For issues, questions, or contributions:
- Open an issue on GitHub
- Check existing documentation
- Review example code in `examples/`
