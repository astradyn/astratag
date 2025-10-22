# AstraTag
### Multi-Range Fiducial Marker for Spacecraft Rendezvous, Proximity Operations and Docking

This repository contains the implementation of multi-range, recursive fiducial marker AstraTag. The marker template is derived from square shaped Spidron. The marker is designed for in-orbit operations around cooperative targets. The objective of introducing recurisve feature is to make it suitable for conducting rendezvous and proximity operation. Its three layers provide robustness against a partial occlusion. Below are some of the sample markers. 

<div style="display: flex; justify-content: space-between;">
    <img src="sample/marker_1.png" width="200">
    <img src="sample/marker_2.png" width="200">
    <img src="sample/marker_3.png" width="200">
</div>

To test the marker, a space like ligthing condition was created. A cube shaped box was used as a mock-up spacecraft. The images below demonstrates a sample result. The setup is placed on turntable to simulate out-of-place rotation. 

<img src="test/enhanced.png" width="600">

<img src="test/result.png" width="600">

AstraTag markers can also be used in robotics application in terrestial and underwater environments. 

### Requirements

- **C++17** or later
- **OpenCV 4.x** (configured in CMakeLists.txt)
- **CMake 3.16** or later
- **Build tools** (make, gcc/clang)

### Building the Project

1. **Clone the repository:**
   ```bash
   git clone https://github.com/ravikt/astratag.git
   cd astratag
   ```

2. **Update OpenCV path** (if needed):
   Edit `CMakeLists.txt` and update the OpenCV_DIR path to match your OpenCV installation:
   ```cmake
   set(OpenCV_DIR "/path/to/your/opencv")
   ```

3. **Create build directory and compile:**
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

### Running the Detector

#### Option 1: Using the convenience script (Recommended)
```bash
./scripts/run_detector.sh
```

#### Option 2: Manual execution
```bash
# Run from the project root directory
./build/quadrilateral_detector
```

#### Input and Output

- **Input**: Place test images in the `test_images/` folder
- **Output**: Processed results will be saved in the `results/` folder
- **Supported formats**: PNG, JPG, JPEG

### Project Structure

```
astratag/
├── src/                    # Source code
│   ├── main.cpp           # Main application
│   └── quadrilateral.cpp  # Detection implementation
├── include/               # Header files
│   └── quadrilateral.hpp  # Class definitions
├── sample/                # Sample marker images (20 markers)
├── test_images/           # Test input images (not in version control)
├── results/               # Output directory (not in version control)
├── scripts/               # Utility scripts
│   └── run_detector.sh    # Convenience script to run detector
└── build/                 # Build artifacts (not in version control)
```

### Sample Usage

The detector will automatically process all supported image files in the `test_images/` directory and output the results with detected quadrilaterals highlighted in green. 
