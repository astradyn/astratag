# AstraTag
### Multi-Range Fiducial Marker for Spacecraft Rendezvous, Proximity Operations and Docking

This repository contains the implementation of multi-range, recursive fiducial marker AstraTag. The marker template is derived from square shaped Spidron. The marker is designed for in-orbit operations around cooperative targets. The objective of introducing recurisve feature is to make it suitable for conducting rendezvous and proximity operation. Its three layers provide robustness against a partial occlusion. Below are some of the sample markers. 

<div style="display: flex; justify-content: space-between;">
    <img src="sample/marker_1.png" width="200">
    <img src="sample/marker_2.png" width="200">
    <img src="sample/marker_3.png" width="200">
</div>

To test the marker, a space like ligthing condition was created a pair of mock-up spacecrafts were used as a target. The images below demonstrates a sample result. The setup is placed on turntable to simulate out-of-plane rotation. 

<div style="display:flex; gap:12px; justify-content:center; align-items:flex-start; flex-wrap:wrap;">
   <img src="test/frame_0001.png" alt="frame 0001" style="max-width:48%; height:auto;">
   <img src="test/frame_0103.png" alt="frame 0103" style="max-width:48%; height:auto;">
</div>

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
├── src/                    
│   ├── main.cpp           
│   └── quadrilateral.cpp  
├── include/               
│   └── quadrilateral.hpp  
├── sample/                
├── scripts/              
│   └── run_detector.sh    
```

### Sample Usage

The detector will automatically process all supported image files in the `test_images/` directory and output the results with detected quadrilaterals highlighted in green. 
