# AstraTag Configuration Files

This directory contains **user-specific** configuration files, primarily camera calibration data.

## Files

### `camera_example.json`
**Purpose**: Example camera calibration file showing the expected format.

**Contents**:
- Camera intrinsic matrix (fx, fy, cx, cy)
- Distortion coefficients (k1, k2, p1, p2, k3)

**Format**:
```json
{
  "camera_matrix": [
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
  ],
  "dist_coeffs": [k1, k2, p1, p2, k3]
}
```

## Calibrating Your Camera

Camera calibration is **required** for accurate 6DOF pose estimation. Without it, you can still detect markers but not estimate their 3D position/orientation.

### Option 1: OpenCV Calibration Tool

```bash
# Collect calibration images (checkerboard pattern)
# Use OpenCV's calibration example
python opencv_calibrate.py --images calib_*.jpg --pattern 9x6 --square_size 0.025

# This outputs camera matrix and distortion coefficients
# Save them to config/my_camera.json
```

### Option 2: MATLAB/Calibration Toolbox

1. Use MATLAB's Camera Calibrator app
2. Export intrinsics and distortion
3. Convert to JSON format

### Option 3: ROS Camera Calibration

```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108
# Export results and convert to JSON
```

## Using Your Calibration

```cpp
// Load your camera calibration
auto intrinsics = load_camera_intrinsics("config/my_camera.json");

// Use in pose estimation
auto pose = estimate_pose(corners, world_points, intrinsics);
```

## Multiple Cameras

If you have multiple cameras, create separate config files:

```
config/
├── camera_example.json
├── webcam_logitech.json
├── iphone_13_main.json
└── realsense_d435.json
```

Load the appropriate one for your camera:
```cpp
auto intrinsics = load_camera_intrinsics("config/webcam_logitech.json");
```

## Notes

- **Don't commit** your personal calibration files to git (add `config/*.json` to `.gitignore`, except example)
- Calibration is camera-specific (different for each physical camera)
- Recalibrate if you change lens, focal length, or resolution
- Poor calibration leads to inaccurate pose estimates

## Skipping Calibration

If you only need **marker detection** (not pose estimation), you can skip calibration:

```cpp
// Detection works without calibration
auto dictionary = load_tag_dictionary("data/new_dictionary.json");
auto result = detect_tag(image, dictionary);

// Just draw marker IDs (no pose cube)
for (size_t i = 0; i < result.count; ++i) {
    draw_tag(image, result.corners[i], result.indices[i]);
}
```

For pose estimation, calibration is essential.
