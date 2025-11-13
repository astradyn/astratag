#ifndef ASTRATAG_TYPES_HPP
#define ASTRATAG_TYPES_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

namespace astratag {

// Marker orientation data for a single orientation (0°, 90°, 180°, 270°)
struct MarkerOrientation {
    std::string signature;                      // Binary signature string
    std::vector<cv::Point3f> world_points;      // 3D world coordinates [4 corners]
};

// Complete marker data with all 4 orientations
struct MarkerData {
    std::map<std::string, MarkerOrientation> orientations; // Keys: "0", "90", "180", "270"
};

// Detection result for a single image
struct DetectionResult {
    std::vector<std::vector<cv::Point2f>> corners;     // Detected corner points for each marker
    std::vector<int> indices;                          // Marker IDs
    std::vector<std::vector<cv::Point3f>> world_locs;  // Corresponding 3D world points
    int count;                                         // Detection status (0=missed, >0=detected)
    
    // Constructor
    DetectionResult() : count(0) {}
};

// Camera calibration parameters
struct CameraIntrinsics {
    cv::Mat camera_matrix;      // 3x3 intrinsic matrix
    cv::Mat dist_coeffs;        // Distortion coefficients
    
    // Constructor
    CameraIntrinsics() {}
};

// Pose estimation result
struct PoseEstimate {
    cv::Mat rvec;               // Rotation vector (3x1)
    cv::Mat tvec;               // Translation vector (3x1)
    double reprojection_error;  // Reprojection error in pixels
    bool success;               // Whether pose estimation succeeded
    
    // Constructor
    PoseEstimate() : reprojection_error(0.0), success(false) {}
};

} // namespace astratag

#endif // ASTRATAG_TYPES_HPP
