#include "visualization.hpp"
#include <iostream>

namespace astratag {

void draw_tag(cv::Mat& image, const DetectionResult& result) {
    std::cout << "Drawing " << result.indices.size() << " detected tags" << std::endl;
    
    for (size_t i = 0; i < result.indices.size(); ++i) {
        // Get corner points
        const auto& corner = result.corners[i];
        
        // Convert to integer coordinates for drawing
        std::vector<cv::Point> corner_int;
        for (const auto& pt : corner) {
            corner_int.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
        }
        
        // Draw contour (green)
        std::vector<std::vector<cv::Point>> contours = { corner_int };
        cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 0), 2);
        
        // Draw marker ID at first corner
        int marker_id = result.indices[i];
        cv::Point text_pos = corner_int[0];
        cv::putText(image, std::to_string(marker_id), text_pos, 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
}

PoseEstimate estimate_pose(const std::vector<cv::Point2f>& corners,
                          const std::vector<cv::Point3f>& world_points,
                          const CameraIntrinsics& intrinsics) {
    PoseEstimate pose;
    
    // Solve PnP to get rotation and translation
    bool success = cv::solvePnP(world_points, corners, 
                                intrinsics.camera_matrix, intrinsics.dist_coeffs,
                                pose.rvec, pose.tvec, false, cv::SOLVEPNP_ITERATIVE);
    
    pose.success = success;
    
    if (success) {
        // Calculate reprojection error
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(world_points, pose.rvec, pose.tvec,
                         intrinsics.camera_matrix, intrinsics.dist_coeffs,
                         projected_points);
        
        // Calculate mean Euclidean distance
        double total_error = 0.0;
        for (size_t i = 0; i < corners.size(); ++i) {
            cv::Point2f diff = corners[i] - projected_points[i];
            total_error += cv::norm(diff);
        }
        pose.reprojection_error = total_error / corners.size();
    }
    
    return pose;
}

void draw_axes_with_pose(cv::Mat& image, 
                         const DetectionResult& result,
                         const CameraIntrinsics& intrinsics) {
    // Define 3D axis points (25 units length)
    std::vector<cv::Point3f> axis = {
        cv::Point3f(0, 0, 0),      // Origin
        cv::Point3f(25, 0, 0),     // X-axis (red)
        cv::Point3f(0, 25, 0),     // Y-axis (green)
        cv::Point3f(0, 0, -25)     // Z-axis (blue)
    };
    
    std::cout << "Drawing axes for " << result.indices.size() << " markers" << std::endl;
    
    for (size_t i = 0; i < result.indices.size(); ++i) {
        // Get marker data
        const auto& corners = result.corners[i];
        const auto& world_points = result.world_locs[i];
        int marker_id = result.indices[i];
        
        // Estimate pose
        PoseEstimate pose = estimate_pose(corners, world_points, intrinsics);
        
        if (!pose.success) {
            std::cerr << "Failed to estimate pose for marker " << marker_id << std::endl;
            continue;
        }
        
        // Project axis points to image
        std::vector<cv::Point2f> img_pts;
        cv::projectPoints(axis, pose.rvec, pose.tvec,
                         intrinsics.camera_matrix, intrinsics.dist_coeffs,
                         img_pts);
        
        // Draw axes
        cv::Point origin(static_cast<int>(img_pts[0].x), static_cast<int>(img_pts[0].y));
        cv::Point x_end(static_cast<int>(img_pts[1].x), static_cast<int>(img_pts[1].y));
        cv::Point y_end(static_cast<int>(img_pts[2].x), static_cast<int>(img_pts[2].y));
        cv::Point z_end(static_cast<int>(img_pts[3].x), static_cast<int>(img_pts[3].y));
        
        // X-axis: Red
        cv::line(image, origin, x_end, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        // Y-axis: Green
        cv::line(image, origin, y_end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        // Z-axis: Blue
        cv::line(image, origin, z_end, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        
        // Draw marker ID at origin
        cv::putText(image, std::to_string(marker_id), origin,
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
}

void draw_pose_cube(cv::Mat& image,
                    const DetectionResult& result,
                    const CameraIntrinsics& intrinsics) {
    // Define cube points (30 units size)
    std::vector<cv::Point3f> cube_points = {
        // Bottom face
        cv::Point3f(0, 0, 0),
        cv::Point3f(0, 30, 0),
        cv::Point3f(30, 30, 0),
        cv::Point3f(30, 0, 0),
        // Top face
        cv::Point3f(0, 0, -30),
        cv::Point3f(0, 30, -30),
        cv::Point3f(30, 30, -30),
        cv::Point3f(30, 0, -30)
    };
    
    std::cout << "Drawing cubes for " << result.indices.size() << " markers" << std::endl;
    
    for (size_t i = 0; i < result.indices.size(); ++i) {
        // Get marker data
        const auto& corners = result.corners[i];
        const auto& world_points = result.world_locs[i];
        int marker_id = result.indices[i];
        
        // Estimate pose
        PoseEstimate pose = estimate_pose(corners, world_points, intrinsics);
        
        if (!pose.success) {
            std::cerr << "Failed to estimate pose for marker " << marker_id << std::endl;
            continue;
        }
        
        // Project cube points to image
        std::vector<cv::Point2f> img_pts;
        cv::projectPoints(cube_points, pose.rvec, pose.tvec,
                         intrinsics.camera_matrix, intrinsics.dist_coeffs,
                         img_pts);
        
        // Convert to integer points
        std::vector<cv::Point> pts;
        for (const auto& pt : img_pts) {
            pts.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
        }
        
        // Draw bottom square (blue)
        for (int j = 0; j < 4; ++j) {
            cv::line(image, pts[j], pts[(j + 1) % 4], cv::Scalar(255, 0, 0), 2);
        }
        
        // Draw top square (green)
        for (int j = 0; j < 4; ++j) {
            cv::line(image, pts[j + 4], pts[(j + 1) % 4 + 4], cv::Scalar(0, 255, 0), 2);
        }
        
        // Draw vertical lines (red)
        for (int j = 0; j < 4; ++j) {
            cv::line(image, pts[j], pts[j + 4], cv::Scalar(0, 0, 255), 2);
        }
        
        // Draw marker ID
        cv::putText(image, std::to_string(marker_id), pts[0],
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }
}

} // namespace astratag
