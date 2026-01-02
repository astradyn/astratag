// Copyright (c) Astradyn Systems LLP.
// SPDX-License-Identifier: Apache-2.0

#ifndef ASTRATAG_VISUALIZATION_HPP
#define ASTRATAG_VISUALIZATION_HPP

#include "types.hpp"
#include <opencv2/opencv.hpp>

namespace astratag {

/**
 * @brief Draw detected tags on image with bounding boxes and IDs
 * 
 * @param image Image to draw on (will be modified)
 * @param result Detection result containing corners and indices
 */
void draw_tag(cv::Mat& image, const DetectionResult& result);

/**
 * @brief Draw 3D coordinate axes on detected markers
 * 
 * Draws X (red), Y (green), and Z (blue) axes at the origin of each
 * detected marker using pose estimation.
 * 
 * @param image Image to draw on (will be modified)
 * @param result Detection result containing corners, indices, and world points
 * @param intrinsics Camera calibration parameters
 */
void draw_axes_with_pose(cv::Mat& image, 
                         const DetectionResult& result,
                         const CameraIntrinsics& intrinsics);

/**
 * @brief Draw 3D cube overlay on detected markers
 * 
 * Draws a 3D cube on each detected marker to visualize pose estimation.
 * 
 * @param image Image to draw on (will be modified)
 * @param result Detection result containing corners, indices, and world points
 * @param intrinsics Camera calibration parameters
 */
void draw_pose_cube(cv::Mat& image,
                    const DetectionResult& result,
                    const CameraIntrinsics& intrinsics);

/**
 * @brief Estimate pose for a single marker
 * 
 * Uses cv::solvePnP to estimate 6DOF pose of the marker.
 * 
 * @param corners 2D image coordinates of marker corners
 * @param world_points 3D world coordinates of marker corners
 * @param intrinsics Camera calibration parameters
 * @return PoseEstimate Structure containing rotation, translation, and error
 */
PoseEstimate estimate_pose(const std::vector<cv::Point2f>& corners,
                          const std::vector<cv::Point3f>& world_points,
                          const CameraIntrinsics& intrinsics);

} // namespace astratag

#endif // ASTRATAG_VISUALIZATION_HPP
