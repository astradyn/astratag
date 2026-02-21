// Copyright (c) Astradyn Systems LLP.
// SPDX-License-Identifier: Apache-2.0

#ifndef ASTRATAG_SIGNATURE_HPP
#define ASTRATAG_SIGNATURE_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace astratag {

/**
 * @brief Extract binary signature from binary marker image using mask method
 * 
 * This function reads keypoints from 'keypoints.txt' and samples the binary image
 * at triangular regions to extract a binary signature string.
 * 
 * @param binary_image Binary thresholded marker image
 * @param keypoints_file Path to keypoints file (default: "keypoints.txt")
 * @return std::string Binary signature as string of '0' and '1' characters
 */
std::string get_id(const cv::Mat& binary_image, const std::string& keypoints_file = "keypoints.txt", float scale = 1.0f);

/**
 * @brief Calculate Hamming distance between two binary signature strings
 * 
 * @param sig1 First binary signature string
 * @param sig2 Second binary signature string
 * @return int Number of differing bits (Hamming distance)
 * @throws std::invalid_argument if strings have different lengths
 */
int hamming_distance(const std::string& sig1, const std::string& sig2);

/**
 * @brief Load triangular sampling regions from file
 * 
 * Reads keypoints file defining triangular regions for signature extraction.
 * Each line: "triangle_N,x1,y1,x2,y2,x3,y3"
 * 
 * @param filename Path to keypoints file
 * @param scale Scale factor for coordinates (default: 1.0)
 * @return std::vector<std::vector<cv::Point2f>> Vector of triangular regions
 */
std::vector<std::vector<cv::Point2f>> load_keypoints(const std::string& filename, float scale = 1.0);

/**
 * @brief Load triangular sampling regions from string (for embedded data)
 * 
 * Parses keypoints from a string instead of file.
 * 
 * @param data Keypoints data as string
 * @param scale Scale factor for coordinates (default: 1.0)
 * @return std::vector<std::vector<cv::Point2f>> Vector of triangular regions
 */
std::vector<std::vector<cv::Point2f>> load_keypoints_from_string(const std::string& data, float scale = 1.0);

/**
 * @brief Load embedded triangular sampling regions
 * 
 * Loads keypoints compiled into the library at build time.
 * No external file needed.
 * 
 * @param scale Scale factor for coordinates (default: 1.0)
 * @return std::vector<std::vector<cv::Point2f>> Vector of triangular regions
 */
std::vector<std::vector<cv::Point2f>> load_embedded_keypoints(float scale = 1.0);

} // namespace astratag

#endif // ASTRATAG_SIGNATURE_HPP
