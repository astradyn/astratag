#ifndef ASTRATAG_DETECTOR_HPP
#define ASTRATAG_DETECTOR_HPP

#include "types.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>

namespace astratag {

/**
 * @brief Extract binary signature from a contour region
 * 
 * Applies perspective transform to normalize the marker region and extracts
 * the binary signature using the mask method.
 * 
 * @param image Input color image
 * @param contour Sub-pixel accuracy contour points [4 corners]
 * @param bits Resolution for warped marker (e.g., 700)
 * @param keypoints_file Path to keypoints file
 * @return std::string Binary signature string
 */
std::string get_contour_bits(const cv::Mat& image, 
                             const std::vector<cv::Point2f>& contour,
                             int bits,
                             const std::string& keypoints_file = "keypoints.txt");

/**
 * @brief Detect tags in an image using dictionary matching
 * 
 * Finds all quadrilateral contours, extracts their signatures, and matches
 * against the tag dictionary using Hamming distance.
 * 
 * @param image Input color image
 * @param tag_dictionary Dictionary mapping marker IDs to orientation data
 * @param allowed_misses Maximum Hamming distance for a match (default: 5)
 * @return DetectionResult Structure containing detected corners, indices, and world points
 */
DetectionResult detect_tag(const cv::Mat& image,
                           const std::map<int, MarkerData>& tag_dictionary,
                           int allowed_misses = 5);

/**
 * @brief Load tag dictionary from JSON file
 * 
 * Loads the dictionary file containing marker signatures and world points
 * for all marker IDs and orientations.
 * 
 * @param filepath Path to dictionary JSON file
 * @return std::map<int, MarkerData> Dictionary mapping marker IDs to data
 */
std::map<int, MarkerData> load_tag_dictionary(const std::string& filepath);

/**
 * @brief Load tag dictionary from JSON string (for embedded data)
 * 
 * Parses dictionary from a JSON string instead of file.
 * Useful for embedded data or network-loaded dictionaries.
 * 
 * @param json_string JSON string containing dictionary data
 * @return std::map<int, MarkerData> Dictionary mapping marker IDs to data
 */
std::map<int, MarkerData> load_tag_dictionary_from_string(const std::string& json_string);

/**
 * @brief Load embedded tag dictionary
 * 
 * Loads the dictionary compiled into the library at build time.
 * No external file needed.
 * 
 * @return std::map<int, MarkerData> Dictionary mapping marker IDs to data
 */
std::map<int, MarkerData> load_embedded_tag_dictionary();

/**
 * @brief Load camera intrinsics from JSON file
 * 
 * @param filepath Path to camera intrinsics JSON file
 * @return CameraIntrinsics Structure containing camera matrix and distortion coefficients
 */
CameraIntrinsics load_camera_intrinsics(const std::string& filepath);

} // namespace astratag

#endif // ASTRATAG_DETECTOR_HPP
