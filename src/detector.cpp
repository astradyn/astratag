// Copyright (c) Astradyn Systems LLP.
// SPDX-License-Identifier: Apache-2.0

#include "detector.hpp"
#include "signature.hpp"
#include "quadrilateral.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#ifdef ASTRATAG_USE_EMBEDDED_DATA
#include "embedded_data.hpp"
#endif

using json = nlohmann::json;

namespace astratag {

std::string get_contour_bits(const cv::Mat& image, 
                             const std::vector<cv::Point2f>& contour,
                             int bits,
                             const std::string& keypoints_file) {
    // Ensure contour has exactly 4 points
    if (contour.size() != 4) {
        throw std::runtime_error("Contour must have exactly 4 points");
    }
    
    // Python uses unordered approx points directly with getPerspectiveTransform
    // Let's try the same approach - just use the contour as-is
    std::vector<cv::Point2f> ordered_contour = contour;
    
    // Define corners of output normalized image
    std::vector<cv::Point2f> corners = {
        cv::Point2f(0, 0),
        cv::Point2f(bits, 0),
        cv::Point2f(bits, bits),
        cv::Point2f(0, bits)
    };
    
    // Get perspective transform matrix
    cv::Mat M = cv::getPerspectiveTransform(ordered_contour, corners);
    
    // Warp perspective to normalize marker
    cv::Mat warped;
    cv::warpPerspective(image, warped, M, cv::Size(bits, bits), cv::INTER_LINEAR);
    
    // Convert to grayscale
    cv::Mat gray;
    if (warped.channels() == 3) {
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = warped.clone();
    }
    
    // Apply Otsu's thresholding
    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    
    // Optional: Enable debug output by uncommenting these lines
    // static int debug_counter = 0;
    // if (debug_counter < 3) {
    //     std::string suffix = std::to_string(debug_counter);
    //     cv::imwrite("cpp_marker_transformed_" + suffix + ".png", gray);
    //     cv::imwrite("cpp_marker_warpedthresh_" + suffix + ".png", binary);
    //     debug_counter++;
    // }
    
    // Extract signature using mask method
    std::string signature = get_id(binary, keypoints_file);
    
    return signature;
}

DetectionResult detect_tag(const cv::Mat& image,
                           const std::map<int, MarkerData>& tag_dictionary,
                           int allowed_misses) {
    DetectionResult result;
    
    // Detect quadrilateral candidates using existing detector
    QuadrilateralDetector detector;
    std::vector<std::vector<cv::Point2f>> candidates = detector.detectQuadrilaterals(image);
    
    std::cout << "Found " << candidates.size() << " quadrilateral candidates" << std::endl;
    
    // Process each candidate contour
    for (size_t cant_num = 0; cant_num < candidates.size(); ++cant_num) {
        const auto& candidate = candidates[cant_num];
        
        // Extract signature from this candidate
        std::string detected_sig;
        try {
            detected_sig = get_contour_bits(image, candidate, 700, "keypoints.txt");
        } catch (const std::exception& e) {
            std::cerr << "Error extracting signature from candidate " << cant_num 
                      << ": " << e.what() << std::endl;
            continue;
        }
        
        // Match against dictionary
        for (const auto& dict_entry : tag_dictionary) {
            int marker_id = dict_entry.first;
            const MarkerData& marker_data = dict_entry.second;
            
            // Check all 4 orientations
            for (const auto& orientation_entry : marker_data.orientations) {
                const MarkerOrientation& orientation_data = orientation_entry.second;
                
                // Calculate Hamming distance
                int distance = hamming_distance(detected_sig, orientation_data.signature);
                
                // Check if match is within threshold
                if (distance <= allowed_misses) {
                    // Store detection result
                    result.corners.push_back(candidate);
                    result.indices.push_back(marker_id);
                    result.world_locs.push_back(orientation_data.world_points);
                    result.count++;
                    
                    // Break after first match for this candidate
                    goto next_candidate;
                }
            }
        }
        
        next_candidate:
        continue;
    }
    
    return result;
}

std::map<int, MarkerData> load_tag_dictionary(const std::string& filepath) {
    std::map<int, MarkerData> dictionary;
    
    // Open and parse JSON file
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open dictionary file: " + filepath);
    }
    
    json j;
    file >> j;
    file.close();
    
    // Parse each marker in the dictionary
    for (auto& [key, value] : j.items()) {
        int marker_id = std::stoi(key);
        MarkerData marker_data;
        
        // Parse each orientation (0, 90, 180, 270)
        for (auto& [orientation, orientation_data] : value.items()) {
            MarkerOrientation orient;
            
            // Extract signature
            orient.signature = orientation_data["signature"].get<std::string>();
            
            // Extract world points (4 corners, each with x, y, z)
            auto world_points_json = orientation_data["world_points"];
            for (const auto& point : world_points_json) {
                float x = point[0].get<float>();
                float y = point[1].get<float>();
                float z = point[2].get<float>();
                orient.world_points.push_back(cv::Point3f(x, y, z));
            }
            
            // Store this orientation
            marker_data.orientations[orientation] = orient;
        }
        
        // Store this marker
        dictionary[marker_id] = marker_data;
    }
    
    std::cout << "Loaded dictionary with " << dictionary.size() << " markers" << std::endl;
    return dictionary;
}

std::map<int, MarkerData> load_tag_dictionary_from_string(const std::string& json_string) {
    std::map<int, MarkerData> dictionary;
    
    // Parse JSON string
    json j = json::parse(json_string);
    
    // Iterate through each marker in dictionary
    for (auto& [key, value] : j.items()) {
        int marker_id = std::stoi(key);
        MarkerData marker_data;
        
        // Parse each orientation (0, 90, 180, 270)
        for (auto& [orientation, orientation_data] : value.items()) {
            MarkerOrientation orient;
            
            // Extract signature
            orient.signature = orientation_data["signature"].get<std::string>();
            
            // Extract world points
            auto world_points_json = orientation_data["world_points"];
            for (const auto& point : world_points_json) {
                float x = point[0].get<float>();
                float y = point[1].get<float>();
                float z = point[2].get<float>();
                orient.world_points.push_back(cv::Point3f(x, y, z));
            }
            
            marker_data.orientations[orientation] = orient;
        }
        
        dictionary[marker_id] = marker_data;
    }
    
    return dictionary;
}

std::map<int, MarkerData> load_embedded_tag_dictionary() {
#ifdef ASTRATAG_USE_EMBEDDED_DATA
    return load_tag_dictionary_from_string(embedded::DICTIONARY_JSON);
#else
    throw std::runtime_error("Embedded data not available. Rebuild with -DASTRATAG_EMBED_DATA=ON");
#endif
}

CameraIntrinsics load_camera_intrinsics(const std::string& filepath) {
    CameraIntrinsics intrinsics;
    
    // Open and parse JSON file
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open camera intrinsics file: " + filepath);
    }
    
    json j;
    file >> j;
    file.close();
    
    // Parse camera matrix (3x3)
    auto cam_matrix = j["camera_matrix"];
    intrinsics.camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            intrinsics.camera_matrix.at<double>(i, j) = cam_matrix[i][j].get<double>();
        }
    }
    
    // Parse distortion coefficients
    auto dist_coeffs = j["distortion_coefficients"];
    int num_coeffs = dist_coeffs[0].size();
    intrinsics.dist_coeffs = cv::Mat(1, num_coeffs, CV_64F);
    for (int i = 0; i < num_coeffs; ++i) {
        intrinsics.dist_coeffs.at<double>(0, i) = dist_coeffs[0][i].get<double>();
    }
    
    std::cout << "Loaded camera intrinsics:" << std::endl;
    std::cout << "  Camera matrix: " << intrinsics.camera_matrix << std::endl;
    std::cout << "  Distortion coeffs: " << intrinsics.dist_coeffs << std::endl;
    
    return intrinsics;
}

} // namespace astratag
