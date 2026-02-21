// Copyright (c) Astradyn Systems LLP.
// SPDX-License-Identifier: Apache-2.0

#include "detector.hpp"
#include "visualization.hpp"
#include "types.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <opencv2/calib3d.hpp>

namespace fs = std::filesystem;
using namespace astratag;

int main(int argc, char** argv) {
    try {
        // Configuration — accept optional CLI overrides:
        //   detect_tags <image_folder> [dict_path] [intrinsics_path]
        const std::string test_folder      = (argc > 1) ? argv[1] : "examples/astratag_data";
        const std::string dict_path        = (argc > 2) ? argv[2] : "data/new_dictionary.json";
        const std::string intrinsics_path  = (argc > 3) ? argv[3] : "config/camera_example.json";
        const std::string output_folder    = (argc > 4) ? argv[4] : "results";
        const int allowed_misses = 10;  // Hamming distance threshold
        
        // Create output directory
        fs::create_directories(output_folder);
        
        // Load tag dictionary and camera intrinsics
        std::cout << "Loading tag dictionary from: " << dict_path << std::endl;
        auto tag_dictionary = load_tag_dictionary(dict_path);
        std::cout << "Loaded dictionary with " << tag_dictionary.size() << " markers" << std::endl;
    
        // Load camera intrinsics
        std::cout << "Loading camera intrinsics from: " << intrinsics_path << std::endl;
        auto camera_intrinsics = load_camera_intrinsics(intrinsics_path);
        
        // Statistics tracking
        int image_count = 0;
        int missed_detections = 0;
        double total_time = 0.0;
        
        auto start_time_all = std::chrono::high_resolution_clock::now();
        
        // Process all images in test folder
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "Processing images from: " << test_folder << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        for (const auto& entry : fs::directory_iterator(test_folder)) {
            if (!entry.is_regular_file()) continue;
            
            std::string filepath = entry.path().string();
            std::string extension = entry.path().extension().string();
            
            // Check if file is an image
            if (extension != ".png" && extension != ".jpg" && 
                extension != ".jpeg" && extension != ".PNG" && 
                extension != ".JPG" && extension != ".JPEG") {
                continue;
            }
            
            std::string filename = entry.path().filename().string();
            std::string output_path = output_folder + "/" + filename;
            
            std::cout << "\nProcessing: " << filename << std::endl;
            
            // Read image
            cv::Mat image = cv::imread(filepath);
            if (image.empty()) {
                std::cerr << "Failed to read image: " << filepath << std::endl;
                continue;
            }
            
            // Time the detection
            auto start_time = std::chrono::high_resolution_clock::now();
            
            // Detect tags
            DetectionResult result = detect_tag(image, tag_dictionary, allowed_misses);
            
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            double process_time = elapsed.count();
            
            // Update statistics
            total_time += process_time;
            image_count++;
            
            if (result.count == 0) {
                std::cout << "  Status: MISSED (" << process_time << " seconds)" << std::endl;
                missed_detections++;
            } else {
                std::cout << "  Status: DETECTED " << result.count << " marker(s) ("
                         << process_time << " seconds)" << std::endl;

                // Pose estimation + reprojection error
                if (!result.corners.empty() && !result.world_locs.empty()) {
                    cv::Mat rvec, tvec;
                    bool pose_ok = cv::solvePnP(
                        result.world_locs[0], result.corners[0],
                        camera_intrinsics.camera_matrix,
                        camera_intrinsics.dist_coeffs,
                        rvec, tvec,
                        false, cv::SOLVEPNP_IPPE);
                    if (pose_ok) {
                        std::vector<cv::Point2f> projected;
                        cv::projectPoints(result.world_locs[0], rvec, tvec,
                                          camera_intrinsics.camera_matrix,
                                          camera_intrinsics.dist_coeffs, projected);
                        double reproj = 0.0;
                        for (size_t k = 0; k < result.corners[0].size(); k++) {
                            double dx = projected[k].x - result.corners[0][k].x;
                            double dy = projected[k].y - result.corners[0][k].y;
                            reproj += std::sqrt(dx*dx + dy*dy);
                        }
                        reproj /= result.corners[0].size();
                        std::cout << "  Pose:"
                                  << " tx=" << tvec.at<double>(0)
                                  << " ty=" << tvec.at<double>(1)
                                  << " tz=" << tvec.at<double>(2)
                                  << " rx=" << rvec.at<double>(0)
                                  << " ry=" << rvec.at<double>(1)
                                  << " rz=" << rvec.at<double>(2)
                                  << " reproj=" << reproj << std::endl;
                    }
                }
            }
            
            // Visualize results
            cv::Mat output_image = image.clone();
            
            if (result.count > 0) {
                // Draw bounding boxes and IDs
                draw_tag(output_image, result);
                
                // Draw 3D pose visualization (cube)
                draw_pose_cube(output_image, result, camera_intrinsics);
                
                // Alternative: Draw axes instead of cube
                // draw_axes_with_pose(output_image, result, camera_intrinsics);
            }
            
            // Save output
            cv::imwrite(output_path, output_image);
        }
        
        auto end_time_all = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> total_elapsed = end_time_all - start_time_all;
        double total_all_time = total_elapsed.count();
        
        // Print statistics
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "PROCESSING STATISTICS" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        if (image_count > 0) {
            double success_rate = 100.0 * (image_count - missed_detections) / image_count;
            double avg_time = total_time / image_count;
            double fps = 1.0 / avg_time;
            
            std::cout << "Total images processed: " << image_count << std::endl;
            std::cout << "Missed detections: " << missed_detections << "/" << image_count 
                     << " (" << (100.0 * missed_detections / image_count) << "%)" << std::endl;
            std::cout << "Success rate: " << success_rate << "%" << std::endl;
            std::cout << "\nTiming:" << std::endl;
            std::cout << "  Total detection time: " << total_time << " seconds" << std::endl;
            std::cout << "  Average time per image: " << avg_time << " seconds" << std::endl;
            std::cout << "  Equivalent FPS: " << fps << std::endl;
            std::cout << "  Total time (including I/O): " << total_all_time << " seconds" << std::endl;
        } else {
            std::cout << "No images processed!" << std::endl;
        }
        
        std::cout << "\nResults saved to: " << output_folder << "/" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}