// Copyright (c) Astradyn Systems LLP.
// SPDX-License-Identifier: Apache-2.0

#include "detector.hpp"
#include "signature.hpp"
#include "quadrilateral.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/shape.hpp>
#include <fstream>
#include <iostream>

#ifdef ASTRATAG_USE_EMBEDDED_DATA
#include "embedded_data.hpp"
#endif

using json = nlohmann::json;

namespace astratag {

// ---------------------------------------------------------------------------
// Curved-surface re-warp helpers
// ---------------------------------------------------------------------------

// Order 4 points as TL, TR, BR, BL using sum/diff heuristic
static std::vector<cv::Point2f> order_quad_corners(const std::vector<cv::Point2f>& pts) {
    std::vector<cv::Point2f> ordered(4);
    std::vector<float> sums(4), diffs(4);
    for (int i = 0; i < 4; ++i) {
        sums[i]  = pts[i].x + pts[i].y;
        diffs[i] = pts[i].y - pts[i].x;
    }
    int tl = std::min_element(sums.begin(),  sums.end())  - sums.begin();
    int br = std::max_element(sums.begin(),  sums.end())  - sums.begin();
    int tr = std::min_element(diffs.begin(), diffs.end()) - diffs.begin();
    int bl = std::max_element(diffs.begin(), diffs.end()) - diffs.begin();
    ordered[0] = pts[tl]; ordered[1] = pts[tr];
    ordered[2] = pts[br]; ordered[3] = pts[bl];
    return ordered;
}

// Back-project a point from warped 700x700 space to original image space
static cv::Point2f backproject(const cv::Mat& M_inv, const cv::Point2f& wp) {
    cv::Mat pt = (cv::Mat_<double>(3,1) << wp.x, wp.y, 1.0);
    cv::Mat h = M_inv * pt;
    return cv::Point2f(h.at<double>(0) / h.at<double>(2),
                       h.at<double>(1) / h.at<double>(2));
}

// Detect a white rectangular border in the binary warped image.
// Returns ordered {TL, TR, BR, BL} corners or empty vector if not found.
static std::vector<cv::Point2f> detect_border_corners(
    const cv::Mat& binary_warp,
    double expected_area,
    double area_tolerance,
    float center_radius)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_warp.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    double best_area_diff = 1e9;
    std::vector<cv::Point2f> best_corners;
    cv::Point2f center(350.0f, 350.0f);

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < expected_area * (1.0 - area_tolerance) ||
            area > expected_area * (1.0 + area_tolerance))
            continue;

        cv::Moments m = cv::moments(c);
        if (m.m00 < 1.0) continue;
        cv::Point2f centroid(m.m10 / m.m00, m.m01 / m.m00);
        if (cv::norm(centroid - center) > center_radius)
            continue;

        std::vector<cv::Point> approx;
        double eps = 0.05 * cv::arcLength(c, true);
        cv::approxPolyDP(c, approx, eps, true);
        if (approx.size() != 4) continue;

        double diff = std::abs(area - expected_area);
        if (diff < best_area_diff) {
            best_area_diff = diff;
            std::vector<cv::Point2f> pts;
            for (const auto& p : approx) pts.emplace_back(p.x, p.y);
            best_corners = order_quad_corners(pts);
        }
    }
    return best_corners;
}

// Attempt a TPS-based curved surface re-warp.
// Returns corrected 700x700 grayscale image, or empty Mat on failure.
static cv::Mat attempt_curved_rewarp(
    const cv::Mat& image,
    const std::vector<cv::Point2f>& outer_corners,
    const cv::Mat& M,
    const cv::Mat& initial_binary)
{
    const int WARP_SIZE = 700;

    // Detect middle and inner border corners in the warped binary image
    auto middle_warp = detect_border_corners(initial_binary, 90000.0, 0.5, 200.0f);
    auto inner_warp  = detect_border_corners(initial_binary, 15625.0, 0.5, 120.0f);

    // Need at least the middle border (8 total correspondences)
    if (middle_warp.size() != 4) {
        std::cout << "    TPS border detection failed: middle=" << middle_warp.size()
                  << " inner=" << inner_warp.size() << std::endl;
        return cv::Mat();
    }

    // Inverse homography for back-projection
    cv::Mat M_inv = M.inv();

    // Build correspondences: marker-space → original-image-space
    // Outer corners (4 pts)
    std::vector<cv::Point2f> marker_pts = {
        {0, 0}, {(float)WARP_SIZE, 0},
        {(float)WARP_SIZE, (float)WARP_SIZE}, {0, (float)WARP_SIZE}
    };
    std::vector<cv::Point2f> orig_pts = {
        outer_corners[0], outer_corners[1],
        outer_corners[2], outer_corners[3]
    };

    // Middle border corners (4 pts)
    const std::vector<cv::Point2f> ideal_middle = {
        {200, 200}, {500, 200}, {500, 500}, {200, 500}
    };
    for (int i = 0; i < 4; ++i) {
        marker_pts.push_back(ideal_middle[i]);
        orig_pts.push_back(backproject(M_inv, middle_warp[i]));
    }

    // Inner border corners (4 pts, if available)
    if (inner_warp.size() == 4) {
        const std::vector<cv::Point2f> ideal_inner = {
            {287, 287}, {412, 287}, {412, 412}, {287, 412}
        };
        for (int i = 0; i < 4; ++i) {
            marker_pts.push_back(ideal_inner[i]);
            orig_pts.push_back(backproject(M_inv, inner_warp[i]));
        }
    }

    int N = (int)marker_pts.size();
    std::cout << "    TPS rewarp with " << N << " correspondences" << std::endl;

    // Fit TPS: marker-space → original-image-space
    auto tps = cv::createThinPlateSplineShapeTransformer(0.0);
    cv::Mat tps_src(1, N, CV_32FC2), tps_dst(1, N, CV_32FC2);
    for (int i = 0; i < N; ++i) {
        tps_src.at<cv::Vec2f>(0, i) = {marker_pts[i].x, marker_pts[i].y};
        tps_dst.at<cv::Vec2f>(0, i) = {orig_pts[i].x, orig_pts[i].y};
    }
    std::vector<cv::DMatch> matches;
    for (int i = 0; i < N; ++i) matches.emplace_back(i, i, 0.0f);
    tps->estimateTransformation(tps_src, tps_dst, matches);

    // Build sparse remap grid (step=10 → 70x70 = 4900 points)
    const int STEP = 10;
    const int GRID = WARP_SIZE / STEP;
    int total = GRID * GRID;
    cv::Mat grid(1, total, CV_32FC2);
    int idx = 0;
    for (int gy = 0; gy < GRID; ++gy)
        for (int gx = 0; gx < GRID; ++gx)
            grid.at<cv::Vec2f>(0, idx++) = {(float)(gx * STEP), (float)(gy * STEP)};

    cv::Mat mapped;
    tps->applyTransformation(grid, mapped);

    // Reshape into 70x70 maps, resize to 700x700
    cv::Mat map_x_sparse(GRID, GRID, CV_32F);
    cv::Mat map_y_sparse(GRID, GRID, CV_32F);
    for (int i = 0; i < total; ++i) {
        map_x_sparse.at<float>(i / GRID, i % GRID) = mapped.at<cv::Vec2f>(0, i)[0];
        map_y_sparse.at<float>(i / GRID, i % GRID) = mapped.at<cv::Vec2f>(0, i)[1];
    }
    cv::Mat map_x, map_y;
    cv::resize(map_x_sparse, map_x, cv::Size(WARP_SIZE, WARP_SIZE), 0, 0, cv::INTER_LINEAR);
    cv::resize(map_y_sparse, map_y, cv::Size(WARP_SIZE, WARP_SIZE), 0, 0, cv::INTER_LINEAR);

    // Remap original image → corrected 700x700
    cv::Mat corrected;
    cv::remap(image, corrected, map_x, map_y, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0));

    // Convert to grayscale if needed
    if (corrected.channels() == 3)
        cv::cvtColor(corrected, corrected, cv::COLOR_BGR2GRAY);

    return corrected;
}

std::string get_contour_bits(const cv::Mat& image, 
                             const std::vector<cv::Point2f>& contour,
                             int bits,
                             const std::string& keypoints_file) {
    // Ensure contour has exactly 4 points
    if (contour.size() != 4) {
        throw std::runtime_error("Contour must have exactly 4 points");
    }
    
    std::vector<cv::Point2f> ordered_contour = contour;

    // Adaptive warp target size based on source quad area
    // Prevents extreme upsampling noise when source is small
    double src_area = cv::contourArea(ordered_contour);
    int warp_size = bits;
    if (src_area < 2500.0) {
        warp_size = std::max(100, std::min(bits, static_cast<int>(std::sqrt(src_area) * 5)));
    }

    // Define corners of output normalized image
    std::vector<cv::Point2f> corners = {
        cv::Point2f(0, 0),
        cv::Point2f(warp_size, 0),
        cv::Point2f(warp_size, warp_size),
        cv::Point2f(0, warp_size)
    };

    // Get perspective transform matrix
    cv::Mat M = cv::getPerspectiveTransform(ordered_contour, corners);

    // Warp perspective to normalize marker
    cv::Mat warped;
    cv::warpPerspective(image, warped, M, cv::Size(warp_size, warp_size), cv::INTER_CUBIC);

    // Convert to grayscale
    cv::Mat gray;
    if (warped.channels() == 3) {
        cv::cvtColor(warped, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = warped.clone();
    }

    // Otsu thresholding — must match the method used to generate dictionary signatures
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
    // Scale keypoints from 700-space to actual warp_size
    float kp_scale = static_cast<float>(warp_size) / 700.0f;
    std::string signature = get_id(binary, keypoints_file, kp_scale);

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

        // Track the best rotation from standard attempts for TPS fallback
        int fallback_best_rot = 0;
        int fallback_best_dist = 999;

        // Try all 4 cyclic rotations of the quad corners.
        for (int rot = 0; rot < 4; ++rot) {
            std::vector<cv::Point2f> rotated_candidate(4);
            for (int k = 0; k < 4; ++k) {
                rotated_candidate[k] = candidate[(k + rot) % 4];
            }

            std::string detected_sig;
            try {
                detected_sig = get_contour_bits(image, rotated_candidate, 700, "data/keypoints.txt");
            } catch (const std::exception& e) {
                std::cout << "  Candidate " << cant_num << " rot=" << rot
                          << " EXCEPTION: " << e.what() << std::endl;
                continue;
            }

            // Match against dictionary
            int best_distance = 999;
            int best_marker_id = -1;
            std::string best_orient;
            const MarkerOrientation* best_orient_data = nullptr;

            for (const auto& dict_entry : tag_dictionary) {
                int marker_id = dict_entry.first;
                const MarkerData& marker_data = dict_entry.second;

                for (const auto& orientation_entry : marker_data.orientations) {
                    const MarkerOrientation& orientation_data = orientation_entry.second;

                    int distance = hamming_distance(detected_sig, orientation_data.signature);

                    if (distance < best_distance) {
                        best_distance = distance;
                        best_marker_id = marker_id;
                        best_orient = orientation_entry.first;
                        best_orient_data = &orientation_data;
                    }
                }
            }

            // Debug: show best match per rotation attempt
            std::cout << "  Candidate " << cant_num << " rot=" << rot
                      << " best_dist=" << best_distance
                      << " marker=" << best_marker_id
                      << " orient=" << best_orient << std::endl;

            if (best_distance <= allowed_misses && best_orient_data != nullptr) {
                result.corners.push_back(candidate);
                result.indices.push_back(best_marker_id);
                result.world_locs.push_back(best_orient_data->world_points);
                result.count++;
                goto next_candidate;
            }

            // Track best rotation for potential TPS fallback
            if (best_distance < fallback_best_dist) {
                fallback_best_dist = best_distance;
                fallback_best_rot = rot;
            }
        }

        // Standard warp failed for all rotations — try curved surface fallback.
        // Only attempt on quads large enough to have visible inner borders.
        {
            double src_area = cv::contourArea(candidate);
            std::cout << "  Candidate " << cant_num
                      << " fallback check: area=" << src_area
                      << " best_rot=" << fallback_best_rot
                      << " best_dist=" << fallback_best_dist << std::endl;
            if (src_area >= 3000.0) {
                // Use the rotation that gave the best (lowest) Hamming distance
                std::vector<cv::Point2f> best_corners(4);
                for (int k = 0; k < 4; ++k)
                    best_corners[k] = candidate[(k + fallback_best_rot) % 4];

                // Compute initial homography and binary warp
                std::vector<cv::Point2f> dst_corners = {
                    {0, 0}, {700, 0}, {700, 700}, {0, 700}
                };
                cv::Mat M = cv::getPerspectiveTransform(best_corners, dst_corners);
                cv::Mat warped;
                cv::warpPerspective(image, warped, M, cv::Size(700, 700), cv::INTER_CUBIC);
                cv::Mat gray_w;
                if (warped.channels() == 3)
                    cv::cvtColor(warped, gray_w, cv::COLOR_BGR2GRAY);
                else
                    gray_w = warped;
                cv::Mat binary_w;
                cv::threshold(gray_w, binary_w, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

                // Attempt TPS re-warp
                cv::Mat corrected = attempt_curved_rewarp(image, best_corners, M, binary_w);
                if (!corrected.empty()) {
                    // Threshold corrected warp and extract signature
                    cv::Mat binary_c;
                    cv::threshold(corrected, binary_c, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
                    std::string curved_sig = get_id(binary_c, "data/keypoints.txt", 1.0f);

                    // Match against dictionary
                    int best_distance = 999;
                    int best_marker_id = -1;
                    std::string best_orient;
                    const MarkerOrientation* best_orient_data = nullptr;

                    for (const auto& dict_entry : tag_dictionary) {
                        int marker_id = dict_entry.first;
                        const MarkerData& marker_data = dict_entry.second;
                        for (const auto& oe : marker_data.orientations) {
                            int dist = hamming_distance(curved_sig, oe.second.signature);
                            if (dist < best_distance) {
                                best_distance = dist;
                                best_marker_id = marker_id;
                                best_orient = oe.first;
                                best_orient_data = &oe.second;
                            }
                        }
                    }

                    std::cout << "  Candidate " << cant_num
                              << " curved_rewarp best_dist=" << best_distance
                              << " marker=" << best_marker_id
                              << " orient=" << best_orient << std::endl;

                    if (best_distance <= allowed_misses && best_orient_data != nullptr) {
                        result.corners.push_back(candidate);
                        result.indices.push_back(best_marker_id);
                        result.world_locs.push_back(best_orient_data->world_points);
                        result.count++;
                    }
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
