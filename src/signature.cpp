#include "signature.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

#ifdef ASTRATAG_USE_EMBEDDED_DATA
#include "embedded_data.hpp"
#endif

namespace astratag {

std::vector<std::vector<cv::Point2f>> load_keypoints(const std::string& filename, float scale) {
    std::vector<std::vector<cv::Point2f>> triangles;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Could not open keypoints file: " + filename);
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x1, y1, x2, y2, x3, y3, cx, cy;
        
        if (!(iss >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> cx >> cy)) {
            continue; // Skip malformed lines
        }
        
        // Create triangle with 3 vertices (scaled)
        std::vector<cv::Point2f> triangle;
        triangle.push_back(cv::Point2f(x1 * scale, y1 * scale));
        triangle.push_back(cv::Point2f(x2 * scale, y2 * scale));
        triangle.push_back(cv::Point2f(x3 * scale, y3 * scale));
        
        triangles.push_back(triangle);
    }
    
    file.close();
    return triangles;
}

std::vector<std::vector<cv::Point2f>> load_keypoints_from_string(const std::string& data, float scale) {
    std::vector<std::vector<cv::Point2f>> triangles;
    
    std::istringstream stream(data);
    std::string line;
    
    while (std::getline(stream, line)) {
        std::istringstream line_stream(line);
        std::string token;
        std::vector<float> values;
        
        // Parse comma-separated values
        while (std::getline(line_stream, token, ',')) {
            try {
                values.push_back(std::stof(token));
            } catch (...) {
                // Skip non-numeric tokens (like "triangle_0")
            }
        }
        
        // Extract the 6 coordinates: x1, y1, x2, y2, x3, y3
        if (values.size() >= 6) {
            std::vector<cv::Point2f> triangle;
            triangle.push_back(cv::Point2f(values[0] * scale, values[1] * scale));
            triangle.push_back(cv::Point2f(values[2] * scale, values[3] * scale));
            triangle.push_back(cv::Point2f(values[4] * scale, values[5] * scale));
            triangles.push_back(triangle);
        }
    }
    
    return triangles;
}

std::vector<std::vector<cv::Point2f>> load_embedded_keypoints(float scale) {
#ifdef ASTRATAG_USE_EMBEDDED_DATA
    return load_keypoints_from_string(embedded::KEYPOINTS_TXT, scale);
#else
    (void)scale; // Suppress unused parameter warning
    throw std::runtime_error("Embedded data not available. Rebuild with -DASTRATAG_EMBED_DATA=ON");
#endif
}

std::string get_id(const cv::Mat& binary_image, const std::string& keypoints_file) {
    // Load keypoints from file
    std::vector<std::vector<cv::Point2f>> triangles = load_keypoints(keypoints_file, 1.0);
    
    std::string signature;
    signature.reserve(triangles.size()); // Pre-allocate space
    
    // Process each triangle region
    for (const auto& triangle : triangles) {
        // Convert Point2f to Point (integer coordinates) for fillPoly
        std::vector<cv::Point> triangle_int;
        for (const auto& pt : triangle) {
            triangle_int.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
        }
        
        // Create mask for this triangle
        cv::Mat mask = cv::Mat::zeros(binary_image.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> contours = { triangle_int };
        cv::fillPoly(mask, contours, cv::Scalar(255));
        
        // Calculate mean value in the masked region
        cv::Scalar mean_val = cv::mean(binary_image, mask);
        
        // Decode bit: high mean value (white) = '1', low mean value (black) = '0'
        // Threshold at 127 (midpoint between 0 and 255)
        char bit = (mean_val[0] > 127) ? '1' : '0';
        signature += bit;
    }
    
    return signature;
}

int hamming_distance(const std::string& sig1, const std::string& sig2) {
    if (sig1.length() != sig2.length()) {
        throw std::invalid_argument("Signature strings must have equal length");
    }
    
    int distance = 0;
    for (size_t i = 0; i < sig1.length(); ++i) {
        if (sig1[i] != sig2[i]) {
            distance++;
        }
    }
    
    return distance;
}

} // namespace astratag
