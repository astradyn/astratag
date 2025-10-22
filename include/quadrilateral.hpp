#ifndef QUADRILATERAL_HPP
#define QUADRILATERAL_HPP

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

struct CartesianLine{
    cv::Point2f p1;
    cv::Point2f p2;
};

class QuadrilateralDetector{

	public:
		QuadrilateralDetector();

		// Main detection funtion 
		std::vector<std::vector<cv::Point2f>> detectQuadrilaterals(const cv::Mat& image);

		// Utility function
		static cv::Mat readImage(const std::string& imagePath);
		static bool drawResults(const std::string& inputPath, const std::string& outputPath);

	private:
		cv::Mat preprocessImage(const cv::Mat& image);
		cv::Mat applyAdaptiveThreshold(const cv::Mat& gradientMagnitude);
		std::vector<std::vector<cv::Point>> findContours(const cv::Mat& binary);
		std::vector<cv::Vec4i> detectLines(const cv::Mat& contourImg);
		std::vector<CartesianLine> convertLinesToCartesian(const std::vector<cv::Vec4i>& lines);
		std::vector<std::pair<int, int>> findParallelPairs(const std::vector<CartesianLine>& cartesianLines);
		std::vector<cv::Point2f> orderContour(const std::vector<cv::Point2f>& contour);
		cv::TermCriteria cornerCriteria;
};


// Batch processing function
void processImageBatch(const std::string& inputFolder, const std::string& outputFolder);

#endif // QUADRILATERAL_HPP