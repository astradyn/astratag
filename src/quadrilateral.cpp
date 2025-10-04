#include <"quadrilateral.hpp">
#include <iostream>
#include <cmath>
#include <algorithm>

QuadrilateralDetector::QuadrilateralDetector()
{
	cornerCriteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
}

cv::Mat QuadrilateralDetector::readImage(const std::string& imagePath)
{
	cv::Mat image = cv::imread(imagePath);
	if (image.empty())
	{
		throw std::runtime_error("Could not read image: "+ imagePath);
	}
	return image;
}

cv::Mat QuadrilateralDetector::preprocessImage(const cv::Mat& image)
{
	cv::Mat gray;
	cv::cvtColor(image, gray, cv::COLORBGR2GRAY);

	// Apply CLAHE
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
	cv::Mat enhanced;
	clahe->apply(gray, enhanced);

	cv::imwrite("enhanced.png", enhanced);
	return enhanced;
}

cv::Mat QuadrilateralDetector::applyAdaptiveThreshold(const cv::Mat& gradientMagnitude)
{
	cv::Mat binary;
	cv::adaptiveThreshold(gradientMagnitude, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
	return binary;
}

std::vector<std::vector<cv::Point>> QuadrilateralDetector::findContours(const cv::Mat& binary)
{
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(binary, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>> filteredContours;

}
