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


