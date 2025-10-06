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

	for (size_t i=0; i<contours.size(); ++i)
	{
		const cv::Vec4i& h = hierarchy[i];

		// h[2] is first child, h[3] is parent
		if (h[2] != -1)
		{
			// If contour has children
			// Add all child contours
			int childIdx = h[2];
			while (childIdx != -1)
			{
				filteredContours.push_back(contours[childIdx]);
				childIdx = hierarchy[childIdx][0]; // get next sibling
			}
		}
		else if (h[3] == -1)
		{
			// If contour has no parent (is outer)
			filteredContours.push_back(contours[i]);
		}
	}

	return filteredContours;
}

std::vector<cv::Vec4i> QuadrilateralDetector::detectLines(const cv::Mat& contour Img)
{
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(contourImg, lines, 1, CV_PI/180, 20, 10, 5);
	return lines;
}

std::vector<CartesianLine> QuadrilateralDetector::convertLinesToCartesian(const std::vector<cv::Vec4i>& lines);
{
	std::vector<CartesianLine> cartesianLines;

	// Consider upto eight lines
	size_t maxLines = std::min(lines.size(), size_t(8));
        
	for (size_t i = 0, i<maxLines; ++i)
	{
		CartesianLine line;
		line.p1 = cv::Point2f(lines[i][0], lines[i][1]);
		line.p2 = cv::Point2f(lines[i][2], lines[i][3]);
		cartesianLines.push_back(line);
	}

	return cartesianLines;
}

std::vector<std::pair<int,int>> QuadrilateralDetector::findParallelPairs(const std::vector<CartesianLins>& cartesianLines)
{
	std::vector<std::pair<int, int>> parallelPairs;
	for (size_t i= 0; i<cartesianLines.size(); ++i)
	{
		for (size_t j=i+1; j<cartesianLines.size(); ++j)
		{
			double angle1 = std::atan2(cartesianLines[i].p2.y - cartesianLines[i].p1.y,
					cartesianLines[i].p2.x - cartesianLines[i].p1.x);

			double angle2 = std::atan2(cartesianLines[j].p2.y - cartesianLines[j].p1.y,
					cartesianLines[j].p2.x - cartesianLines[j].p1.x);

			double angleDiff = std::abs(angle1 - angle2);

			if (angleDiff < 0.1 || std::abs(angleDiff - CV_PI) < 0.1)
			{
				parallelPairs.emplace_back(i,j);
			}
		}
	}

	return parallelPairs;
}

std::vector<cv::Point2f> QuadrilateralDetector::orderContour(const std::vector<cv::Point2f>& contour)
{
	if (contour.size() != 4)
	{
		throw std::invalid_argument("Contour must have exactly 4 points");
	}
	std::vector<cv::Point2f> ordered = contour;

	// Calculate center
	float cx = 0, cy = 0;
	for (const auto& point : ordered)
	{
		cx += point.x;
		cy += point.y;
	}
	cx /= 4.0f;
	cy /= 4.0f;

	// If first corner is top-left, swap diagonal
	if (ordered[0].x <= cx && ordered[0].y <= cy)
	{
		std::swap(ordered[1], ordered[3]);
	}
	else
	{
		std::swap(ordered[0], ordered[1]);
		std::swap(ordered[2], ordered[3]);
	}

	return ordered;
}

std::vector<std::vector<cv::Point2f>> QuadrilateralDetector::detectQuadrilaterals(const cv::Mat& image)
{
	cv::Mat output = image.clone();
	cv::Mat gray   = preprocessImage(image);
	cv::Mat binary = applyAdaptiveThreshold(gray);

	cv::imwrite("binary.png", binary);

	auto contours = findContours(binary);
	std::vector<std::vector<cv::Point2f>> detectQuadrilaterals;

	for (const auto& contour : contours)
	{
		if (cv::contourArea(contour) < 20)
		{
			continue;
		}

		cv::Mat contourImg = cv::Mat::zeros(binary.size(), binary.type());
		cv::drawContours(contourImg, std::vector<std::vector<cv::Point>>{contour}, -1, 255, 1);

		auto lines = detectLines(contourImg);

		if (!lines.empty() && lines.size() >= 4) 
		{
			auto cartesianLines = convertLinesToCartesian(lines);
			auto parallelPairs  = findParallelPairs(cartesianLines);

			if (parallelPairs.size() >= 2)
			{
				double epsilon = 0.09 * cv::arcLength(contour, true);
				std::vector<cv::Point> approx;
				cv::approxPolyDP(contour, approx, epsilon, true);

				if (approx.size() == 4)
				{
					cv::RotatedRect rect = cv::minAreaRect(approx);
					float width  = rect.size.width;
					float height = rect.size.height;
					float aspectRatio = std::max(width, height) / std::min(width, height);

					if (aspectRatio > 0.5 && aspectRatio < 2.0)
					{
						std::vector<cv::Point2f> corners(approx.begin(), approx.end());
						std::vector<cv::Point2f> refinedCorners;
						cv::Mat gray;
						cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
						cv::cornerSubPix(gray, corners, cv::Size(3,3), cv::Size(-1,1),
								cornerCriteria);

                                                try
						{
							auto orderedCorners = orderContour(corners); 
							detectedQuadrilaterals.push_back(orderedCorners);
						}
						catch (const std::invalid_argument& e)
						{
							// Use original corners if ordering fails
							detectedQuadrilaterals.push_back(corners);
						}
					
					}
				}
			}
		}
	}
	return detectedQuadrilaterals;
}

bool QuadrilateralDetector::drawResults(const std::string& inputPath, const std::string& outputPath)
{
	try
	{
		cv::Mat image = readImage(inputPath);
		QuadrilateralDetector detector;
		auto quads = detector.detectQuadrilaterals(image);

		std::cout<<"Found "<< quads.size() <<"quadrilaterals "<<std::endl;

		cv::Mat resultImage = image.clone();
		for (const auto& quad : quads)
		{
			std::vector<cv::Point> intQuad;
			for (const auto& point: quad)
			{
				intQuad.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
			}
			cv::polylines(resultImage, intQuad, true, cv::Scalar(0,255,0), 2);
		}
	}
}












