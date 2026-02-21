// Copyright (c) Astradyn Systems LLP.
// SPDX-License-Identifier: Apache-2.0

#include "quadrilateral.hpp"
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
	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

	// Apply CLAHE
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
	cv::Mat enhanced;
	clahe->apply(gray, enhanced);

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

std::vector<cv::Vec4i> QuadrilateralDetector::detectLines(const cv::Mat& contourImg)
{
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(contourImg, lines, 1, CV_PI/180, 15, 5, 10);
	return lines;
}

std::vector<CartesianLine> QuadrilateralDetector::convertLinesToCartesian(const std::vector<cv::Vec4i>& lines)
{
	std::vector<CartesianLine> cartesianLines;

	// Consider upto eight lines
	size_t maxLines = std::min(lines.size(), size_t(16));
        
	for (size_t i = 0; i < maxLines; ++i)
	{
		CartesianLine line;
		line.p1 = cv::Point2f(lines[i][0], lines[i][1]);
		line.p2 = cv::Point2f(lines[i][2], lines[i][3]);
		cartesianLines.push_back(line);
	}

	return cartesianLines;
}

std::vector<std::pair<int,int>> QuadrilateralDetector::findParallelPairs(const std::vector<CartesianLine>& cartesianLines)
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

			if (angleDiff < 0.3 || std::abs(angleDiff - CV_PI) < 0.3)
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

	// Order corners as: top-left, top-right, bottom-right, bottom-left
	// Top-left has smallest sum (x+y), bottom-right has largest sum
	// Top-right has smallest difference (y-x), bottom-left has largest difference
	std::vector<cv::Point2f> ordered(4);

	std::vector<float> sums(4), diffs(4);
	for (int i = 0; i < 4; ++i)
	{
		sums[i] = contour[i].x + contour[i].y;
		diffs[i] = contour[i].y - contour[i].x;
	}

	auto min_sum = std::min_element(sums.begin(), sums.end());
	auto max_sum = std::max_element(sums.begin(), sums.end());
	auto min_diff = std::min_element(diffs.begin(), diffs.end());
	auto max_diff = std::max_element(diffs.begin(), diffs.end());

	ordered[0] = contour[std::distance(sums.begin(), min_sum)];   // top-left
	ordered[1] = contour[std::distance(diffs.begin(), min_diff)]; // top-right
	ordered[2] = contour[std::distance(sums.begin(), max_sum)];   // bottom-right
	ordered[3] = contour[std::distance(diffs.begin(), max_diff)]; // bottom-left

	return ordered;
}

std::vector<std::vector<cv::Point2f>> QuadrilateralDetector::detectQuadrilaterals(const cv::Mat& image)
{
	cv::Mat gray   = preprocessImage(image);
	cv::Mat binary = applyAdaptiveThreshold(gray);

	auto contours = findContours(binary);
	std::vector<std::vector<cv::Point2f>> detectedQuadrilaterals;

	// Diagnostic counters
	int cnt_total = contours.size();
	int cnt_area = 0, cnt_lines = 0, cnt_parallel = 0, cnt_quad = 0, cnt_aspect = 0, cnt_passed = 0;

	for (const auto& contour : contours)
	{
		if (cv::contourArea(contour) < 20)
		{
			continue;
		}
		cnt_area++;

		cv::Mat contourImg = cv::Mat::zeros(binary.size(), binary.type());
		cv::drawContours(contourImg, std::vector<std::vector<cv::Point>>{contour}, -1, 255, 1);

		auto lines = detectLines(contourImg);

		if (!lines.empty() && lines.size() >= 4)
		{
			cnt_lines++;
			auto cartesianLines = convertLinesToCartesian(lines);
			auto parallelPairs  = findParallelPairs(cartesianLines);

			if (parallelPairs.size() >= 1)
			{
				cnt_parallel++;
				double epsilon = 0.09 * cv::arcLength(contour, true);
				std::vector<cv::Point> approx;
				cv::approxPolyDP(contour, approx, epsilon, true);

				if (approx.size() == 4)
				{
					cnt_quad++;
					cv::RotatedRect rect = cv::minAreaRect(approx);
					float width  = rect.size.width;
					float height = rect.size.height;
					float aspectRatio = std::max(width, height) / std::min(width, height);

					if (aspectRatio >= 1.0 && aspectRatio < 5.0)
					{
						cnt_aspect++;
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
							cnt_passed++;
						}
						catch (const std::invalid_argument& e)
						{
							// Use original corners if ordering fails
							detectedQuadrilaterals.push_back(corners);
							cnt_passed++;
						}

					}
				}
			}
		}
	}

	std::cout << "Pipeline: " << cnt_total << " contours -> "
		  << cnt_area << " area -> " << cnt_lines << " lines -> "
		  << cnt_parallel << " parallel -> " << cnt_quad << " quad -> "
		  << cnt_aspect << " aspect -> " << cnt_passed << " passed" << std::endl;

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

		if (!outputPath.empty())
		{
			cv::imwrite(outputPath, resultImage);
			std::cout<<"Results saved to "<<outputPath<<std::endl;
		}
		return !quads.empty();

	}

	catch (const std::exception& e)
	{
		std::cerr<<"Error processing image: "<<e.what()<<std::endl;
		return false;
	}
}


void processImageBatch(const std::string& inputFolder, const std::string& outputFolder)
{
	if(!fs::exists(outputFolder))
	{
		fs::create_directories(outputFolder);
	}

	int successfulDetections = 0;
	int totalImages          = 0;

	for (const auto& entry : fs::directory_iterator(inputFolder))
	{
		if (entry.is_regular_file())
		{
			std::string extension = entry.path().extension().string();
			std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
			if (extension == ".png" || extension == ".jpg" || extension == ".jpeg")
			{
				totalImages++;
				std::string inputPath = entry.path().string();
				std::string outputPath= fs::path(outputFolder)/("output_" + entry.path().filename().string());

				if (QuadrilateralDetector::drawResults(inputPath, outputPath))
				{
					successfulDetections++;
				}
			}
		}
	}

	std::cout<<"\nSummary: Found quadrilaterals in "<<successfulDetections
		<<" out of "<< totalImages<< " images" <<std::endl;
}
