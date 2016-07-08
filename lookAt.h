#ifndef _LOOK_AT_H
#define _LOOK_AT_H

#include <opencv2/opencv.hpp>

void lookAtParam(const cv::Mat R, const cv::Mat T, cv::Mat& eye, cv::Mat& center, cv::Mat & up);

#endif // _LOOK_AT_H
