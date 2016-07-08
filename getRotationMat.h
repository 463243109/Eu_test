#ifndef _GET_ROTATION_MAT_H
#define _GET_ROTATION_MAT_H
#include "public.h"

void getRotateMat(cv::Point2f vx, cv::Point2f vy, double focal_length, cv::Mat &R);

void adjustRotateMat(cv::Mat srcR, cv::Mat &dstR);

#endif // _GET_ROTATION_MAT_H
