#ifndef _CALIBRATE_H
#define _CALIBRATE_H
#include "public.h"
#include "getFocalLength.h"
#include "getRotationMat.h"
#include "getVanishPoint.h"
#include "getTranslationVector.h"

void Calibrate(cv::Mat frame, cv::Point2f data_point[11], double &focal_length, cv::Mat &R, cv::Mat &T);

#endif // _CALIBRATE_H
