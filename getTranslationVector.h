#ifndef _GET_TRANSLATION_VECTOR_H
#define _GET_TRANSLATION_VECTOR_H
#include "public.h"

#define SCALE 100

bool getTranslationVector(const cv::Point2f origin, const cv::Point2f v1, const cv::Mat R, const double focal_length, cv::Mat &T);

#endif // _GET_TRANSLATION_VECTOR_H
