#ifndef _PUBLIC_H
#define _PUBLIC_H
#include <opencv2/opencv.hpp>

/**  求两直线的的焦点 p0, p1代表第一条直线。 p2, p3代表第二条直线  返回交点*/
 cv::Point2f getLineIntersection(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);

#endif // _PUBLIC_H
