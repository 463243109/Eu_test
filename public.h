#ifndef _PUBLIC_H
#define _PUBLIC_H
#include <opencv2/opencv.hpp>

/**  ����ֱ�ߵĵĽ��� p0, p1�����һ��ֱ�ߡ� p2, p3����ڶ���ֱ��  ���ؽ���*/
 cv::Point2f getLineIntersection(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);

#endif // _PUBLIC_H
