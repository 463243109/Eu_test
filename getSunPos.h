#ifndef _GET_SUN_POS_H_
#define _GET_SUN_POS_H_
#include <opencv.hpp>
/***��̫������ ���̲ο�����Camera Calibration and Light Source Estimation from Images with Shadows.�����ʵ��޸�**/
void getSunPos(const int H, const int W, const cv::Point2f data_point[11],cv::Point2d &sunPos, double &theta, double &phi);

/**��theta, phi*/
void get_theta_and_phi(const cv::Mat w, const cv::Point2d sunPos, const cv::Point2d v_prime, \
                       const cv::Point2d vx, const cv::Point2d vy, const cv::Point2d vz,  double &theta, double &phi);

#endif // _GET_SUN_POS_H_
