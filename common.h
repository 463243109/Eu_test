#ifndef _COMMON_H
#define _COMMON_H

#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv.hpp>

#define SCALE 100
#define LINE_POINT_NUM 18

static const float PI = 3.1415926;

// distance of two points
float distance(cv::Point2f pt1, cv::Point2f pt2);
//
cv::Point2f  point_on_line(cv::Point2f pt, cv::Point2f line_start, cv::Point2f line_end);

/**求焦距 返回焦距**/
float get_focal_length(cv::Point2f v1, cv::Point2f v2, const int& width, const int& height);

/**  求两直线的的焦点 p0, p1代表第一条直线。 p2, p3代表第二条直线  返回交点*/
 cv::Point2f get_line_intersection(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3);

double get_focal_length2(cv::Point2f v1, cv::Point2f v2);
//


// line is composed of four float: (x1, y1; x2, y2)
cv::Point2f	get_vanishing_point(const float* line1, const float* line2);
// light tracking
void tracking(cv::Mat & frame, cv::Point2f point_data[11]);
// get mid points when out of range
bool half_line(int out_point_num, cv::Point2f pre_point[11], cv::Point2f now_point[11]);

int getCamPara(int W, int H, const double* pointVec, double& phy, double &theta, double &xxx, double &yyy);

int draw_coordinate_sys(cv::Point2f point_data[11], cv::Mat &frame, float x, float y);

// point_data should have 11 elements
int sunPositionProcess(int init_num, std::string fileName, cv::Point2f point_data[]);


int draw_coordinate_sys(cv::Point2f point_data[11], cv::Mat &frame, float x, float y);

void get_vp_for_R(cv::Mat frame, const float *pointVec, cv::Point2f &v1, cv::Point2f &v2, double &f);

void get_rotation_matrix(const cv::Point2f v1, const cv::Point2f v2, const double f, const cv::Point3f Zc, cv::Mat &R);

void get_rotation_mat(const cv::Point2f v1, const cv::Point2f v2, const double f, cv::Mat &R);

/**parameter:
  * origin     :input, the original point in image space
  * v1         :input, vanishing point for x-axis
  * R          :input, Rotation matrix
  *focal_length:input,focal length of camera
  * T          :output, the translation vector
*/
bool get_translation_vector(const cv::Point2f origin, const cv::Point2f v1, const cv::Mat R, const double focal_length, double T[]);

bool adjust_rotation_matrix(cv::Mat src_R, cv::Mat &dst_R);

void para_to_decare(const double r, const double phi, const double theta, double &x, double &y, double &z);

void switch_to_cam_coordinate(const cv::Mat _Pw, const cv::Mat _R, const cv::Mat _t, cv::Mat &_Pc, cv::Mat &_M);

void draw_line_2(cv::Mat & temp , cv::Point2f data[11] ,int out_point_mark[11] , cv::Point2f line_point[4][LINE_POINT_NUM] , int out_line_mark[4][LINE_POINT_NUM]);

bool reAllocation(const cv::Mat frame, cv::Point2f data[11] ,int out_point_mark[11] , cv::Point2f line_point[4][LINE_POINT_NUM] , int out_line_mark[4][LINE_POINT_NUM]);//点出界时重新划分数据点

void extend(cv::Point2f point_data[11] ) ; //延长数据线
/**中心对称 将src 以origin 为中心作中心对称点**/
void central_symmetry( cv::Point2f src, cv::Point2f origin, cv::Point2f& dst );
/**从旋转矩阵求x-axis 和 y-axis 方便画坐标轴**/
void xy_axis(cv::Mat R, cv::Point2f& x_axis, cv::Point2f& y_axis);

#endif // _COMMON_H
