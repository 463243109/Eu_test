#include "getTranslationVector.h"

bool getTranslationVector(const cv::Point2f origin, const cv::Point2f v1, const cv::Mat R, const double focal_length, cv::Mat &T)
{

    if (fabs((double)(origin.x - v1.x))<20 && fabs((double)(origin.y - v1.y))<20)
        return false;

    /**x-axis is a line from origin to v1 in image space*/
    cv::Mat line_x_axis = cv::Mat_<double>(4,1);
    line_x_axis.at<double>(0,0) = origin.x;
    line_x_axis.at<double>(1,0) = origin.y;
    line_x_axis.at<double>(2,0) = v1.x;
    line_x_axis.at<double>(3,0) = v1.y;

    /**direction of x-axis */
    cv::Mat dir_x_axis = cv::Mat_<double>(2,1);
    dir_x_axis.at<double>(0,0) = v1.x - origin.x;
    dir_x_axis.at<double>(1,0) = v1.y - origin.y;
    cv::Mat length = cv::Mat_<double>(1,1);
    length = dir_x_axis.t()*dir_x_axis;
    dir_x_axis.at<double>(0,0) /= sqrt(length.at<double>(0,0));
    dir_x_axis.at<double>(1,0) /= sqrt(length.at<double>(0,0));

    /**p为x轴上一点在图像上的投影坐标,其在世界坐标系内的坐标为（scale*d,0,0）**/
    cv::Point2f p;
    /**p点同原点在图像上的距离**/
    int d=20;
    int scale = d * SCALE;

    p.x = (int)(origin.x + d*dir_x_axis.at<double>(0,0) + 0.5);
    p.y = (int)(origin.y + d*dir_x_axis.at<double>(1,0) + 0.5);

    /**translation vector*/
	if (fabs((double)(p.x - origin.x)) > fabs((double)(p.y - origin.y)))
    {
        T.at<double>(2,0) = scale  * (R.at<double>(0,0)*focal_length - R.at<double>(2,0)*p.x)/(double)(p.x - origin.x);
    }
    else
    {
        T.at<double>(2,0) = scale  * (R.at<double>(1,0)*focal_length - R.at<double>(2,0)*p.y)/(double)(p.y - origin.y);
    }

    if(T.at<double>(2,0) < 0)
    {
        T.at<double>(2,0) =  -1 * T.at<double>(2,0);
    }
    if(T.at<double>(2,0) < focal_length)
    {
        return false;
    }
    T.at<double>(0,0) = origin.x * T.at<double>(2,0) / focal_length;
    T.at<double>(1,0) = origin.y * T.at<double>(2,0) / focal_length;
    return true;
}
