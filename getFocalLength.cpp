#include "getFocalLength.h"

double getFocalLength(cv::Point2f vx, cv::Point2f vy)
{
    cv::Point2f v1 = vx;
    cv::Point2f v2 = vy;

    return sqrt( -1*(v1.x * v2.x + v1.y * v2.y) );
}
