#include "getVanishPoint.h"

cv::Point2f getVanishPoint(cv::Point2f v0, cv::Point2f v1, cv::Point2f v2, cv::Point2f v3)
{
    cv::Point2f vp;
    vp = getLineIntersection(v0, v1, v2, v3);
    return vp;
}
