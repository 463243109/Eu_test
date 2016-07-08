#include "public.h"

cv::Point2f getLineIntersection(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3)
{
    cv::Point2f vp;
    float x0, x1, x2, x3, y0, y1, y2, y3;
    x0 = p0.x;   y0 = p0.y;
    x1 = p1.x;   y1 = p1.y;
    x2 = p2.x;   y2 = p2.y;
    x3 = p3.x;   y3 = p3.y;
    vp.y = ( (y0-y1)*(y3-y2)*x0 + (y3-y2)*(x1-x0)*y0 + (y1-y0)*(y3-y2)*x2 + (x2-x3)*(y1-y0)*y2 ) / ( (x1-x0)*(y3-y2) + (y0-y1)*(x3-x2) );
    vp.x = x2 + (x3-x2)*(vp.y-y2) / (y3-y2);
    return vp;
}
