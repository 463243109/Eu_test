#include "calibrate.h"
#include "public.h"

void Calibrate(cv::Mat frame, cv::Point2f data_point[11], double &focal_length, cv::Mat &R, cv::Mat &T)
{
    assert(R.rows == 3 && R.cols == 3);
    assert(T.rows == 3 && T.cols == 1);
    cv::Point2f point[11];
    cv::Point2f principal_point;
    principal_point.x = (double)frame.cols/2.0;
    principal_point.y = (double)frame.rows/2.0;

    for(int i=0; i<11; i++)
    {
        point[i] = data_point[i]-principal_point;
    }
    cv::Point2f origin = point[10];

    cv::Point2f v1 = getVanishPoint(point[0], point[1], point[2], point[3]);
    cv::Point2f v2 = getVanishPoint(point[4], point[5], point[6], point[7]);


    focal_length = getFocalLength(v1, v2);
    getRotateMat(v1, v2, focal_length, R);
    getTranslationVector(origin,v1, R, focal_length, T);
}
