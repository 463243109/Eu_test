#include "getRotationMat.h"

void getRotateMat(cv::Point2f vx, cv::Point2f vy, double focal_length, cv::Mat &R)
{
    cv::Mat r1 = cv::Mat_<double>(3,1);
    cv::Mat r2 = cv::Mat_<double>(3,1);
    cv::Mat r3 = cv::Mat_<double>(3,1);

    r1.at<double>(0,0) = vx.x;
    r1.at<double>(1,0) = vx.y;
    r1.at<double>(2,0) = focal_length;
    cv::normalize(r1,r1);

    r2.at<double>(0,0) = vy.x;
    r2.at<double>(1,0) = vy.y;
    r2.at<double>(2,0) = focal_length;
    cv::normalize(r2,r2);

    r3 = r1.cross(r2);
    R = cv::Mat_<double>(3,3);
    R.at<double>(0,0) = r1.at<double>(0,0);
    R.at<double>(1,0) = r1.at<double>(1,0);
    R.at<double>(2,0) = r1.at<double>(2,0);

    R.at<double>(0,1) = r2.at<double>(0,0);
    R.at<double>(1,1) = r2.at<double>(1,0);
    R.at<double>(2,1) = r2.at<double>(2,0);

    R.at<double>(0,2) = r3.at<double>(0,0);
    R.at<double>(1,2) = r3.at<double>(1,0);
    R.at<double>(2,2) = r3.at<double>(2,0);
    adjustRotateMat(R,R);
}

void adjustRotateMat(cv::Mat srcR, cv::Mat &dstR)
{
    cv::Mat x_axis = cv::Mat_<double>(3,1);
    cv::Mat y_axis = cv::Mat_<double>(3,1);
    cv::Mat z_axis = cv::Mat_<double>(3,1);
    x_axis.at<double>(0,0) = srcR.at<double>(0,0);
    x_axis.at<double>(1,0) = srcR.at<double>(1,0);
    x_axis.at<double>(2,0) = srcR.at<double>(2,0);

    y_axis.at<double>(0,0) = srcR.at<double>(0,1);
    y_axis.at<double>(1,0) = srcR.at<double>(1,1);
    y_axis.at<double>(2,0) = srcR.at<double>(2,1);

    z_axis.at<double>(0,0) = srcR.at<double>(0,2);
    z_axis.at<double>(1,0) = srcR.at<double>(1,2);
    z_axis.at<double>(2,0) = srcR.at<double>(2,2);

    int biggest_index=1;
    double biggest_val = fabs((double)x_axis.at<double>(1,0));
	if (fabs((double)y_axis.at<double>(1, 0)) > biggest_val)
    {
        biggest_index = 2;
		biggest_val = fabs((double)y_axis.at<double>(1, 0));
    }
	if (fabs((double)z_axis.at<double>(1, 0)) > biggest_val)
    {
        biggest_index = 3;
		biggest_val = fabs((double)z_axis.at<double>(1, 0));
    }
    int symble;
    cv::Mat v1 = cv::Mat_<double>(3,1);
    cv::Mat v2 = cv::Mat_<double>(3,1);
    cv::Mat v3 = cv::Mat_<double>(3,1);
    switch(biggest_index)
    {
    case 1:
        if(x_axis.at<double>(1,0) > 0) /**ÊúÖ±ÏòÏÂ*/
            symble = -1;
        else
        {
            symble = 1;
        }
        z_axis.at<double>(0,0) = symble * x_axis.at<double>(0,0);
        z_axis.at<double>(1,0) = symble * x_axis.at<double>(1,0);
        z_axis.at<double>(2,0) = symble * x_axis.at<double>(2,0);
        v1.at<double>(0,0) = y_axis.at<double>(0,0);
        v1.at<double>(1,0) = y_axis.at<double>(1,0);
        v1.at<double>(2,0) = y_axis.at<double>(2,0);

        v2.at<double>(0,0) = z_axis.at<double>(0,0);
        v2.at<double>(1,0) = z_axis.at<double>(1,0);
        v2.at<double>(2,0) = z_axis.at<double>(2,0);
        /**cross pruduct v1 * v2 */
        v3 = v1.cross(v2);
        //v3 = v2.cross(v1);
        x_axis = v3;
        break;
    case 2:
        if(y_axis.at<double>(1,0) > 0)
        {
            symble = -1;
        }
        else
        {
            symble = 1;
        }
        z_axis.at<double>(0,0) = symble * y_axis.at<double>(0,0);
        z_axis.at<double>(1,0) = symble * y_axis.at<double>(1,0);
        z_axis.at<double>(1,0) = symble * y_axis.at<double>(2,0);
        v1.at<double>(0,0) = z_axis.at<double>(0,0);
        v1.at<double>(1,0) = z_axis.at<double>(1,0);
        v1.at<double>(2,0) = z_axis.at<double>(2,0);

        v2.at<double>(0,0) = x_axis.at<double>(0,0);
        v2.at<double>(1,0) = x_axis.at<double>(1,0);
        v2.at<double>(2,0) = x_axis.at<double>(2,0);

        v3 = v1.cross(v2);
        y_axis = v3.clone();
        break;
    case 3:
        if(z_axis.at<double>(1,0) > 0)
        {
            z_axis.at<double>(0,0) = -1 * z_axis.at<double>(0,0);
            z_axis.at<double>(1,0) = -1 * z_axis.at<double>(1,0);
            z_axis.at<double>(2,0) = -1 * z_axis.at<double>(2,0);
            v1.at<double>(0,0) = z_axis.at<double>(0,0);
            v1.at<double>(1,0) = z_axis.at<double>(1,0);
            v1.at<double>(2,0) = z_axis.at<double>(2,0);
            v2.at<double>(0,0) = x_axis.at<double>(0,0);
            v2.at<double>(1,0) = x_axis.at<double>(1,0);
            v2.at<double>(2,0) = x_axis.at<double>(2,0);
            v3 = v1.cross(v2);
            y_axis = v3.clone();

        }

        break;
    default:
        std::cout<<">> case default"<<std::endl;
        break;

    }

    dstR.at<double>(0,0) = x_axis.at<double>(0,0);
    dstR.at<double>(1,0) = x_axis.at<double>(1,0);
    dstR.at<double>(2,0) = x_axis.at<double>(2,0);

    dstR.at<double>(0,1) = y_axis.at<double>(0,0);
    dstR.at<double>(1,1) = y_axis.at<double>(1,0);
    dstR.at<double>(2,1) = y_axis.at<double>(2,0);

    dstR.at<double>(0,2) = z_axis.at<double>(0,0);
    dstR.at<double>(1,2) = z_axis.at<double>(1,0);
    dstR.at<double>(2,2) = z_axis.at<double>(2,0);
}
