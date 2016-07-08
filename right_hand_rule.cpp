#include "right_hand_rule.h"
#include "common.h"

bool adjust_vanishing_point(const cv::Point2f  _v1, const cv::Point2f _v2, const cv::Point2f origin, cv::Point2f &v1, cv::Point2f &v2)
{
    cv::Point2f tv1, tv2;
    tv1 = _v1;
    tv2 = _v2;
    v1 = _v1;
    v2 = _v2;

    cv::Mat lvector[3];//坐标轴方向向量
	lvector[0] = cv::Mat_<double>(2, 1);
	lvector[1] = cv::Mat_<double>(2, 1);
	lvector[2] = cv::Mat_<double>(2, 1);

	lvector[0].at<double>(0,0) = tv1.x - origin.x;
	lvector[0].at<double>(1,0) = tv1.y - origin.y;
	lvector[1].at<double>(0,0) = tv2.x - origin.x;
	lvector[1].at<double>(1,0) = tv2.y - origin.y;

	cv::normalize(lvector[0],lvector[0]);
	cv::normalize(lvector[1],lvector[1]);

	double angle_cos = (lvector[0].at<double>(0, 0) * lvector[1].at<double>(0, 0) + \
                     lvector[0].at<double>(1, 0) * lvector[1].at<double>(1, 0)) / (sqrt(lvector[0].at<double>(0, 0)*lvector[0].at<double>(0, 0) + \
                    lvector[0].at<double>(1, 0)*lvector[0].at<double>(1, 0))*sqrt(lvector[1].at<double>(0, 0)*lvector[1].at<double>(0, 0) + \
                                                            lvector[1].at<double>(1, 0)*lvector[1].at<double>(1, 0)));//夹角cos值
	double xy_angle = acos(angle_cos);
	double angle_sin = sin(xy_angle);
	cv::Mat R = cv::Mat_<double>(2, 2);//角度旋转矩阵
	R.at<double>(0, 0) = angle_cos;
	R.at<double>(0, 1) = -angle_sin;
	R.at<double>(1, 0) = angle_sin;
	R.at<double>(1, 1) = angle_cos;

	cv::gemm(R.inv(), lvector[0], 1, 0, 0, lvector[2], 0);
	double dx = lvector[2].at<double>(0, 0) - lvector[1].at<double>(0, 0), dy = lvector[2].at<double>(1, 0) - lvector[1].at<double>(1, 0);
    std::cout<<"dx = "<<std::endl;
    std::cout<<dx<<std::endl;
    std::cout<<"dy = "<<std::endl;
    std::cout<<dy<<std::endl;
	if (fabs(dx) > 0.1 || fabs(dy) > 0.1)
    {
        v1 = _v2;
        v2 = _v1;
        return true;
    }
    return false;
}


void adjustInteraction(cv::Point2f src_point[11], cv::Point2f dst_point[11])
{
    cv::Point2f v1, v2;
    v1 = getVanishPoint(src_point[0],src_point[1], src_point[2], src_point[3]);
    v2 = getVanishPoint(src_point[4],src_point[5], src_point[6], src_point[7]);
    for(int i=0; i<11; i++)
    {
        dst_point[i] = src_point[i];
    }
    if(adjust_vanishing_point(v1, v2, src_point[10], v1, v2) == true)
    {
        cv::Point2f temp;
        for(int i=0; i<4; i++)
        {
            temp = dst_point[i+4];
            dst_point[i+4] = dst_point[i];
            dst_point[i] = temp;

        }
    }
    return ;

}
