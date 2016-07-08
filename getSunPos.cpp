#include "getSunPos.h"
#include "common.h"
#include "right_hand_rule.h"


void getSunPos(const int H, const int W, const cv::Point2f data_point[11],cv::Point2d &sunPos, double &theta, double &phi)
{
    cv::Point2f point[11];
    for(int i=0; i<11; i++)
    {
        point[i] = data_point[i];
    }
    /**vx, vy 分别代表x方向,y方向的消失点*/
    cv::Point2f vx;
    cv::Point2f vy;
    cv::Point2f vz;/**z 方向的消失点*/
    cv::Point2f v_prime;
    cv::Point2d principal_point; /**相机主点**/
    cv::Mat K = cv::Mat_<double>(3,3); /**相机内参K**/
    cv::Mat w = cv::Mat_<double>(3,3); /** w = K.inv().t()*K.inv() */
    vx = get_line_intersection(point[0], point[1], point[2], point[3]);
    vy = get_line_intersection(point[4], point[5], point[6], point[7]);
    double f; /**焦距**/

    /**调整消失点，使得z-axis朝上**/
   // adjust_vanishing_point(vx, vy, point[10],vx, vy);

    /***失点vx, vy 所在直线与头的阴影点，脚点所在直线的交点为 v', 用v_prime表示*/
    v_prime = get_line_intersection(vx, vy, point[9], point[10]);

    /**求焦距f**/
    cv::Point2d v1, v2;
//    v1.x = vx.x - (double)W/2;
//    v1.y = vx.y - (double)H/2;
//    v2.x = vy.x - (double)W/2;
//    v2.y = vy.y - (double)H/2;

    v1.x = vx.x ;
    v1.y = vx.y ;
    v2.x = vy.x ;
    v2.y = vy.y ;
    f = get_focal_length2(v1, v2);
   // std::cout<<">> focal length f = "<<  f  <<std::endl;
    /**求相机主点**/
    principal_point.x = (double)W/2;
    principal_point.y = (double)H/2;
    /**求K**/
    K.at<double>(0,0) = f;
    K.at<double>(0,1) = 0;
    K.at<double>(0,2) = principal_point.x;
    K.at<double>(1,0) = 0;
    K.at<double>(1,1) = f;
    K.at<double>(1,2) = principal_point.y;
    K.at<double>(2,0) = 0;
    K.at<double>(2,1) = 0;
    K.at<double>(2,2) = 1;
    /**求w**/
    w = K.t().inv() * K.inv();
    //std::cout<<"w = "<<w<<std::endl;
    /**求z-axis消失点**/
    cv::Mat mx = (cv::Mat_<double>(3,1) << vx.x, vx.y, 1); /**将vx, vy变为齐次的*/
    cv::Mat my = (cv::Mat_<double>(3,1) << vy.x, vy.y, 1);
    cv::Mat xw = cv::Mat_<double>(1,3);
    cv::Mat yw = cv::Mat_<double>(1,3);
    xw = mx.t() * w;

    yw = my.t() * w;
    cv::Mat B = (cv::Mat_<double>(2,1) << -xw.at<double>(0,2), -yw.at<double>(0,2));
    cv::Mat A = (cv::Mat_<double>(2,2) << xw.at<double>(0,0), xw.at<double>(0,1), yw.at<double>(0,0), yw.at<double>(0,1));
    cv::Mat mz = cv::Mat_<double>(2,2);
   // std::cout<<"A = "<<A<<std::endl;
    mz = A.inv() * B;
   //mz = (A.t()*A).inv()*A.t()*B;
   /* std::cout<<"A.inv="<<A.inv()<<std::endl;
    std::cout<<"detA"<<cv::determinant(A)<<std::endl;
    std::cout<<"B="<<B<<std::endl;*/
    vz.x = mz.at<double>(0,0);
    vz.y = mz.at<double>(1,0);
    cv::Mat tmz = (cv::Mat_<double>(3,1) << vz.x, vz.y, 1);
    /*std::cout<<">> vy' * w * vx  = "<<std::endl;
    std::cout<< mx.t() * w * my<<std::endl;
    std::cout<<">> vy' * w * vz  = "<<std::endl;
    std::cout<< my.t() * w * tmz<<std::endl;
    std::cout<<">> vx' * w * vz  = "<<std::endl;
    std::cout<< mx.t() * w * tmz<<std::endl;*/

    /**v_prime与z轴消失点vz所在直线与头的阴影点，头点所在直线交点即太阳位置**/
    sunPos = get_line_intersection(v_prime, vz, point[8], point[9]);

    /**根据论文公式求theta和phi**/
    get_theta_and_phi(w, sunPos, v_prime, vx, vy, vz, theta, phi);

}
void get_theta_and_phi(const cv::Mat w, const cv::Point2d sunPos, const cv::Point2d v_prime, \
                       const cv::Point2d vx, const cv::Point2d vy, const cv::Point2d vz,  double &theta, double &phi)
{
    cv::Point2d vs = get_line_intersection(vx, vy, vz, sunPos);
    cv::Mat mv_prime = (cv::Mat_<double>(3,1) << v_prime.x, v_prime.y, 1);
    cv::Mat mvx = (cv::Mat_<double>(3,1) << vx.x, vx.y, 1);
    cv::Mat mvy = (cv::Mat_<double>(3,1) << vy.x, vy.y, 1);
    cv::Mat mvz = (cv::Mat_<double>(3,1) << vz.x, vz.y, 1);
    cv::Mat mv = (cv::Mat_<double>(3,1) << sunPos.x, sunPos.y, 1);
    cv::Mat mvs = (cv::Mat_<double>(3,1) << vs.x, vs.y, 1);
   // std::cout<<">> mvz :"<<mvz<<std::endl;
   // std::cout<<">> mvs :"<<mvs<<std::endl;
    /**求theta*/
    cv::Mat tp = mvx.t()*w*mvs;
    cv::Mat dwn1 = mvs.t()*w*mvs;
    cv::Mat dwn2 = mvx.t()*w*mvx;
    double cosTheta = tp.at<double>(0,0)/(sqrt(dwn1.at<double>(0,0)) * sqrt(dwn2.at<double>(0,0)));
    theta = acos(cosTheta);
    /**求phi**/
    tp = mvz.t()*w*mv;
    dwn1 = mv.t()*w*mv;
    dwn2 = mvz.t()*w*mvz;
    double cosPhi = tp.at<double>(0,0)/(sqrt(dwn1.at<double>(0,0)) * sqrt(dwn2.at<double>(0,0)));
    phi = acos(cosPhi);
    if(phi > PI/2)
        phi = PI-phi;
 //   std::cout<<"theta = "<<(theta/3.14)*180<<" phi = "<<(phi/3.14)*180<<std::endl;
}
