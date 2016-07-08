#ifndef  GETANGLE_H
#define GETANGLE_H
#ifndef PI
#define PI 3.1416
#endif // PI
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>


static cv::Mat rotateMat;
static cv::Mat tranVector;
static double sunAzimuth;//方位角
static double sunZenith;
static double sunAltitude;//高度角，太阳方向和地面的夹角
static double focalLength;
static double width;
static double height;
typedef CvPoint3D64f XgyVector3D;
static cv::Point3f camPos;
typedef struct Line{
	//直线上的两点；
	cv::Point2f x1;
	cv::Point2f x2;

	//设定直线
	void setLine(cv::Point2f p1,cv::Point2f p2);
	cv::Point2f getDirection();	//获取直线的方向

}XgyLine;
typedef struct Plane //平面数据结构ax+by+cz+d=0;
{
	double a;
	double b;
	double c;
	double d;

	void setPlane(double a,double b,double c,double d);
}XgyPlane;
void setRT(cv::Mat R,cv::Mat T);
bool img2world(cv::Point2f iP,XgyPlane pl,cv::Point3f &wP);
double getVectorLength3D(XgyVector3D v);
double normalizeVector3D(XgyVector3D &v);
XgyVector3D setVector3D(cv::Point3f p1,cv::Point3f p2);
void getcamPose(cv::Point3f & campose);
double dotProduct3D(XgyVector3D v1,XgyVector3D v2);
void scalarMul(double num,XgyVector3D src,XgyVector3D &tar);
bool setSunAngleEx(std::vector<XgyLine> lineArray);
void setFlocallength(double f);
void printAngle();
void setWandH(double w,double h);
void setVecA(std::vector<XgyLine>& lineArray,std::vector<cv::Point2f> data);
void getAngle(cv::Mat R,cv::Mat T,double f,std::vector<cv::Point2f> data,double&_sunAzimuth,double& _sunAltitude,double w,double h );//data为三个点，第一个点为阴影点，第二个点为脚点，第三个点为头点,f为focallength
bool world2img(cv::Mat rotateMat,cv::Mat tranVector,double focalLength,cv::Point3d wP, cv::Point2d &iP);
#endif
