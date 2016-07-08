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
static double sunAzimuth;//��λ��
static double sunZenith;
static double sunAltitude;//�߶Ƚǣ�̫������͵���ļн�
static double focalLength;
static double width;
static double height;
typedef CvPoint3D64f XgyVector3D;
static cv::Point3f camPos;
typedef struct Line{
	//ֱ���ϵ����㣻
	cv::Point2f x1;
	cv::Point2f x2;

	//�趨ֱ��
	void setLine(cv::Point2f p1,cv::Point2f p2);
	cv::Point2f getDirection();	//��ȡֱ�ߵķ���

}XgyLine;
typedef struct Plane //ƽ�����ݽṹax+by+cz+d=0;
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
void getAngle(cv::Mat R,cv::Mat T,double f,std::vector<cv::Point2f> data,double&_sunAzimuth,double& _sunAltitude,double w,double h );//dataΪ�����㣬��һ����Ϊ��Ӱ�㣬�ڶ�����Ϊ�ŵ㣬��������Ϊͷ��,fΪfocallength
bool world2img(cv::Mat rotateMat,cv::Mat tranVector,double focalLength,cv::Point3d wP, cv::Point2d &iP);
#endif
