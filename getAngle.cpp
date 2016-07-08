#include"getAngle.h"
void XgyLine::setLine(cv::Point2f p1,cv::Point2f p2)
{
	x1.x=p1.x;
	x1.y=p1.y;
	x2.x=p2.x;
	x2.y=p2.y;
}
cv::Point2f XgyLine::getDirection()
{
	cv::Point2f v;
	v.x=x2.x-x1.x;
	v.y=x2.y-x1.y;
	double l=sqrt(v.x*v.x+v.y*v.y);
	if(l!=0)
	{
		v.x/=l;
		v.y/=l;
	}
	return v;
}

void XgyPlane::setPlane(double a, double b, double c, double d)
{
	this->a=a;
	this->b=b;
	this->c=c;
	this->d=d;
}
void setRT(cv::Mat R,cv::Mat T)
{
	rotateMat=R;
	tranVector=T;
}
void setWandH(double w,double h)
{
	width = w;
	height = h;
}
bool img2world(cv::Point2f iP,XgyPlane pl,cv::Point3f &wP)
{

	cv::Mat temp_cP(3,1,CV_64F);
	temp_cP.at<double>(0,0)=(iP.x-0.5*width)/focalLength;
	temp_cP.at<double>(1,0)=(iP.y-0.5*height)/focalLength;
	temp_cP.at<double>(2,0)=1;



	cv::Mat v1(3,1,CV_64F);
	cv::Mat invRot(3,3,CV_64F);
	invRot=rotateMat.t();
	v1=invRot*temp_cP;
	cv::Point3f t1;
	t1.x=v1.at<double>(0,0);
	t1.y=v1.at<double>(1,0);
	t1.z=v1.at<double>(2,0);

	cv::Mat v2(3,1,CV_64F);
	v2=invRot*tranVector;
	cv::Point3f t2;
    t2.x=v2.at<double>(0,0);
	t2.y=v2.at<double>(1,0);
	t2.z=v2.at<double>(2,0);


	double temp=pl.a*t1.x+pl.b*t1.y+pl.c*t1.z;
	double cPz;
	if(temp!=0)
		cPz=(pl.a*t2.x+pl.b*t2.y+pl.c*t2.z-pl.d)/temp;
	else
	{
		return false;
	}

	if(cPz<0)
	{
		return false;
	}

	wP.x=cPz*t1.x-t2.x;
	wP.y=cPz*t1.y-t2.y;
	wP.z=cPz*t1.z-t2.z;



	return true;
}
bool getPlanefromLine(XgyLine l, XgyPlane &pl)
{
	XgyPlane floor;
	floor.setPlane(0,0,1,0);

	//计算线段两端点的3维位置
	cv::Point3f wP1;
	if(!(img2world(l.x1,floor,wP1)))
	{
		//cout<<"Can not create plane!"<<endl;
		return false;
	}

	cv::Point3f wP2;
	if(!(img2world(l.x2,floor,wP2)))
	{
		//cout<<"Can not create plane!"<<endl;
		return false;
	}

	XgyVector3D tempNormal;	//平面法向垂直于直线wP1wP2，且平行于z=0
	tempNormal.x=wP1.y-wP2.y;
	tempNormal.y=wP2.x-wP1.x;
	tempNormal.z=0;
	normalizeVector3D(tempNormal);

	XgyVector3D view;
	getcamPose(camPos);
	view=setVector3D(camPos,wP1);
	normalizeVector3D(view);

	XgyVector3D normal;
	if(dotProduct3D(tempNormal,view)>0)	//若法向与视线同向
		scalarMul(-1.0,tempNormal,normal);	//反向
	else
		scalarMul(1.0,tempNormal,normal);

	double d=-1*(normal.x*wP1.x+normal.y*wP1.y+normal.z*wP1.z);
	pl.setPlane(normal.x,normal.y,normal.z,d);

	return true;
}
double getVectorLength3D(XgyVector3D v)
{
	double l;
	l=0;
	l=v.x*v.x+v.y*v.y+v.z*v.z;
	l=sqrt(l);
	return l;
}
double normalizeVector3D(XgyVector3D &v)
{
	double l;
	l=getVectorLength3D(v);
	if(l==0)
		return 0;
	v.x/=l;
	v.y/=l;
	v.z/=l;
	return l;
}
XgyVector3D setVector3D(cv::Point3f p1,cv::Point3f p2)
{
	XgyVector3D v;
	v.x=p2.x-p1.x;
	v.y=p2.y-p1.y;
	v.z=p2.z-p1.z;
	return v;
}
void getcamPose(cv::Point3f & campose)
{
	cv::Mat invR(3,3,CV_64F);
	invR=rotateMat.t();
	cv::Mat camPos(3,1,CV_64F);	//正交阵求逆
	camPos=invR*tranVector;

   // std::cout<<"invR="<<invR<<std::endl;
	//std::cout<<"camPos"<<camPos<<std::endl;
	campose.x=-1*camPos.at<double>(0,0);
	campose.y=-1*camPos.at<double>(1,0);
	campose.z=-1*camPos.at<double>(2,0);

}
double dotProduct3D(XgyVector3D v1,XgyVector3D v2)
{
	double r=v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;
	return r;
}
void scalarMul(double num,XgyVector3D src,XgyVector3D &tar)
{
	tar.x=num*src.x;
	tar.y=num*src.y;
	tar.z=num*src.z;
}
bool setSunAngleEx(std::vector<XgyLine> lineArray)
{

    if(lineArray.size()==0)
		return false;

	XgyPlane P1;		//确定方位角
	P1.setPlane(0,0,1,0);
	cv::Point3f startP,endP;		//方位角指示线的起点和终点
	if(!img2world(lineArray[0].x2,P1,startP))
		return false;
	if(!img2world(lineArray[0].x1,P1,endP))
		return false;

	XgyVector3D v1;
	v1=setVector3D(startP,endP);
	normalizeVector3D(v1);
	double beijian=PI/180*90;
	if(v1.y>=0)
		{sunAzimuth=acos(v1.x);
	}
	else
		{sunAzimuth=-1.0*acos(v1.x);

	}
	if(lineArray.size()==1)
		return false;
	else
	{
		XgyPlane P2;
		getPlanefromLine(lineArray[0], P2);
		cv::Point3f endZP;			//高度角指示线终点
		if(!img2world(lineArray[1].x2,P2,endZP))
			return false;
		XgyVector3D v2;
		v2=setVector3D(startP,endZP);
		normalizeVector3D(v2);
		if(v2.z<0)
			return false;
		else
		{
			sunZenith=acos(v2.z);//sunZenith为太阳方向与z轴的夹角
			sunAltitude=beijian- sunZenith;//sunAltitude为太阳方向与y轴的夹角
		}
	}
	return true;
}
void printAngle()
{
	std::cout<<"sunAzimuth="<<sunAzimuth<<std::endl;
	std::cout<<"sunAltitude="<<sunAltitude<<std::endl;
}
void setVecA(std::vector<XgyLine>& lineArray,std::vector<cv::Point2f> data)
{
	lineArray[0].x1=data[1];
	lineArray[0].x2=data[0];
	lineArray[1].x1=data[1];
	lineArray[1].x2=data[2];
}
void setFlocallength(double f)
{
    focalLength=f;
}
void getAngle(cv::Mat R,cv::Mat T,double f,std::vector<cv::Point2f> data,double&_sunAzimuth,double& _sunAltitude,double w,double h)
{
	setWandH(w,h);
    setFlocallength(f);
	setRT(R,T);
	std::vector<XgyLine> lineArray(2);
	setVecA(lineArray,data);
	setSunAngleEx(lineArray);
	_sunAzimuth=sunAzimuth;
	_sunAltitude=sunAltitude;
	//printAngle();
}
bool world2img(cv::Mat rotateMat,cv::Mat tranVector,double focalLength,cv::Point3d wP, cv::Point2d &iP)
{
	//CvMat *originalPos=cvCreateMat(3,1,CV_32FC1);
	cv::Mat originalPos(3,1,CV_64F);
	originalPos.at<double>(0,0)=wP.x;
	originalPos.at<double>(1,0)=wP.y;
	originalPos.at<double>(2,0)=wP.z;
	//cvmSet(originalPos,0,0,wP.x);
	//cvmSet(originalPos,1,0,wP.y);
	//cvmSet(originalPos,2,0,wP.z);
	cv::Mat rotationPos(3,1,CV_64F);
	rotationPos=rotateMat*originalPos;
	//CvMat *rotationPos=cvCreateMat(3,1,CV_32FC1);	//旋转变换
	cv::Mat translationPos(3,1,CV_64F);
	translationPos=rotationPos+tranVector;

	//CvMat *translationPos=cvCreateMat(3,1,CV_32FC1);	//位移变换
	//std::cout<<"test img2world:"<<std::endl;
	//std::cout<<translationPos.at<double>(0,0)<<", "<<translationPos.at<double>(1,0)<<", "<<translationPos.at<double>(2,0)<<std::endl;

	cv::Point2f	imgPos;	//透视变换
	iP.x=0.5*width+translationPos.at<double>(0,0)*focalLength/translationPos.at<double>(2,0);
	iP.y=0.5*height+translationPos.at<double>(1,0)*focalLength/translationPos.at<double>(2,0);

	//iP.x=0.5*width+cvmGet(translationPos,0,0)*focalLength/cvmGet(translationPos,2,0);
	//iP.y=0.5*height+cvmGet(translationPos,1,0)*focalLength/cvmGet(translationPos,2,0);

	//if(cvmGet(translationPos,2,0)<=focalLength)
	if(translationPos.at<double>(2,0)<=focalLength)
	{
		//cvReleaseMat(&originalPos);
		//cvReleaseMat(&translationPos);
		//cvReleaseMat(&rotationPos);
		return false;
	}
	//cvReleaseMat(&originalPos);
	//cvReleaseMat(&translationPos);
	//cvReleaseMat(&rotationPos);


	return true;
}
