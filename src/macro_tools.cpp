#include <macro.h>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/stitching/detail/warpers_inl.hpp>
#include "readparameter.h"
#include "log.h"

#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
double time_start=0;
const float kPI=3.141593;
////////////////////////////// struct //////////////////////////////

PointList::PointList() {
    const vector<Point3f> tmp[8] ={ //从左到右依次是 左灯条{0上定点,1下顶点,2中心点} 数字贴纸{3左中心点,4右中心点,5上中心点,6下中心点,7中心点} 右灯条{8上定点,9下顶点,10中心点}
            /*None******/{Point3f(-kSAW/2,-kLBH/2,0.f),Point3f(-kSAW/2,kLBH/2,0.f),Point3f(-kSAW/2,0.f,0.f),Point3f(-kNumW[0]/2,0.f,0.f),Point3f(kNumW[0]/2,0.f,0.f),Point3f(0.f,-kNumH[0]/2,0.f),Point3f(0.f,kNumH[0]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSAW/2,-kLBH/2,0.f),Point3f(kSAW/2,kLBH/2,0.f),Point3f(kSAW/2,0.f,0.f),},
            /*HERO******/{Point3f(-kSBW/2,-kLBH/2,0.f),Point3f(-kSBW/2,kLBH/2,0.f),Point3f(-kSBW/2,0.f,0.f),Point3f(-kNumW[1]/2,0.f,0.f),Point3f(kNumW[1]/2,0.f,0.f),Point3f(0.f,-kNumH[1]/2,0.f),Point3f(0.f,kNumH[1]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSBW/2,-kLBH/2,0.f),Point3f(kSBW/2,kLBH/2,0.f),Point3f(kSBW/2,0.f,0.f),},
            /*ENGINEER**/{Point3f(-kSAW/2,-kLBH/2,0.f),Point3f(-kSAW/2,kLBH/2,0.f),Point3f(-kSAW/2,0.f,0.f),Point3f(-kNumW[2]/2,0.f,0.f),Point3f(kNumW[2]/2,0.f,0.f),Point3f(0.f,-kNumH[2]/2,0.f),Point3f(0.f,kNumH[2]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSAW/2,-kLBH/2,0.f),Point3f(kSAW/2,kLBH/2,0.f),Point3f(kSAW/2,0.f,0.f),},
            /*INFANTRY3*/{Point3f(-kSAW/2,-kLBH/2,0.f),Point3f(-kSAW/2,kLBH/2,0.f),Point3f(-kSAW/2,0.f,0.f),Point3f(-kNumW[3]/2,0.f,0.f),Point3f(kNumW[3]/2,0.f,0.f),Point3f(0.f,-kNumH[3]/2,0.f),Point3f(0.f,kNumH[3]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSAW/2,-kLBH/2,0.f),Point3f(kSAW/2,kLBH/2,0.f),Point3f(kSAW/2,0.f,0.f),},
            /*INFANTRY4*/{Point3f(-kSAW/2,-kLBH/2,0.f),Point3f(-kSAW/2,kLBH/2,0.f),Point3f(-kSAW/2,0.f,0.f),Point3f(-kNumW[4]/2,0.f,0.f),Point3f(kNumW[4]/2,0.f,0.f),Point3f(0.f,-kNumH[4]/2,0.f),Point3f(0.f,kNumH[4]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSAW/2,-kLBH/2,0.f),Point3f(kSAW/2,kLBH/2,0.f),Point3f(kSAW/2,0.f,0.f),},
            /*INFANTRY5*/{Point3f(-kSAW/2,-kLBH/2,0.f),Point3f(-kSAW/2,kLBH/2,0.f),Point3f(-kSAW/2,0.f,0.f),Point3f(-kNumW[5]/2,0.f,0.f),Point3f(kNumW[5]/2,0.f,0.f),Point3f(0.f,-kNumH[5]/2,0.f),Point3f(0.f,kNumH[5]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSAW/2,-kLBH/2,0.f),Point3f(kSAW/2,kLBH/2,0.f),Point3f(kSAW/2,0.f,0.f),},
            /*SENTRY****/{Point3f(-kSBW/2,-kLBH/2,0.f),Point3f(-kSBW/2,kLBH/2,0.f),Point3f(-kSBW/2,0.f,0.f),Point3f(-kNumW[6]/2,0.f,0.f),Point3f(kNumW[6]/2,0.f,0.f),Point3f(0.f,-kNumH[6]/2,0.f),Point3f(0.f,kNumH[6]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSBW/2,-kLBH/2,0.f),Point3f(kSBW/2,kLBH/2,0.f),Point3f(kSBW/2,0.f,0.f),},
            /*BASE******/{Point3f(-kSBW/2,-kLBH/2,0.f),Point3f(-kSBW/2,kLBH/2,0.f),Point3f(-kSBW/2,0.f,0.f),Point3f(-kNumW[6]/2,0.f,0.f),Point3f(kNumW[6]/2,0.f,0.f),Point3f(0.f,-kNumH[6]/2,0.f),Point3f(0.f,kNumH[6]/2,0.f),Point3f(0.f,0.f,0.f),Point3f(kSBW/2,-kLBH/2,0.f),Point3f(kSBW/2,kLBH/2,0.f),Point3f(kSBW/2,0.f,0.f),},};
    for(int i=0;i<8;i++){
        RobotWorldPoints_List[i]=tmp[i];
    }
    for(int i=0;i<5;i++){
        if(i==0){
            EBWorldPoints_List[i]={Point3f(0,0,0),Point3f(0,kEBr,14.5f),Point3f(-kEBAMwidth/2,kEBAMheight/2+kEBr,14.5),Point3f(kEBAMwidth/2,kEBAMheight/2+kEBr,14.5)};
        } else{
            EBWorldPoints_List[i]={Point3f(-kEBAMwidth/2,kEBr,14.5),Point3f(kEBAMwidth/2,kEBr,14.5)};
            for(int j=0;j<EBWorldPoints_List[i].size();j++){
                Vec3f diffrot = {0,0,-i *2*kPI /5};//旋转的roll角度
                Mat rotmatix=eulerAnglesToRotationMatrix(diffrot);
                Point3f tmpp3=EBWorldPoints_List[i][j];
                Mat p=rotmatix*Mat(Point3d(tmpp3));//物体坐标系中点的位置
                p.convertTo(p,CV_32F);;
                EBWorldPoints_List[i][j]=(*(p.ptr<Point3f>()));//装甲版中心点
            }
        }
    }
}
////////////////////////////// others //////////////////////////////

void father_process(pid_t child_pid,int* child_time) {
    int status;
    int retval;
    while (1) {
        sleep(2);//挂起两秒
        OERROR("看门狗正常");
        struct timeval tv;
        gettimeofday(&tv, NULL);
        int father_time = tv.tv_sec;
        pid_t wait_flag = waitpid(child_pid, &status, WNOHANG);
        if ((wait_flag == 0 && abs(father_time - *child_time) > 30))//没有已退出的子进程可收集，则返回0；||且有500毫秒没有收到信息
        {
            OWARNING("程序超时");
            cout<<"小老弟，程序超时了，然后看门狗把它强制关闭了。等一会，马上重启！哈哈哈"<<endl;
            retval = kill(child_pid, SIGKILL);
            if (retval) {
                cout << "kill失败（一般这一句不会输出，如果输出了一定要告诉我）" << endl;
                break;
            } else {
                cout << "kill成功" << endl;
                break;
            }
        } else if (wait_flag > 0) {
            cout << "程序自己退出了。那谁，出来背锅！" << endl;
            break;
        }
    }
}
void feed_dog(){
    struct timeval time_child;
    gettimeofday(&time_child, NULL);
    *child_time= time_child.tv_sec;

}

/**********************************
 * @函数名 minAreaRect
 * @作者
 * @联系方式
 * @简介
 * @传入参数
 * @传出参数 vector<Points>,float angle,float deiation
 **********************************/
cv::RotatedRect minAreaRect( InputArray _points, float angle, float deviation ) {
    Mat hull;
    Point2f out[3];
    RotatedRect box;

    cv::convexHull(_points, hull, true, true);

    if( hull.depth() != CV_32F ) {
        Mat temp;
        hull.convertTo(temp, CV_32F);
        hull = temp;
    }

    const Point2f* hpoints = hull.ptr<Point2f>();


    angle=-angle;
    vector<Point> hullpoints;
    for(int i=0;i<hull.rows;i++){
        float x1=hpoints[i].x;
        float y1=hpoints[i].y;
        float x2=cos(angle*CV_PI/180)*x1-sin(angle*CV_PI/180)*y1;
        float y2=sin(angle*CV_PI/180)*x1+cos(angle*CV_PI/180)*y1;
        hullpoints.push_back(Point(x2,y2));
    }
    sort(hullpoints.begin(),hullpoints.end(),
         [](Point a,Point b)->bool{ return a.x<b.x;});
    Point2f p1=hullpoints[0];
    Point2f p2=hullpoints[hullpoints.size()-1];
    sort(hullpoints.begin(),hullpoints.end(),
         [](Point a,Point b)->bool{ return a.y<b.y;});

    Point2f p3=hullpoints[0];
    Point2f p4=hullpoints[hullpoints.size()-1];
    box.size.width=p2.x-p1.x+1;
    box.size.height=p4.y-p3.y+1;
    Point2f center_=Point2f((p1.x+p2.x)/2,(p3.y+p4.y)/2);
    box.center.x =cos(-angle*CV_PI/180)*center_.x-sin(-angle*CV_PI/180)*center_.y;
    box.center.y=sin(-angle*CV_PI/180)*center_.x+cos(-angle*CV_PI/180)*center_.y;
    box.angle=-angle*CV_PI/180;

    box.angle = (float)(box.angle*180/CV_PI);
    return box;
}


Rect rectCenterScale(Rect rect, Size2f size) {
    Size arg=Size(cvRound((size.width-1) *rect.size().height),  cvRound((size.height-1) *rect.size().height));
    rect= rect + arg;
    Point pt;
    pt.x = cvRound(arg.width/2.0);
    pt.y = cvRound(arg.height/2.0);
    return (rect-pt);
}

bool RectSafety(Rect &brect,Size size) {
    Rect out_rect=Rect(0,0,size.width,size.height);
    brect=brect&out_rect;
    return brect.area()!=0;

}

bool RectSafety(Rect &brect, int rows, int cols) {
    Rect out_rect=Rect(0,0,cols,rows);
    brect=brect&out_rect;
    return brect.width != 0 && brect.height != 0;

}


float getDistance(Point pointO, Point pointA) {
    float distance;
    //distance=pointO.x-pointA.x;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}
float getDistance(Point2f pointO, Point2f pointA) {
    float distance;
    //distance=pointO.x-pointA.x;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}

float getDistance(Point3f pointO, Point3f pointA) {
    float distance;
    //distance=pointO.x-pointA.x;
    distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2) +powf((pointO.z - pointA.z), 2);
    distance = sqrtf(distance);
    return distance;
}
float getDist_P2L(Point pointP, Point pointA, Point pointB) {
    //求直线方程
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x * pointB.y - pointA.y * pointB.x;
    //代入点到直线距离公式
    float distance = 0;
    distance = ((float)abs(A * pointP.x + B * pointP.y + C)) /
               (sqrtf(A * A + B * B));
    return distance;
}


void LeftRightLightBarSafety(LightBar &left_lightbar,LightBar &right_lightbar){
    if(left_lightbar.get_center().x>right_lightbar.get_center().x){
        LightBar tmp=left_lightbar;
        left_lightbar=right_lightbar;
        right_lightbar=tmp;
    }
}

float getSimilarity(const cv::Mat& first,const cv::Mat& second){
    double dotSum=first.dot(second);//内积
    double normFirst=cv::norm(first);//取模
    double normSecond=cv::norm(second);
    if(normFirst!=0 && normSecond!=0){
        return static_cast<float >(dotSum/(normFirst*normSecond));
    } else{
        return normFirst==normSecond? 1:0;
    }
}


Vec2f parabolasolve(Point2f ThroughPoint,float kv,float kg){
    //x1=v*cos@*t
    //y1=v*sin@t-g*t^2/2
    //联立方程消去t,得关于出射角tan@的方程kg*x1*x1/(2*kv*kv)*tan@^2+x1*tan@+kg*x1*x1/(2*kv*kv)-y1=0
    float a,b,c;
    float x1=ThroughPoint.x;
    float y1=ThroughPoint.y;
    a=kg*x1*x1/(2*kv*kv);b=x1;
    c=kg*x1*x1/(2*kv*kv)-y1;
    float tan_phi0, tan_phi1,phi0,phi1;
    tan_phi0=(-b+sqrt(b*b-4*a*c))/(2*a);
    tan_phi1=(-b-sqrt(b*b-4*a*c))/(2*a);
    phi0=atan(tan_phi0);phi1=atan(tan_phi1);
    Vec2f ret={-phi0,-phi1};

    return ret;
}
void LeastSquare(const vector<float>& x, const vector<float>& y,float&a, float&b) {
    float t1=0, t2=0, t3=0, t4=0;
    for(int i=0; i<x.size(); ++i){
        t1 += x[i]*x[i];
        t2 += x[i];
        t3 += x[i]*y[i];
        t4 += y[i];
    }
    a = (t3*x.size() - t2*t4) / (t1*x.size() - t2*t2);
    b = (t1*t4 - t2*t3) / (t1*x.size() - t2*t2);
}

double get_timenow(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return double(tv.tv_sec*1000)+double(tv.tv_usec/1000.f)-time_start;
}

////////////////////
Polor3f to_polor(const Point3f& rectangular_point){
    Polor3f polor_point;
    polor_point.distance=static_cast<float>(cv::norm(Mat(rectangular_point,CV_32F)));
    polor_point.yaw=atan(rectangular_point.x/rectangular_point.z);
    if(rectangular_point.z<0){
        if(rectangular_point.x>0) polor_point.yaw+=kPI;
        else polor_point.yaw-=kPI;
    }
    polor_point.pitch=-asin(rectangular_point.y/polor_point.distance);
    return polor_point;
}
Point3f to_sprectangular(const Polor3f& polor_point){
    Point3f rectangular_point;
    rectangular_point.y=-sin(polor_point.pitch)*polor_point.distance;
    float r=cos(polor_point.pitch)*polor_point.distance;
    rectangular_point.x=r*sin(polor_point.yaw);
    rectangular_point.z=r*cos(polor_point.yaw);

    return rectangular_point;
}
////////////////////////////////////////////////////////

// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta ,bool order)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
                                 1,       0,              0,
            0,       cos(theta[0]),   -sin(theta[0]),
            0,       sin(theta[0]),   cos(theta[0])
    );

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
                                 cos(theta[1]),    0,      sin(theta[1]),
            0,               1,      0,
            -sin(theta[1]),   0,      cos(theta[1])
    );

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
                                 cos(theta[2]),    -sin(theta[2]),      0,
            sin(theta[2]),    cos(theta[2]),       0,
            0,               0,                  1);

    // Combined rotation matrix
    Mat R;
    if(order) {
        R = R_z * R_y * R_x;
    } else{
        R = R_x*R_y* R_z;

    }

    return R;
}


// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

Mat eulerAnglesToRvec(Vec3f &theta){
    Mat rotmat,rvec;
    rotmat=eulerAnglesToRotationMatrix(theta);
    Rodrigues(rotmat,rvec);
    return rvec;
}
Vec3f rvecToEulerAngles(Mat &R){
    Mat rotmat;
    Rodrigues(R,rotmat);
    Vec3f oe= rotationMatrixToEulerAngles(rotmat);
    return oe;
}

////////////////////////////////////////////////////////
float getDiffangle(float a,float b){
    AngleCorrect(a);
    AngleCorrect(b);
    float  diffangle=a-b;
    if(fabs(diffangle)>kPI){//考虑到2PI处瞬变,这里的处理可以得到真实角度差
        if(diffangle>0){
            diffangle=-2*kPI+diffangle;
        } else{
            diffangle+=2*kPI;
        }
    }
    return diffangle;
}
void AngleCorrect(float &a){
    while (a>kPI){
        a-=2*kPI;
    }
    while (a<-kPI){
        a+=2*kPI;
    }
}
