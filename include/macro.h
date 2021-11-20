/*****************************************************************************
 * @brief        工程的宏定义声明
 * @version      1.0.0.1
 *
 * @author
 * @qq
 *
 *----------------------------------------------------------------------------
 * Change History :
 * <Date>     | <Version> | <Author> | <Description>
 *----------------------------------------------------------------------------
 * 2019/02/20 | 1.0.0.1   | 黎容熙    | 代码规范
 *----------------------------------------------------------------------------
 *
*****************************************************************************/

#ifndef T_DT2019VISION_MACRO_H
#define T_DT2019VISION_MACRO_H

#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "sys/time.h"
#include <sys/statfs.h>

using namespace cv;
using namespace std;
/*工程模式：
 * 1 调用海康摄像头 HIKVISION_CAM
 * 2调用普通摄像头 NORMAL_CAM
 * 3调用录像 VIDEO_CAM
 */
extern double time_start;

/*打开相机标定：CALIBRATE*/
//#define CALIBRATE


#define DEBUG

////////////////////////////////////////////////////////
enum ClockWise{
    ANTICLOCKWISE=-1,CLOCKWISE=1
};
enum Color {BLUE=0, UNKNOW, RED=2};//灯条颜色
enum RobotType {//机器人种类
    TYPEUNKNOW=0, HERO=1,ENGINEER=2,INFANTRY3=3,INFANTRY4=4,INFANTRY5=5,SENTRY=6,BASE=7};
enum EnergyBuffType {//能量机关种类
    None=0, FlowWater=1,NoFlowWater=2};
enum ArmorType{
    Empty=0,Lightbar=1,NoLightbar=2 };
enum Mode{ArmorMode=0,EnergyBuffMode=1,LobMode=2};

extern int *child_time;
extern const float kPI;
///////////////////////////////////////////////////////////////////////////////////
/**
 * @brief    云台
 * @author
 */
typedef struct ShootPlatform{
    float pitch;
    float yaw;
    float bulletspeed=2.7f;
}ShootPlatform;

/**
 * @brief    从下位机接收的信息
 */

typedef struct ReciveMessage{
    bool lock_command= false;
    Mode mode=ArmorMode;
    bool firemode=false;
    float bulletspeed=2.7f;
    int enemy_color;
    ShootPlatform shoot_platform;


}ReciveMessage;
/**
 * @brief    发送给下位机的数据
 * @author
 */

typedef struct SendMessage{
    float pitch=0;
    float yaw=0;
    bool beat= false;
    bool no_object= true;
    char visiononline =0;
    char find_object_right=0;
    char find_object_left=0;
}SendMessage;

/**
 * @brief    空间极坐标
 * @author

 */

typedef struct Polor3f{
    float distance=0;
    float yaw=0;
    float pitch=0;
}Polor3f;

/**
 * @brief    物体坐标系的点
 * @author
 */
typedef struct PointList{
    PointList();
    float kNumH[8]={0,10.2,10.1,10.3,9.4,10.2,10.2,10.2};
    float kNumW[8]={0,4.4 ,6.6 ,6.3 ,7.2,6.4 ,6.5,6.5};
    float kLBH = 6.3;//装甲板灯条的高度
    float kSAW = 11.8;//小装甲板的宽度
    float kSBW = 21.8;//大装甲板的宽度
    float kEBr = 72;
    float kEBAMwidth = 20;
    float kEBAMheight = 10;
    vector<Point3f> RobotWorldPoints_List[8];
    vector<Point3f> EBWorldPoints_List[5];
}PointList;


//////////////////////////////////////////////////////////////////////////////////
/**
 * @brief    灯条
 * @author
 */
class LightBar {
public:
    LightBar(RotatedRect rotrect,Color color);
    LightBar();
    ~LightBar();
    ////旋转矩形的各个顶点
    inline  Point tl() const { return static_cast<Point>(tl_); }
    inline  Point tr() const { return static_cast<Point>(tr_); }
    inline  Point bl() const { return static_cast<Point>(bl_); }
    inline  Point br() const { return static_cast<Point>(br_); }
    inline Color get_color() const { return color_; }
    inline int get_height() const { return height_; }
    inline int get_width() const { return width_; }
    inline float get_angle() const { return angle_; }
    inline Point get_center() const { return center_; }
    inline void set_judge_search(int judge_search) { judge_search_=judge_search;}
    inline int get_judge_search() const { return judge_search_; }
    inline RotatedRect get_rotatedrect() const {return rotated_rect_;}
    inline int get_area() const {return area_;}
    inline Rect get_bounding_rect() const {return bounding_rect_;}
    inline float get_ratio() const{return ratio_width_to_height_;}
    inline bool empty() const {return empty_;}

private:
    ////增加成员变量记得修改=赋值符的重载
    vector<Point>contour_;
    int area_;
    float angle_;
    float width_;
    float height_;
    Point center_;
    Point2f tl_;
    Point2f tr_;
    Point2f bl_;
    Point2f br_;
    Rect bounding_rect_;
    RotatedRect rotated_rect_;
    Color color_;
    int judge_search_;//(0:none -1:left 1:right 2:both) have been search
    float ratio_width_to_height_;
    bool empty_;
};
/**
 * @brief    贴纸
 * @author
 */
class NumberStiker {
public:
    explicit NumberStiker(RotatedRect &rotated_rect);
    NumberStiker();
    ~NumberStiker();
    inline  Point tl() const { return static_cast<Point>(tl_); }
    inline  Point tr() const { return static_cast<Point>(tr_); }
    inline  Point bl() const { return static_cast<Point>(bl_); }
    inline  Point br() const { return static_cast<Point>(br_); }
    inline  vector<Point> get_contour() const { return contour_; }
    inline int get_height() const { return height_; }
    inline int get_width() const { return width_; }
    inline Rect get_bounding_rect() const { return bounding_rect_; }
    inline RotatedRect get_rotated_rect() const { return rotated_rect_;}
    inline float get_angle() const { return angle_;}
    inline Point2f get_center() const { return center_;}
    inline RotatedRect get_number_rect(){ return  number_rect_;}
    inline void set_number_rect(RotatedRect &number_rect){ number_rect_=number_rect;}
    inline bool empty() const { return empty_;}//之后要考虑加入Rect类型是否为空
    inline RotatedRect set_rotated_rect(RotatedRect &search_rotatedrect) { rotated_rect_=search_rotatedrect;}
    inline void set_bounding_rect(Rect &bouding_rect){ bounding_rect_=bouding_rect;}
private:
    float angle_;
    int width_;
    int height_;
    Point2f center_;
    Rect bounding_rect_;
    RotatedRect rotated_rect_;
    vector<Point>  contour_;
    Point2f tl_;
    Point2f tr_;
    Point2f bl_;
    Point2f br_;
    RotatedRect number_rect_;
    bool empty_;
};
/**
 * @brief    装甲板类
 * @author

 */
class Armor {
public:
    Armor();
    explicit Armor(NumberStiker numberstiker);
    Armor(NumberStiker numberstiker,LightBar left_lightbar,LightBar right_lightbar);
    Armor(NumberStiker numberstiker,LightBar single_lightbar);
    ~Armor();

    inline const LightBar& get_left_lightbar() const { return  left_lightbar_; }
    inline const LightBar& get_right_lightbar() const { return  right_lightbar_; }
    inline const NumberStiker& get_numberstiker() const { return numberstiker_; }
    inline const bool *get_form()  const  { return form_; }
    inline RobotType get_robot_type() const { return robot_type_; }
    inline RobotType set_robot_type(RobotType robot_type)  { robot_type_=robot_type; }
    inline Rect get_rect() const{return armor_rect_;}
    inline void set_rect(Rect &armor_rect) { armor_rect_=armor_rect;}
    inline ArmorType get_armor_type() const {return aromor_type_;}
    inline void set_armor_type( ArmorType aromor_type){aromor_type_=aromor_type;}
    inline bool empty() const {return empty_;}
    inline Mat get_ml_roi() const { return ml_roi_;}
    inline void set_ml_roi(Mat &ml_roi) {ml_roi.copyTo(ml_roi_); }
    inline Mat get_ml_hog() const { return ml_hog_;}
    inline void set_ml_hog(Mat &ml_hog) {ml_hog.copyTo(ml_hog_); }
    inline int get_num_threshold() const { return num_threshold_;}
    inline void set_num_threshold(int &num_threshold) {num_threshold_=num_threshold; }
    inline float get_similar_to_last() const { return similar_to_last_;}
    inline void set_similar_to_last(float similar_to_last) {similar_to_last_=similar_to_last; }
    float get_armor_angle();
    int get_distance_to_screen_center();

    RotatedRect  get_armor_rotatedrect();
    void calc_pnppoints(vector<Point2f> &pnp_points);

    //inline void roi_recover(Rect roi){armor_rect_+=Point(roi.x,roi.y);}
private:
    Rect armor_rect_;
    ArmorType aromor_type_;
    RobotType robot_type_;//机器人种类
    float armor_angle_;//装甲板角度
    RotatedRect armor_rotatedrect_;//装甲板的旋转矩形
    NumberStiker numberstiker_;
    LightBar left_lightbar_;
    LightBar right_lightbar_;
    bool form_[3];//装甲板种类 //0:leftbar 1:rightbar 2:numberstiker
    float distance;//距离
    float similar_to_last_;
    int distance_to_screen_center;
    int num_threshold_;
    Mat ml_roi_;
    Mat ml_hog_;
    bool empty_= true;
};
/////////////////////////////////////////////////////////////////////////////////

class EnergyBuffArmor{
public:
    EnergyBuffArmor();

    EnergyBuffArmor(RotatedRect armor_rotatedrect,bool up_OR_down);
    inline  Point2f tl() const { return tl_; }
    inline  Point2f tr() const { return tr_; }
    inline  Point2f bl() const { return bl_; }
    inline  Point2f br() const { return br_; }

    inline  float get_height() const {return height_;};
    inline  float get_width() const {return width_;};

    inline bool empty(){ return empty_;}
    inline Point2f center() const {return center_;}
    inline Point2f get_circle() const {return circle_;}
    void set_circle(Point2f circle);


    inline Mat get_ml_image(){ return ml_image_;}
    inline Mat set_ml_image(Mat &ml_image) {ml_image_=ml_image;}
    inline  RotatedRect get_rotatedrect() const { return  rotatedrect_;}
    inline  float get_angle(){ return  angle;}
    inline  EnergyBuffType get_type(){ return  type_;}
    inline void set_type(EnergyBuffType type){type_=type;}
    inline  float get_kd_r() const {return kd_r_;};
    inline  float get_kd_flowwater() const {return kd_flowwater_;};

    inline void set_kd_r(float kd_r)  {kd_r_=kd_r;};
    inline void set_kd_flowwater(float kd_flowwater)  { kd_flowwater_=kd_flowwater;};

    RotatedRect get_circle_rect_() const ;
    RotatedRect get_ml_rect_() const ;

private:

    Point2f center_;
    Point2f tl_;
    Point2f tr_;
    Point2f br_;
    Point2f bl_;

    float height_;
    float width_;
    float kd_r_=100;
    float kd_flowwater_=100;
    Point2f circle_;
    float angle;//0~360;
    bool empty_;
    Mat ml_image_;
    RotatedRect rotatedrect_;
    EnergyBuffType type_;

};
//////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  空间中的一个有限平面,用装甲板来封装
 */
class FinitePlane {
public:
    FinitePlane();
    FinitePlane(Mat rot_vector,Mat tran_vector, Armor &armor,float pnp_assess );
    FinitePlane(Mat rot_vector,Mat tran_vector,  EnergyBuffArmor &armor,float pnp_assess );

    inline bool empty() const { return empty_;};

    inline void set_world_point(Point3f &word_point){ world_point_=word_point;}
    inline Point3f get_world_point() const { return  world_point_;}
    inline Point2f get_image_point() const { return  image_point_;}
    inline void set_polor_point(Polor3f &polor_point){ polor_point_=polor_point;}
    inline Polor3f get_polor_point() const { return  polor_point_;}
    inline Vec3f get_euler_angle() const { return euler_angle_;}
    inline float get_pnpassess() const { return  pnp_assess_;}
    inline Size get_region_size() const { return region_size_;}
    inline  Mat get_rvec() const { return  rvec_;}
    inline  Mat get_tvec() const { return  tvec_;}

    inline float get_llh(){ return llheight_;}
    inline float get_rlh(){ return rlheight_;}

private:
    Point3f world_point_; //世界坐标系中的位置
    Point2f image_point_; //像素坐标系中的位置
    Size region_size_; //该装甲板在图像中区域的大小
    Vec3f euler_angle_; //欧拉角
    Polor3f polor_point_; //世界坐标系中的极坐标
    bool empty_;
    float pnp_assess_; //pnp评估,暂时没啥用
    Mat rvec_; //世界坐标系与物体坐标系的旋转矩阵
    Mat tvec_; //世界坐标系与物体坐标系的平移矩阵
    float llheight_=0; //装甲板左灯条高度
    float rlheight_=0; //装甲板右灯条高度,这两个成员变量不合理,以后会处理掉
};
/**
 * @brief    世界坐标系下的目标
 * @author
 */

class Target{
public:
    Target();
    Target(Mat &rvec, Mat &tvec,vector <FinitePlane> &planes,ReciveMessage recive_message);

    inline  Mat get_rvec() const { return  rvec_;}
    inline  Mat get_tvec() const { return  tvec_;}

    inline void set_time_now(double time_now){time_now_=time_now;}
    inline double get_time_now() const { return time_now_;}
    inline const vector<FinitePlane>&  get_planes() const { return planes_;}
    inline bool  empty() const { return empty_;}


protected:
    vector <FinitePlane> planes_;
    ReciveMessage recive_message_;
    bool empty_;
    double time_now_;  //此刻时间
    Mat rvec_;
    Mat tvec_;

};

class BuffTarget :public Target{
public:
    BuffTarget():Target(){;}
    BuffTarget(Mat &rvec,Mat&tvec,vector <FinitePlane> &planes,ReciveMessage recive_message);
    inline const int get_idex(){return idex_;}
    inline const EnergyBuffType* get_form(){return form_;}
    float set_instant_angle(float instant_angle){angle_ = instant_angle;}
    float get_instant_angle(void){return angle_;}

    void correctself(Mat state);
private:

    float angle_;
    Point3f circle_world_point_;
    Polor3f circle_world_polor_;
    EnergyBuffType form_[5];
    int idex_;
};

class RobotTarget :public Target{
public:
    RobotTarget():Target(){;}
    RobotTarget(Mat &rvec, Mat &tvec,vector <FinitePlane> &planes,RobotType robotType,ReciveMessage recive_message);

    inline void set_robot_type(RobotType type){robot_type_=type;}
    inline RobotType get_robot_type(){ return robot_type_;}
    inline int get_linkid(){ return linkid_;}
    inline void set_linkid(int linkid){ linkid_=linkid;}
    void correctself(Mat state);
    float get_axis_yaw();
    inline void set_axis_yaw(float axis_yaw){axis_yaw_=axis_yaw;}
    inline bool plane1_is_empty(){ return planes_[1].empty();}
private:
    RobotType robot_type_=TYPEUNKNOW;
    int linkid_=0;
    float axis_yaw_= -1;

};

/////////////////////////////////////////////////////////////////////////////////


cv::RotatedRect minAreaRect(InputArray _points, float angle, float deviation);

/***
     *  @name     LeftRightLightBarSafety
     *  @brief    左右灯条排序
     *  @param    left_lightbar  输出左灯条
     *  @param    right_lightbar 输出右灯条
     *  @author   李思祁
     ***/
void LeftRightLightBarSafety(LightBar &left_lightbar, LightBar &right_lightbar);

/**
 * @brief 判断两张图像相似度
 * @param first 第一张图像
 * @param second 第二张图像
 * @return 相似度 1完全相同 0完全不同
 */
float getSimilarity(const cv::Mat &first, const cv::Mat &second);

/**
 * @brief     看门狗
 * @author   黄志豪
 */
void father_process(pid_t child_pid, int *child_time);

/**
 * @brief     ”喂狗“，就是测个时间
 * @author   黄志豪
 */
void feed_dog();

/**
 * @brief     将Rect超出图像边界部分去除
 * @param rect  目标Rect
 * @param size 图像的size
 * @return   true 成功
 *           false Rect与图像没有重叠部分
 *
 * @author   李思祁
 * @qq 975216527
 */
bool RectSafety(Rect &brect, Size size);

/**
 * @brief     将Rect超出图像边界部分去除
 * @param rect  目标矩形
 * @param rows 图像行数 （高）
 * @param cols 图像列数 （宽）
 * @return  缩放后的Rect
 *
 * @author   李思祁
 * @qq 975216527
 */
bool RectSafety(Rect &brect, int rows, int cols);

/**
 * @brief    计算两点的距离
 * @param pointO
 * @param pointA
 * @return  disance
 */
float getDistance(Point pointO, Point pointA);

float getDistance(Point2f pointO, Point2f pointA);

float getDistance(Point3f pointO, Point3f pointA);


/**
 * @brief    点到直线距离
 * @return   disance
 */
float getDist_P2L(Point pointP, Point pointA, Point pointB);  //求点到直线距离

/**
 * @brief     保持Rect中心不变缩放
 * @param rect  目标矩形
 * @param size 长宽的缩放比例
 * @return   缩放后的Rect
 * @author   李思祁
 */
Rect rectCenterScale(Rect rect, Size2f size); //Rect按中心缩放
/**
 * @brief     最小二乘法
 * @param x  x集合
 * @param y  y集合
 * @return   a,b 直线参数
 */
void LeastSquare(const vector<float> &x, const vector<float> &y, float &a, float &b);

/**
 * @brief     解算抛物线出射角
 * @param ThroughPoint  x集合
 * @param kg重力加速度(cm^2/ms) kv 弹速(cm/ms)
 * @return   解出的两个角度值
 */
Vec2f parabolasolve(Point2f ThroughPoint, float kv, float kg = 0.00098);

/**
 * @brief  空间直角坐标转为极坐标
 * @param rectangular_point  直角坐标
 * @return  极坐标
 */
Polor3f to_polor(const Point3f& rectangular_point);

/**
 * @brief  极坐标转为空间直角坐标
 * @param polor_point  极坐标
 * @return  直角坐标
 */
Point3f to_sprectangular(const Polor3f& polor_point);

/**
 * @brief  旋转矩阵变欧拉角,顺序为roll->yaw->pitch
 * @param R  旋转矩阵
 * @return  欧拉角
 */
Vec3f rotationMatrixToEulerAngles(Mat &R);

/**
 * @brief  欧拉角变旋转矩阵,顺序为roll->yaw->pitch
 * @param eular  欧拉角
 * @param order  顺序true为roll->yaw->pitch,false为pitch->yaw->roll
 * @return  旋转矩阵
 */
Mat eulerAnglesToRotationMatrix(Vec3f &eular, bool order = true);

/**
 * @brief  欧拉角变旋转向量,顺序为roll->yaw->pitch
 * @param eular  欧拉角
 * @return  旋转向量
 */
Mat eulerAnglesToRvec(Vec3f &eular);

/**
 * @brief  变旋转向量欧拉角,顺序为roll->yaw->pitch
 * @param R  旋转向量
 * @return  欧拉角
 */
Vec3f rvecToEulerAngles(Mat &R);

/**
 * @brief  获取程序运行到当前经过的时间
 * @return  时间,单位ms
 */
double get_timenow();

/**
 * @brief  计算两个弧度值的差,定范围到-PI~PI
 * @param a  角度a
 * @param b  角度b
 * @return  角度差
 */
float getDiffangle(float a,float b);

/**
 * @brief  将弧度定范围到-PI~PI
 * @param a  角度a
 * @return  矫正之后的角度(弧度值)
 */
void AngleCorrect(float &a);

/**
 * @brief 查看可用空间
 * return 可用空间大小
 */
int getDiskfreespace();

unsigned char get_find_object();

class BulletSpeedSolve{
public:
    BulletSpeedSolve();
    float solve_speed(float bullet_speed,double current_time);

private:
    vector<float> saved_speed;
    double last_time;
    float last_speed;
};

#endif
