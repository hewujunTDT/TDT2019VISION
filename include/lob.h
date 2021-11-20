#ifndef T_DT2019VISION_LOB_H
#define T_DT2019VISION_LOB_H

#include "macro.h"
#include <iostream>
#include "opencv2/core.hpp"
#include "macro.h"
#include "camera.h"
#include "armordetect.h"
#include "readparameter.h"
#include "kalman.h"


/**
 * @brief    桥头吊射（识别基地吊射），敌方桥头
 * @author   黎容熙
 */
enum Lobmode {BRIDGEMODE=0, FREEWAYMODE=1, SELFBRIDGEMODE=3, SPRINGMODE=4};
class BridgeLob{
public:
    BridgeLob(const Camera &camera);
    bool Get(Mat src,ReciveMessage recive_message,SendMessage &send_massage,int where=BRIDGEMODE);
    void Get(ReciveMessage recive_message,SendMessage &send_massage,int where);
//    bool FireCommand();
    Rect get_search_rect(const LightBar &lightbar1, const LightBar &lightbar2);
    bool region_otsu_threshold(const Mat &inputimage, Mat &outputimage, int &thre, int lr);
    void SetMlRoi(Armor &armor,const Mat &src);
    void TnetDL(vector<Armor> &input_armors);

private:
    /**
     * @brief        从图像中央寻找基地顶部装甲板中心像素坐标
     */
    bool BaseDectector(Mat src);
    /**
     * @brief        根据小孔成像算法测距
     */
    void DistanceFinder();
    /**
     * @brief        根据基地顶部装甲板距离和枪口与基地的角度解算出击打顶部三角装甲target_
     */
    void AngleResolver(SendMessage &send_message);
    /**
     * @brief            角度转像素长度
     * @param angle      相机偏移角度
     * @param axis       x 或 y 轴
     * @param cam_matrix 相机内参
     * @return           对应像素长度
     */
    inline double Angle2Pixel(double angle,char axis,Mat cam_matrix){
        if (axis == 'y' || axis == 'Y'){
            return tan(angle) * cam_matrix.at<double>(0,0);
        }else{
            return tan(angle) * cam_matrix.at<double>(1,1);
        }
    }
    /**
     * @brief            像素长度转角度
     * @param angle      像素长度
     * @param axis       x 或 y 轴
     * @param cam_matrix 相机内参
     * @return           对应相机偏移角度
     */
    inline double Pixel2Angle(double pixel,char axis,Mat cam_matrix){
        if (axis == 'y' || axis == 'Y'){
            return atan(pixel/cam_matrix.at<double>(0,0));
        }else{
            return atan(pixel/cam_matrix.at<double>(1,1));
        }
    }

//    inline float GetPointLeght(Point point_a,Point point_b){
//        pow(pow(point_a.x-point_b.x,2)+pow(point_a.y-point_b.y,2),0.5);
//    }
    Rect roi_filter(Mat src,Point last_target);

private:
    Color base_color_;
    ShootPlatform shoot_platform_;
    int wigth_;  // 图像宽
    int height_;  // 图像高
    const int base_height_=913+600;  // mm   //TODO +600
    int bridge_height_=1050;
    const int car_height =420;              //TODO 步兵420 英大327 英小548 新英410
    float bullet_speed_=2.5;  // cm/ms
    Mat cam_matrix_;
    Mat dist_coeffs_;
    Point base_armor_;
    double light_dis_;
    int world_light=225;  // TODO 内灯条225
    double z_distance_;// z轴距离
    double last_distance;
    _1KalmanFilter KF_[1]; //kalman滤波器数组
    int get_target_;
    Point last_target_;
    double last_time_=-100;
    Rect roi_rect_;
};


class AirLob{
public:
    AirLob(const Camera &camera);
    bool Get(Mat src,ReciveMessage recive_message,SendMessage &send_message);

private:
    /**
     * @brief        从图像中央寻找基地顶部装甲板中心像素坐标
     */
    bool BaseDectector(Mat src);
    void DistanceFinder();
    /**
     * @brief        根据基地顶部装甲板距离和枪口与基地的角度解算出击打顶部三角装甲target_
     */
    void AngleResolver(SendMessage &send_message);
    /**
     * @brief            角度转像素长度
     * @param angle      相机偏移角度
     * @param axis       x 或 y 轴
     * @param cam_matrix 相机内参
     * @return           对应像素长度
     */
    inline double Angle2Pixel(double angle,char axis,Mat cam_matrix){
        if (axis == 'y' || axis == 'Y'){
            return tan(angle) * cam_matrix.at<double>(0,0);
        }else{
            return tan(angle) * cam_matrix.at<double>(1,1);
        }
    }
    /**
     * @brief            像素长度转角度
     * @param angle      像素长度
     * @param axis       x 或 y 轴
     * @param cam_matrix 相机内参
     * @return           对应相机偏移角度
     */
    inline double Pixel2Angle(double pixel,char axis,Mat cam_matrix){
        if (axis == 'y' || axis == 'Y'){
            return atan(pixel/cam_matrix.at<double>(0,0));
        }else{
            return atan(pixel/cam_matrix.at<double>(1,1));
        }
    }

    Rect roi_filter(Mat src,Point last_armor);
    vector<vector<Point>> second_detect(Mat src,Rect roi);
private:
    Color base_color_;
    ShootPlatform shoot_platform_;
    int wigth_;  // 图像宽
    int height_;  // 图像高
    int bridge_height_=1050;
    float bullet_speed_=2.6;  // cm/ms
    Mat cam_matrix_;
    Mat dist_coeffs_;
    Point base_armor_;
    double light_dis_;
    int world_light=113;  //113.366
    double z_distance_;// z轴距离
    double last_distance;
    _1KalmanFilter KF_[4]; //kalman滤波器数组
    int get_target_;
    Rect roi_rect;
    Point last_armor_position;
    int height_offset_=-0;
    double last_time_=-100;
    int deviation_=0;
    Point last_base_=Point(720,810);
};



#endif //T_DT2019VISION_LOB_H