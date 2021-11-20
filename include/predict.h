
#ifndef T_DT2019VISION_PREDICT_H
#define T_DT2019VISION_PREDICT_H



#include <opencv/cv.hpp>
#include "macro.h"
#include "kalman.h"
using namespace cv;
using namespace std;
class  Predictor {
public:

    Predictor();

};

class  RobotPredictor :Predictor {
public:

    RobotPredictor();

    /**
      * @brief    进行决策并计算要输出的信息(包括重力补偿 角度解算 开火控制)
      * @param    shootplatform  云台
      * @param    message 对message成员变量进行赋值
      * @return   void
      */
    void calc_Output(ReciveMessage const &reciveMessage,SendMessage &sendMessage);

    /***
     *  @brief    获取一个新的target
     *  会结合各个角度更新history_targets_和history_rotators_(是否清空之前targets还是pushback),同时也会更新卡尔曼滤波器KF
     *  @param    target  目标装甲板
     ***/
    void set_new_target(RobotTarget &target);

private:

    /**
     * @brief  预测函数
     * 计算旋转速度,计算轴的位置,对四个装甲板分别进行预测
     * @return   void
     */
    void predict(const ShootPlatform &platform);

    /**
     * @brief 计算旋转速度,更新成员spinning_speed_
     */
    void calc_spinning_speed();

    vector<RobotTarget> history_targets_;//储存历史RobotTarget

    Polor3f predict_polors_[4]; //四个装甲板的预测位置极坐标

    Polor3f current_polors_[4]; //四个装甲板当前位置

    _1KalmanFilter KF_[5]; //kalman滤波器数组

    Polor3f  axis_polor_; //目标轴心位置的极坐标

    float spinning_speed_=0; //旋转速度,单位:弧度值/s

    bool spinningTop_form_= false; //小陀螺模式

    int lastgoodid=0; // 上一次装甲板正对相机时,目标在history_targets_中的index

    int lastbeat_j_; // 上次击打装甲板的index

    double lastbeat_time_; // 最近一次发出beat位=1的时间
};

class  EnergyBuffPredictor :Predictor {

public:
    EnergyBuffPredictor();

    /**
     * @brief    计算要输出的信息(包括重力补偿 角度结算 开火控制)
     * @param    shootplatform  云台
     * @param    message 对message成员变量进行赋值
     * @return   void
     */
    void calc_Output(ReciveMessage const &reciveMessage,SendMessage &sendMessage);

    /***
     *  @brief    获取一个新的target
     *  会结合各个角度更新history_targets_(是否清空之前targets还是pushback),同时也会更新卡尔曼滤波器KF
     *  @param    target  目标装甲板
     ***/
    void set_new_target(BuffTarget &target);

private:

    /**
    * @brief    预测函数 会更新 成员函数predict_rotator_
    * @return   void
    */
    void predict(ReciveMessage const &reciveMessage);

    Point3f PredictTime2Point(const float predict_time,int kr=1);

    void DetaConvert(const double angle, const double deta_x, const double deta_y, double &deta_radial, double &deta_tangent);

    /***
     *  @brief    通过history_targets_判断顺逆时针,会修改clockwise_
     ***/
    void judge_clockwise();

    vector<BuffTarget> history_targets_;//储存历史的BuffTarget;

    Point3f predict_point_;//储存预测的结果;
    Point3f circle_point_;

    bool clockwise_= true;//true顺时针 fasle逆时针
    bool buff_type_ = false;//true大符,false小符


    double last_uptime_;
    float all_uptime_;
    float last_set_time_;

    float last_calc_time_ = 0;
    float last_diff_yaw = 0;
    float last_diff_pitch = 0;
    float predict_point_angle;


    int basePredictTime = 400;  //500 for follow, 400 for static
    int beatStableDelay = 130;
    float maxErrorOnBeat = 5.7; //was 8
    float maxOmegaOnBeat = 0.000000009; //最大允许误差值,越大开火准度越低,开火条件越宽松,越大开火准度越高,开火条件越苛刻
    bool follow_= true;

    double  last_beat_time_;

};

#endif// T_DT2019VISION_PREDICT_H
