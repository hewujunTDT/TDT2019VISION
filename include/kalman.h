

#ifndef T_DT2019VISION_KALMAN_H
#define T_DT2019VISION_KALMAN_H

#include "macro.h"
#include <opencv/cv.hpp>

/**
 * @brief 控制矩阵为1*1的kalman滤波器
 * 继承了opencv类KalmanFilter,为了方便调用,对一些操作进行了封装
 */
class _1KalmanFilter : private KalmanFilter{
public:
    _1KalmanFilter():KalmanFilter(1,1,1){
        resetzero_= false;
        difftime_=50;
        last_time_=0;
    }
    _1KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, float difftime=50,bool resetzero= false)
            :KalmanFilter(dynamParams,measureParams,controlParams){
        setIdentity(transitionMatrix,Scalar::all(1)); // 转移矩阵A[1]
        setIdentity(controlMatrix,Scalar::all(1));    // 控制矩阵A[1]
        setIdentity(measurementMatrix,Scalar::all(1));    // 测量矩阵H
        setIdentity(errorCovPost, Scalar::all(0.3));  // 后验错误估计协方差矩阵P
        this->resetzero_=resetzero;
        this->difftime_=difftime;
        last_time_=get_timenow();
    }
    inline double get_last_time(){ return last_time_;}

    inline void set_processNoiseCov(float pn){
        pn_=pn;
        setIdentity(this->processNoiseCov, Scalar::all(pn)); //系统噪声方差矩阵Q
    }
    inline void set_measurementNoiseCov(float mn){
        mn_=mn;
        setIdentity(this->measurementNoiseCov, Scalar::all(mn)); //系统噪声方差矩阵Q
    }
    inline void set_errorCovPost(float ep){
        setIdentity(this->errorCovPost, Scalar::all(ep)); //后验错误估计协方差矩阵P
    }

    inline float get_statePost() const {
        return statePost_;
    }
    inline float get_measurement() const {
        return mea_;
    }
    inline float get_statePre() const {
        return statePre_;
    }

    inline float get_processNoise() const {
        return pn_;
    }
    inline float get_measurementNoise() const {
        return mn_;
    }
    void correct(float mea,bool continuity= true){
        mea_=mea;
        double currenttime=get_timenow();
        if(float(currenttime-last_time_)>difftime_){
            continuity= false;
        }
        if(continuity){
            KalmanFilter::predict();
            Mat measure=Mat_<float>(1, 1) <<mea;  //观测值
            KalmanFilter::correct(measure);

        } else{
            if(resetzero_){
                setIdentity(this->statePost, Scalar::all(0));                //后验错误估计协方差矩阵P

            } else{
                setIdentity(this->statePost, Scalar::all(mea));                //后验错误估计协方差矩阵P

            }
//            setIdentity(this->errorCovPost, Scalar::all(1));                //后验错误估计协方差矩阵P
        }
        last_time_=currenttime;
        statePost_=statePost.at<float>(0);
        statePre_=statePre.at<float>(0);

    }

private:
    bool resetzero_;
    float difftime_;
    double last_time_;
    float mea_=0;
    float statePost_=0;
    float statePre_=0;
    float pn_=0;
    float mn_=0;


};
#endif //T_DT2019VISION_KALMAN_H
