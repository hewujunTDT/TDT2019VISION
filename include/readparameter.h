/*****************************************************************************
 * @brief        从 .yaml 文件读取参数
 * @version      1.0.0.1
 *
 * @author       黎容熙
 * @qq           919935013
 *
 *----------------------------------------------------------------------------
 * Change History :
 * <Date>     | <Version> | <Author> | <Description>
 *----------------------------------------------------------------------------
 * 2019/02/16 | 1.0.0.1   | 黎容熙    | 代码规范
 *----------------------------------------------------------------------------
 *
*****************************************************************************/

#ifndef T_DT2019VISION_READ_PARAMETER_H
#define T_DT2019VISION_READ_PARAMETER_H

#include <vector>
#include <opencv2/opencv.hpp>


/**
 * @brief        相机参数结构体
 */
typedef struct CamParameter
{
    float fps;
    int width;
    int height;
    int exposure;
    int red_exposure;
    int blue_exposure;
    int gain;
    int brightness;
    int saturation;
    int contrast;
    float gamma;
    int sharpness;
    int black_level;
    int balance_value;
    int balance_ratio_red;
    int balance_ratio_green;
    int balance_ratio_blue;
    cv::Mat camera_matrix=cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
    cv::Mat dist_coeffs=cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    int save_res;
    int save_runlog;
    int video_fps;
    cv::String video_path;
}CamParameter;


/**
 * @brief        全局参数结构体
 */
typedef struct GlobalParameter
{
    int enemy_colour;
    int firing_rate;
    float gun_height;
    double pitch_k;
    int ArmorDetect_lightbarthre;
    int EnergyBuffDetect_gray_thre;
    int EnergyBuffDetect_split_thre;
    int EnergyBuffDetect_dialesize;
    int EnergyBuffDetect_erodesize;
    bool DebugWork;
    bool VideoRecordWork;
    double lob_yaw_offset;
    double lob_pitch_offset;
    std::vector<CamParameter> cam_parameter;
    char date[15];  // 日期，用于设文件名
}Struct_Parameter;


// 声明而不定义一个参数结构体
extern  Struct_Parameter Parameter;

/**
 * @brief        运行程序初始化时读取参数文件
 */
class ReadParameter
{
public:
    ReadParameter();
    static void WriteParameter();
};



#endif //T_DT2019VISION_READ_PARAMETER_H
