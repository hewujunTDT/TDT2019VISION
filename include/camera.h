/*****************************************************************************
 * @brief        调用Hikvision和免驱相机
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

#ifndef T_DT2019VISION_CAMERA_H
#define T_DT2019VISION_CAMERA_H

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include "macro.h"
//海康头文件
#include "MvUsb3VDevice.h"



enum CamType{
    kNormalCam = 0,
    kHikvisionCam = 1,
    kVideoDebug = 2,
};

/**
 * @brief        调用相机的基类，
 *               用于存获取和存储相机基本参数，
 *               不直接调用。
 */

class Camera {
public:
    explicit Camera(int parameter_index);
    ~Camera();
    Color get_enemy_colour()const {
        // lock_guard<mutex> guard(enemy_color_mutex_);//互斥锁，构造上锁析构解锁
        return enemy_color_;
        //return enemy_color_1;
    }
    int get_cam_id() const { return cam_id_; }
    int get_cam_num() const { return cam_online_num_;}
    /**
     * @brief        获取相机类型
     * @return       免驱0，海康1，视频2
     */
    int get_cam_type()const { return cam_type_;}
    int get_parameter_index(){ return parameter_index_; }
    inline int get_width()const{ return width_;}
    inline int get_height()const{ return height_;}
    inline Mat get_matrix()const{ return camera_matrix_;}
    inline Mat get_dist_coeffs()const{ return dist_coeffs_;}
    void SaveRunLog(volatile int data[]);
    static int CameraCalibrate(cv::Mat input);  // 相机标定
    void SetLut(float val,int mode=0);


    virtual bool RestartCam()=0;
    virtual bool get_mat(Mat &img)=0;  // 获取Opencv Mat图像
    virtual bool set_exposure(int t)=0;
    virtual bool set_gain(int val)=0;
    virtual bool set_brightness(int val)=0;  // 置亮度
    virtual bool set_white_balance(int r_or_val,int g,int b)=0;
    virtual bool set_hue(int val)=0;
    virtual bool set_saturation(int val)=0;
    virtual bool set_gamma(float val,int selector=MV_GAMMA_SELECTOR_USER)=0;
    virtual bool set_sharpness(int val)=0;
    virtual bool set_black_level(int val)=0;  // 设置黑位
    virtual int get_cam_bus()=0;
    virtual std::string get_camguid()=0;



protected:
    virtual bool set_format(unsigned int width,
                            unsigned int height,
                            int fps,
                            unsigned int pixelformat)=0;

    static int cam_online_num_;  //在线相机数量
    static int cam_total_num_;  //总申请相机数量
    int cam_id_;
    int cam_type_;
    int parameter_index_;  // 相机使用的参数序号
    int save_res_;  // 录像
    Color enemy_color_;
    Mat camera_matrix_;  // 相机标定的旋转矩阵
    Mat dist_coeffs_;  // 相机标定的畸变系数
    int width_;
    int height_;
    cv::FileStorage fd;
    int frame_index;
    char path[64];
    Mat lut_;
    bool open_lut_= false;
    static std::vector<std::vector<cv::Point2f>> camera_calibrate_points_sequence_;  // 保存多组标定板上角点的三维坐标
};

/**
 * @brief        用于调用普通免驱摄像头
 */
class NormalCam : public Camera {
public:
    explicit NormalCam(unsigned int cam_index,int parameter_index);
    ~NormalCam();

    virtual bool RestartCam();
    virtual bool get_mat(Mat &img);  // 获取Opencv Mat图像

    virtual bool set_exposure(int t);  // 设置曝光
    virtual bool set_gain(int val);  // 置增益
    virtual bool set_brightness(int val);  // 置亮度

    virtual bool set_white_balance(int val,int useless1=0,int useless2=0);  // 设置白平衡(2800-6500)
    bool DoWhiteBalance();  // 单帧自动白平衡
    virtual bool set_hue(int val);  // 设置色调
    virtual bool set_saturation(int val);  // 置饱和度

    bool set_contrast(int val);  // 设置对比度
    virtual bool set_gamma(float val,int useless=0);  // 设置伽玛
    virtual bool set_sharpness(int val);  // 设置锐度
    virtual bool set_black_level(int val){cout<<"NormalCamera:no ability to set blacklevel<<endl";}
    bool set_backlight_compensation(int val);  // 设置背光补偿
    bool set_power_line_frequency(int val);  // 设置防闪烁频率(关:0 50Hz:1 60Hz:2)

    virtual int get_cam_bus() { return cam_bus_;}  // 获取相机在哪一个接口，用于分辨
    virtual std::string get_camguid(){};
    bool get_query_ctrl(__u32 id);  // 获取可调范围，参数如：V4L2_CID_GAIN

private:
    bool InitCam(const char *device, __u32 size_buffer = 4);
    virtual bool set_format(
            unsigned int width,
            unsigned int height,
            int fps,
            __u32 pixelformat = V4L2_PIX_FMT_MJPEG);
    bool StartStream();
    bool CloseStream();
    bool InitMMap();
    int xioctl(int fd, unsigned long request, void *arg);

    // 须安装第三方库libjpeg-turbo
    std::tuple<bool,std::vector<uint8_t>,uint64_t,uint64_t,uint64_t> DecodeJpeg2X(uint8_t* p_jpeg_data,uint64_t jpeg_data_size);
    cv::Mat Jpeg2Mat(uint8_t *jpeg_data, uint64_t jpeg_size);

private:
    typedef struct VideoBuffer{
        void *start;
        size_t length;
    }video_buffer_;
    VideoBuffer* buffers_;
    __u32 buffer_size_ = 4;
    unsigned int buffer_idx_ = 0;
    __u32 pixel_format_ = V4L2_PIX_FMT_MJPEG;
    const char* video_path_;
    int fd_;
    int cam_bus_;
    cv::VideoWriter OutputVideo;
};

/**
 * @brief        用于调用Hikvision摄像头
 */
class HikvisionCam : public Camera {
public:
    explicit HikvisionCam(int nDeviceIndex,int parameter_index);
    ~HikvisionCam();

    virtual bool RestartCam();
    virtual bool get_mat(Mat &img);  // 获取Opencv Mat图像

    virtual bool set_exposure(int t=-1);  // 设置曝光
    bool DoExposure();    // 单帧自动曝光
    virtual bool set_gain(int val=-1);  // 设置增益
    virtual bool set_brightness(int val);  // 设置亮度

    virtual bool set_white_balance(int r=-1,int g=-1,int b=-1);  // 设置白平衡
    bool DoWhiteBalance();  // 单帧自动白平衡
    virtual bool set_hue(int val);  // 设置色调
    virtual bool set_saturation(int val);  // 设置饱和度

    virtual bool set_gamma(float val = 1./2.2,int selector = MV_GAMMA_SELECTOR_SRGB);  // 设置伽玛
    virtual bool set_sharpness(int val);  // 设置锐度
    bool set_black_level(int val);  // 设置黑位 //todo(祁）第一位参数不一样李思祁3.24
    virtual std::string get_camguid() { return cam_guid_;}
    virtual int get_cam_bus(){};

private:
    bool InitCam(int nDeviceIndex);
    virtual bool set_format(unsigned int width,
                            unsigned int height,
                            int fps=-1,
                            unsigned int pixelformat = PixelType_Gvsp_RGB8_Packed);
    bool StartGrabbing();
    bool CloseGrabbing();

private:
    void* handle_ = NULL;
    int n_ret_ = -1;
    unsigned char * p_data_ = NULL;
    int n_data_size_ = 0;
    MV_FRAME_OUT_INFO_EX st_img_info_ = {0};
    int cam_index_ = -1;
    std::string cam_guid_;
    cv::VideoWriter OutputVideo;
};

/**
 * @brief        用于调用视频以调试
 */
class VideoDebug : public Camera {
public:
    VideoDebug(cv::String source,int parameter_index = 0);
    VideoDebug(int num,int parameter_index = 0);
    virtual bool RestartCam(){};
    virtual bool set_exposure(int t){}
    virtual bool set_gain(int val){};
    virtual bool set_brightness(int val){};  // 置亮度
    virtual bool set_white_balance(int r_or_val,int g,int b){};
    virtual bool set_hue(int val){};
    virtual bool set_saturation(int val){};
    virtual bool set_gamma(float val,int selector){};
    virtual bool set_sharpness(int val){};
    virtual int get_cam_bus(){};
    virtual std::string get_camguid(){};
    virtual bool set_black_level(int val){}
    virtual bool get_mat(Mat &img);

private:
    cv::VideoCapture capture_;
    virtual bool set_format(unsigned int width,
                            unsigned int height,
                            int fps,
                            unsigned int pixelformat){};
};

/**
 * @brief    保存录像
 * @author   李思祁
 */
class Videocorder{
public:
    Videocorder(string path);
    void Recorder(Mat src);
    void Recorder(Mat src, struct timeval now_time);
private:
    VideoWriter video_;
    bool init_flag_=true;
    string path_;
    struct timeval start_time_;
};

#endif //T_DT2019VISION_CAMERA_H
