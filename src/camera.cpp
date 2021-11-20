/*****************************************************************************
 * @brief        调用Hikvision和免驱相机
 * @version      1.0.0.1
 *
 * @author       黎容熙
 *
 *----------------------------------------------------------------------------
 * Change History :
 * <Date>     | <Version> | <Author> | <Description>
 *----------------------------------------------------------------------------
 * 2019/02/16 | 1.0.0.1   | 黎容熙    | 代码规范
 *----------------------------------------------------------------------------
 *
*****************************************************************************/
#include "camera.h"

#include <stdio.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <opencv2/opencv.hpp>
#include <turbojpeg.h>
#include <MvCameraControl.h>
#include <MvGigEDevice.h>
#include "log.h"



#include "readparameter.h"
//成员静态变量初始化
vector<std::vector<cv::Point2f>> Camera::camera_calibrate_points_sequence_;
int Camera::cam_online_num_ = 0;
int Camera::cam_total_num_ = 0;
Camera::Camera(int parameter_index) {
    cam_id_ = cam_total_num_;
    cam_total_num_ ++;
    cam_online_num_ ++;
    parameter_index_ = parameter_index;
    save_res_ = Parameter.cam_parameter[parameter_index].save_res;
    enemy_color_=Color(Parameter.enemy_colour);
    camera_matrix_=Parameter.cam_parameter[parameter_index_].camera_matrix;
    dist_coeffs_=Parameter.cam_parameter[parameter_index_].dist_coeffs;
    width_=Parameter.cam_parameter[parameter_index_].width;
    height_=Parameter.cam_parameter[parameter_index_].height;

    if (Parameter.cam_parameter[parameter_index_].save_runlog){
        if (Parameter.cam_parameter[parameter_index_].video_path
            [Parameter.cam_parameter[parameter_index_].video_path.size()-1] != '/'){
            Parameter.cam_parameter[parameter_index_].video_path += '/';
        }
        sprintf(path,"%s%s%d",Parameter.cam_parameter[parameter_index_].video_path.c_str(),
                Parameter.date,cam_id_);
        char runlog_path[70];
        sprintf(runlog_path,"%s.yaml",path);
        fd.open(runlog_path,FileStorage::APPEND);
        fd<<"online_camera"<<cam_online_num_;
        fd<<"enemy_color"<<enemy_color_;
        fd<<"camera_id"<<cam_id_;
        fd<<"camera_type"<<cam_type_;
//        fd<<"camera_param"<<Parameter.cam_parameter[parameter_index_];
        fd.release();
    }
}
Camera::~Camera() {
    cam_online_num_ --;
    fd.release();
}
void Camera::SaveRunLog(volatile int data[]) {
    timeval time;
    gettimeofday(&time,NULL);
    if (frame_index == 0){
        char runlog_path[70];
        sprintf(runlog_path,"%s.yaml",path);
        fd.open(runlog_path,FileStorage::APPEND);
        fd<<"usart"<<"[";
    }
    fd<<"{";
    fd<<"frame_index"<<frame_index;
    fd<<"data0"<<data[0];
    fd<<"data1"<<data[1];
    fd<<"sec"<<(int)time.tv_sec;
    fd<<"usec"<<(int)time.tv_usec;
    fd<<"}";
    frame_index++;
}
void Camera::SetLut(float val, int mode) {
    if(val<0){
        open_lut_=false;
    }else{
        lut_.create(1,256,CV_8UC1);
        Mat_<uchar> table = lut_;
        switch (mode){
            case 0:
                for (int i = 0; i < 256; i++) {
                    double f;
                    f = (i + 0.5F) / 256;
                    f = pow(f, val);
                    table(0, i) = (f * 256 - 0.5F);
                }
                open_lut_ = true;
                break;
            case 1:
                for (int i = 0; i < val; i++) {

                    table(0,i) = 255 * i / val;
                }
                for (int i = val; i < 256; i++) {

                    table(0,i) = 255;
                }
                open_lut_ = true;
                break;
        }
    }
}
NormalCam::NormalCam(unsigned int cam_index, int parameter_index) : Camera(parameter_index){
    cam_type_ = kNormalCam;
    char path[13];
    sprintf(path,"/dev/video%d",cam_index);
    InitCam(path);
    set_format(Parameter.cam_parameter[parameter_index_].width,Parameter.cam_parameter[parameter_index_].height,Parameter.cam_parameter[parameter_index_].fps);
    StartStream();

    set_exposure(Parameter.cam_parameter[parameter_index_].exposure);
    set_gain(Parameter.cam_parameter[parameter_index_].gain);
    set_brightness(Parameter.cam_parameter[parameter_index_].brightness);
    set_white_balance(Parameter.cam_parameter[parameter_index_].balance_value,0,0);
    set_saturation(Parameter.cam_parameter[parameter_index_].saturation);
    set_contrast(Parameter.cam_parameter[parameter_index_].contrast);
    set_gamma(Parameter.cam_parameter[parameter_index_].gamma);
    set_sharpness(Parameter.cam_parameter[parameter_index_].sharpness);
    if (save_res_ == 1){
        if (Parameter.cam_parameter[parameter_index_].video_path
            [Parameter.cam_parameter[parameter_index_].video_path.size()-1] != '/'){
            Parameter.cam_parameter[parameter_index_].video_path += '/';
        }
        char path[64];
        sprintf(path,"%s%s%d.avi",Parameter.cam_parameter[parameter_index_].video_path.c_str(),
                Parameter.date,cam_id_);
        OutputVideo.open(path, CV_FOURCC('M', 'J', 'P', 'G'), Parameter.cam_parameter[parameter_index_].video_fps,
                         cv::Size(Parameter.cam_parameter[parameter_index_].width,
                                  Parameter.cam_parameter[parameter_index_].height));
    }

}
NormalCam::~NormalCam(){
    CloseStream();
    close(fd_);
    delete [] buffers_;
}
bool NormalCam::InitCam(const char *device, unsigned int size_buffer){
    //打开设备
    video_path_ = device;
    //fd_ = open(video_path,O_RDWR | O_NONBLOCK,0); //用非阻塞模式打开摄像头设备
    fd_ = open(video_path_,O_RDWR,0); //用阻塞模式打开摄像头设备
    if (fd_ < 0){
        printf("[%d] ",cam_id_);
        perror("OPEN DEVICE fail!");
        return false;
    }
    struct v4l2_capability cap;

    if (ioctl(fd_,VIDIOC_QUERYCAP,&cap) < 0){
        printf("[%d] ",cam_id_);
        perror("VIDIOC_QUERYCAP error!");
        return false;
    }
    buffer_size_ = size_buffer;
    cam_bus_ = cap.bus_info[17];
    return true;
}
bool NormalCam::set_format(unsigned int width,
                           unsigned int height,
                           int fps,
                           unsigned int pixelformat){

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if(xioctl(fd_,VIDIOC_S_FMT,&fmt) < 0){
        printf("[%d] ",cam_id_);
        perror("VIDIOC_S_FMT error!");
        return false;
    }
    Parameter.cam_parameter[parameter_index_].width = fmt.fmt.pix.width;
    Parameter.cam_parameter[parameter_index_].height = fmt.fmt.pix.height;
    pixel_format_ = fmt.fmt.pix.pixelformat;
    if (Parameter.cam_parameter[parameter_index_].width != width || Parameter.cam_parameter[parameter_index_].height != height){
        printf("[%d] set pixel width||height fail!\ncurrent value: %d,%d\n",cam_id_,Parameter.cam_parameter[parameter_index_].width,Parameter.cam_parameter[parameter_index_].height);
        return false;
    }
    if (pixel_format_ != pixelformat){
        printf("[%d] set pixel format fail!\ncurrent value: %d\n",cam_id_,pixel_format_);
        return false;
    }


    struct v4l2_streamparm stream_param = {0};
    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_param.parm.capture.timeperframe.denominator = fps;
    stream_param.parm.capture.timeperframe.numerator = 1;
    if (ioctl(fd_, VIDIOC_S_PARM, &stream_param) < 0){
        printf("[%d] ",cam_id_);
        perror("VIDIOC_S_PARM error!");
        return false;
    }
    Parameter.cam_parameter[parameter_index_].fps = stream_param.parm.capture.timeperframe.denominator
                                                    /stream_param.parm.capture.timeperframe.numerator;
    if (Parameter.cam_parameter[parameter_index_].fps != fps){
        printf("set fps fail!\ncurrent value: %f\n",Parameter.cam_parameter[parameter_index_].fps);
        return false;
    }
    return true;
}
bool NormalCam::StartStream(){
    //开始视频流数据的采集
    if(!InitMMap())  return false;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_,VIDIOC_STREAMON,&type) < 0) {
        printf("[%d] ",cam_id_);
        perror("VIDIOC_STREAMON error!");
        return false;
    }
    return true;
}
bool NormalCam::CloseStream(){
    buffer_idx_ = 0;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_,VIDIOC_STREAMOFF,&type) < 0) {
        printf("[%d] ",cam_id_);
        perror("VIDIOC_STREAMOFF error!");
        return false;
    }
    for(int i = 0;i < buffer_size_;i++){
        munmap(buffers_[i].start,buffers_[i].length);
    }
    StartStream();
    return true;
}
bool NormalCam::RestartCam(){
    close(fd_);
    //int fd_ = open(video_path,O_RDWR | O_NONBLOCK,0);
    fd_ = open(video_path_,O_RDWR,0);
    if (fd_ < 0){
        printf("[%d] ",cam_id_);
        perror("open device fail!");
        return false;
    }

    buffer_idx_ = 0;
    return true;
}
bool NormalCam::get_mat(cv::Mat &img){
    //从视频采集输出队列中取出已含有采集数据的帧缓冲区
    struct v4l2_buffer bufferinfo = {0};
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_idx_;
    if(ioctl(fd_, VIDIOC_DQBUF, &bufferinfo) < 0){
        printf("[%d] ",cam_id_);
        perror("VIDIOC_DQBUF error!");
        return false;
    }

    if (pixel_format_ == V4L2_PIX_FMT_MJPEG){
        img = Jpeg2Mat((uint8_t*)buffers_[buffer_idx_].start,buffers_[buffer_idx_].length);
        if(open_lut_){
            LUT(img,lut_,img);
        }
//        Opencv自带解码与海康库不兼容
//        cv::Mat src(Parameter.cam_parameter[parameter_index_].height-300,Parameter.cam_parameter[parameter_index_].width,CV_8UC1,buffers_[buffer_idx_].start);
//        img = cv::imdecode(src, 1);
        if (save_res_ == 1){
            OutputVideo << img;
        }
    }
    else if(pixel_format_ == V4L2_PIX_FMT_YUYV){
        cv::Mat yuyv(Parameter.cam_parameter[parameter_index_].height,Parameter.cam_parameter[parameter_index_].width,CV_8UC2,buffers_[buffer_idx_].start);
        cv::cvtColor(yuyv, img, CV_YUV2BGR_YUYV);
        if(open_lut_){
            LUT(img,lut_,img);
        }
        if (save_res_ == 1){
            OutputVideo << img;
        }
    }
    else{
        printf("pixel format undefined!");
        return false;
    }

    //将该帧缓冲区重新排入输入队列
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_idx_;

    // Queue the next one.
    if(ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0){
        printf("[%d] ",cam_id_);
        perror("VIDIOC_QBUF error!");
        exit(1);
    }
    ++buffer_idx_;
    buffer_idx_ = buffer_idx_ >= buffer_size_ ? buffer_idx_ - buffer_size_ : buffer_idx_;
    return true;
}
bool NormalCam::InitMMap(){
    //申请若干个帧缓冲区
    struct v4l2_requestbuffers bufrequest = {0};
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = buffer_size_;

    if(ioctl(fd_, VIDIOC_REQBUFS, &bufrequest) < 0){
        printf("[%d] ",cam_id_);
        perror("VIDIOC_REQBUFS error!");
        return false;
    }

    buffers_ = (VideoBuffer*)calloc(bufrequest.count, sizeof(*buffers_));
    if (!buffers_){
        printf("[%d] out of memory",cam_id_);
        return false;
    }

    //查询帧缓冲区在内核空间中的长度和偏移量
    for(unsigned int i = 0; i < bufrequest.count; ++i){
        struct v4l2_buffer bufferinfo = {0};
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;

        if(ioctl(fd_, VIDIOC_QUERYBUF, &bufferinfo) < 0){
            printf("[%d] ",cam_id_);
            perror("VIDIOC_QUERYBUF error!");
            return false;
        }

        //通过内存映射将帧缓冲区的地址映射到用户空间
        buffers_[i].length = bufferinfo.length;
        buffers_[i].start = (char*)mmap(
                NULL,
                bufferinfo.length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd_,
                bufferinfo.m.offset);

        if(buffers_[i].start == MAP_FAILED){
            printf("[%d] ",cam_id_);
            perror("MMAP fail!");
            return false;
        }
        memset(buffers_[i].start, 0, bufferinfo.length);

        //将申请到的帧缓冲全部放入视频采集输出队列
        memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;
        if(ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0){
            printf("[%d] ",cam_id_);
            perror("VIDIOC_QBUF error!");
            return false;
        }
    }
    return true;
}
int NormalCam::xioctl(int fd_, unsigned long request, void *arg){
    int r;
    do r = ioctl (fd_, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}
std::tuple<bool,std::vector<uint8_t>,uint64_t,uint64_t,uint64_t> NormalCam::DecodeJpeg2X(uint8_t* p_jpeg_data,uint64_t jpeg_data_size) {
    assert( p_jpeg_data != NULL );
    int width = 0,height = 0,jpegsubsamp = 0;

    tjhandle jpeg = tjInitDecompress();

    if(jpeg == nullptr)
    {
        return std::make_tuple(false, std::vector<uint8_t>(0), 0, 0, 0);
    }

    if(tjDecompressHeader2(jpeg,p_jpeg_data,jpeg_data_size,&width,&height,&jpegsubsamp) != 0)
    {
        return std::make_tuple(false, std::vector<uint8_t>(0), 0, 0, 0);
    }

    TJPF eformat = TJPF::TJPF_BGR;

    uint64_t pitch = tjPixelSize[eformat] * width;
    uint64_t size = pitch * height;
    std::vector<uint8_t> output(size);

    if(tjDecompress2(jpeg,p_jpeg_data,jpeg_data_size,&output.front(),width,pitch,height,eformat,0) != 0)
    {
        return std::make_tuple(false, std::vector<uint8_t>(0), 0, 0, 0);
    }

    return std::make_tuple(true, std::move(output), size, width, height);
}
cv::Mat NormalCam::Jpeg2Mat(uint8_t *jpeg_data, uint64_t jpeg_size) {
    auto res = DecodeJpeg2X( (uint8_t*)jpeg_data,jpeg_size);
    bool success = false;
    std::vector<uint8_t> buff;
    int width,height,size;

    std::tie(success,buff,size,width,height) = res;
    cv::Mat dst(height,width,CV_8UC3,(uint8_t*)&buff.front());
    return dst.clone();
}
bool NormalCam::set_exposure(int t){
    struct v4l2_control ctrl;
    if (t<0){
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_AUTO;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_EXPOSURE_AUTO fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_EXPOSURE_AUTO;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    else{
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_MANUAL;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_EXPOSURE_AUTO fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_EXPOSURE_AUTO;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
        }

        ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        ctrl.value = (__signed__ int)t;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_EXPOSURE_ABSOLUTE fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_gain(int val) {
    struct v4l2_control ctrl;
    if (val<0){
        ctrl.id = V4L2_CID_AUTOGAIN;
        ctrl.value = true;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0)
        {
            printf("[%d] V4L2_CID_AUTOGAIN fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTOGAIN;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    else{
        ctrl.id = V4L2_CID_AUTOGAIN;
        ctrl.value = false;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_AUTOGAIN fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTOGAIN;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }

        ctrl.id = V4L2_CID_GAIN;
        ctrl.value = (__signed__ int)val;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_GAIN fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_GAIN;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_brightness(__signed__ int val){
    struct v4l2_control ctrl;
    if(val<0){
        ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
        ctrl.value = true;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_AUTOBRIGHTNESS fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        } else{
            ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
            ctrl.value = false;
            if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
                printf("[%d] V4L2_CID_AUTOBRIGHTNESS fail!\n",cam_id_);
                struct v4l2_control get_ctrl;
                get_ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
                ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
                printf("current value!: %d\n",get_ctrl.value);
                //return false;
            }

            ctrl.id = V4L2_CID_BRIGHTNESS;
            ctrl.value =val;
            if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
                printf("[%d] V4L2_CID_BRIGHTNESS fail!\n",cam_id_);
                struct v4l2_control get_ctrl;
                get_ctrl.id = V4L2_CID_BRIGHTNESS;
                ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
                printf("current value!: %d\n",get_ctrl.value);
                return false;
            }
        }
    }
}
bool NormalCam::set_white_balance(__signed__ int val,int useless1,int useless2){
    struct v4l2_control ctrl;
    if (val <0){
        ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
        ctrl.value = V4L2_WHITE_BALANCE_AUTO;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_AUTO_WHITE_BALANCE fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    else{
        ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
        ctrl.value = V4L2_WHITE_BALANCE_MANUAL;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_AUTO_WHITE_BALANCE fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
        }

        ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
        ctrl.value = val;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_WHITE_BALANCE_TEMPERATURE fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::DoWhiteBalance(){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_DO_WHITE_BALANCE;
    if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
        printf("[%d] V4L2_CID_DO_WHITE_BALANCE fail!\n",cam_id_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_DO_WHITE_BALANCE;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        printf("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool NormalCam::set_hue(__signed__ int val){
    struct v4l2_control ctrl;
    if(val>=0){
        ctrl.id = V4L2_CID_HUE;
        ctrl.value = val;

        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            printf("[%d] V4L2_CID_HUE fail!\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_HUE;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_saturation(__signed__ int val){
    struct v4l2_control ctrl;
    if(val>=0){
        ctrl.id = V4L2_CID_SATURATION;
        ctrl.value = val;
        if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
            printf("[%d] V4L2_CID_SATURATION\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_SATURATION;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_contrast(__signed__ int val){
    struct v4l2_control ctrl;
    if(val >= 0){
        ctrl.id = V4L2_CID_CONTRAST;
        ctrl.value = val;
        if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
            printf("[%d] V4L2_CID_CONTRAST\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_CONTRAST;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_gamma(float val,int useless){
    struct v4l2_control ctrl;
    if(val >=0){
        ctrl.id = V4L2_CID_GAMMA;
        ctrl.value = (__signed__ int)val;
        if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
            printf("[%d] V4L2_CID_GAMMA\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_GAMMA;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_sharpness(__signed__ int val){
    struct v4l2_control ctrl;
    if(val >=0){
        ctrl.id = V4L2_CID_SHARPNESS;
        ctrl.value = val;
        if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
            printf("[%d] V4L2_CID_SHARPNESS\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_SHARPNESS;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_backlight_compensation(__signed__ int val){
    struct v4l2_control ctrl;
    if(val>=0){
        ctrl.id = V4L2_CID_BACKLIGHT_COMPENSATION;
        ctrl.value = val;
        if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
            printf("[%d] V4L2_CID_BACKLIGHT_COMPENSATION\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_BACKLIGHT_COMPENSATION;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::set_power_line_frequency(__signed__ int val){
    struct v4l2_control ctrl;
    if(val>=0){
        ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
        ctrl.value = val;
        if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
            printf("[%d] V4L2_CID_POWER_LINE_FREQUENCY\n",cam_id_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            printf("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool NormalCam::get_query_ctrl(unsigned int id){
    struct v4l2_queryctrl  Setting;
    Setting.id = id;
    if(xioctl(fd_, VIDIOC_QUERYCTRL, &Setting) < 0){
        printf("[%d] ",cam_id_);
        perror("VIDIOC_QUERYCTRL error!");
        return false;
    }
    printf("max_value:%d\nmin_value:%d\ndefault_value:%d\nstep:%d\n",Setting.maximum,Setting.minimum,Setting.default_value,Setting.step);

    return true;
}
//////////////////////////////HikvisionCam///////////////////////////////
HikvisionCam::HikvisionCam(int nDeviceIndex,int parameter_index) : Camera(parameter_index){
    cam_type_ = kHikvisionCam;
    InitCam(nDeviceIndex);
    set_format(Parameter.cam_parameter[parameter_index_].width,
               Parameter.cam_parameter[parameter_index_].height,
               Parameter.cam_parameter[parameter_index_].fps);
    StartGrabbing();

    set_exposure(Parameter.cam_parameter[parameter_index_].exposure);
    set_gain(Parameter.cam_parameter[parameter_index_].gain);
    set_brightness(Parameter.cam_parameter[parameter_index_].brightness);
    set_white_balance(Parameter.cam_parameter[parameter_index_].balance_ratio_red,Parameter.cam_parameter[parameter_index_].balance_ratio_green,Parameter.cam_parameter[parameter_index_].balance_ratio_blue);
    set_saturation(Parameter.cam_parameter[parameter_index_].saturation);
    set_gamma(Parameter.cam_parameter[parameter_index_].gamma,MV_GAMMA_SELECTOR_USER);
    set_sharpness(Parameter.cam_parameter[parameter_index_].sharpness);
    set_black_level(Parameter.cam_parameter[parameter_index_].black_level);
}
HikvisionCam::~HikvisionCam(){
    CloseGrabbing();
    n_ret_ = MV_CC_DestroyHandle(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_DestroyHandle_ fail! n_ret_ [%x]\n", n_ret_);
    }
}
bool HikvisionCam::InitCam(int nDeviceIndex){
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList);
    if (MV_OK != n_ret_){
        printf("MV_CC_EnumDevices fail [%x]\n", n_ret_);
    }

    if (m_stDevList.nDeviceNum == 0){
        OERROR("未连接相机,数据连接出问题");
        printf("no camera found!\n");
    }

    //选择查找到的一台在线设备，创建设备句柄
    MV_CC_DEVICE_INFO m_stDevInfo = {0};
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));

    n_ret_ = MV_CC_CreateHandle(&handle_, &m_stDevInfo);

    if (MV_OK != n_ret_){
        printf("MV_CC_CreateHandle_ fail [%x]\n", n_ret_);
    }

    cam_index_ = nDeviceIndex;
    cam_guid_ = std::string((char*)m_stDevInfo.SpecialInfo.stUsb3VInfo.chDeviceGUID);

    //连接设备
    unsigned int nAccessMode = MV_ACCESS_Exclusive;
    unsigned short nSwitchoverKey = 0;

    n_ret_ = MV_CC_OpenDevice(handle_, nAccessMode, nSwitchoverKey);
    if (MV_OK != n_ret_){
        printf("MV_CC_OpenDevice fail [%x]\n", n_ret_);
        return false;
    }
}
bool HikvisionCam::set_format(unsigned int width,
                              unsigned int height,
                              int fps,
                              unsigned int pixelformat){
    n_ret_ = MV_CC_SetEnumValue(handle_,"PixelFormat",pixelformat);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetPixelFormat fail! n_ret_ [%x]\n", n_ret_);
        return false;
    }

    n_ret_ = MV_CC_SetIntValue(handle_,"Width",width);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetWidth fail! n_ret_ [%x]\n", n_ret_);
        MVCC_INTVALUE CurrentValue = {0};
        n_ret_ = MV_CC_GetIntValue(handle_,"Width",&CurrentValue);
        Parameter.cam_parameter[parameter_index_].width = CurrentValue.nCurValue;
        printf("current value: %d\n",CurrentValue.nCurValue);
        return false;
    }

    n_ret_ = MV_CC_SetIntValue(handle_,"Height",height);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetHeight fail! n_ret_ [%x]\n", n_ret_);
        MVCC_INTVALUE CurrentValue = {0};
        n_ret_ = MV_CC_GetIntValue(handle_,"Height",&CurrentValue);
        Parameter.cam_parameter[parameter_index_].height = CurrentValue.nCurValue;
        printf("current value: %d\n",CurrentValue.nCurValue);
        return false;
    }

    if (fps <= 0){
        n_ret_ = MV_CC_SetBoolValue(handle_,"AcquisitionFrameRateEnable",false);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetAcquisitionFrameRateEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"AcquisitionFrameRateEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return false;
        }
    }
    else{
        n_ret_ = MV_CC_SetBoolValue(handle_,"AcquisitionFrameRateEnable",true);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetAcquisitionFrameRateEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"AcquisitionFrameRateEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return false;
        }
        n_ret_ = MV_CC_SetFloatValue(handle_,"AcquisitionFrameRate",fps);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetAcquisitionFrameRate fail! n_ret_ [%x]\n", n_ret_);
            MVCC_FLOATVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetFloatValue(handle_,"AcquisitionFrameRate",&CurrentValue);
            Parameter.cam_parameter[parameter_index_].fps = CurrentValue.fCurValue;
            printf("current value: %f\n",CurrentValue.fCurValue);
            return false;
        }
    }
    return true;
}
bool HikvisionCam::StartGrabbing(){
    n_ret_ = MV_CC_StartGrabbing(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_StartGrabbing fail! n_ret_ [%x]\n", n_ret_);
        MV_CC_StopGrabbing(handle_);
        MV_CC_DestroyHandle(handle_);
        return false;
    }

    MVCC_INTVALUE stIntvalue = {0};
    n_ret_ = MV_CC_GetIntValue(handle_, "PayloadSize", &stIntvalue);
    if (n_ret_ != MV_OK){
        printf("MV_CC_GetPayloadSize failed! n_ret_ [%x]\n", n_ret_);
        return false;
    }

    n_data_size_ = stIntvalue.nCurValue + 2048; //一帧数据大小+预留字节(用于SDK内部处理)

    p_data_ =  (unsigned char*)malloc(n_data_size_);


    memset(&st_img_info_, 0, sizeof(MV_FRAME_OUT_INFO_EX));

    return true;
}
bool HikvisionCam::CloseGrabbing(){
    n_ret_ = MV_CC_StopGrabbing(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_StopGrabbing fail! n_ret_ [%x]\n", n_ret_);
        return -1;
    }
    return true;
}
bool HikvisionCam::RestartCam(){
    n_ret_ = MV_CC_DestroyHandle(handle_);
    if (MV_OK != n_ret_){
        printf("MV_CC_DestroyHandle_ fail! n_ret_ [%x]\n", n_ret_);
    }

    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList);
    if (MV_OK != n_ret_){
        printf("MV_CC_EnumDevices fail [%x]\n", n_ret_);
    }

    int i = 0;
    if (m_stDevList.nDeviceNum == 0){
        printf("no camera found!\n");
    }

    //选择查找到的一台在线设备，创建设备句柄
    MV_CC_DEVICE_INFO m_stDevInfo = {0};
    memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[cam_index_], sizeof(MV_CC_DEVICE_INFO));

    n_ret_ = MV_CC_CreateHandle(&handle_, &m_stDevInfo);

    if (MV_OK != n_ret_){
        printf("MV_CC_CreateHandle_ fail [%x]\n", n_ret_);
    }

    //连接设备
    unsigned int nAccessMode = MV_ACCESS_Exclusive;
    unsigned short nSwitchoverKey = 0;

    n_ret_ = MV_CC_OpenDevice(handle_, nAccessMode, nSwitchoverKey);
    if (MV_OK != n_ret_){
        printf("MV_CC_OpenDevice fail [%x]\n", n_ret_);
    }

    StartGrabbing();

    return true;
}
bool HikvisionCam::get_mat(cv::Mat &img){
    /* 旧取帧法 */
    /*n_ret_ = MV_CC_GetImageForBGR(handle_, p_data_, n_data_size_, &st_img_info_, 1000);
    struct timeval get_mat_time;
    gettimeofday(&get_mat_time,NULL);
    cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3, p_data_);//读入图片到frame
    img=frame;//赋给Camera_Output，注意img和frame共享一个矩阵
    if (save_res_ == 1){
        OutputVideo << img;
    }
    return true;*/


    n_ret_ = MV_CC_GetOneFrameTimeout(handle_, p_data_, n_data_size_, &st_img_info_, 1000);
    if (n_ret_ != MV_OK){
        OERROR("相机取帧超时");
        printf("MV_CC_GetOneFrameTimeout failed! n_ret_ [%x]\n", n_ret_);
        return false;
    }

    MV_CC_PIXEL_CONVERT_PARAM StParam = {0};
    memset(&StParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
    //源数据
    StParam.pSrcData       = p_data_;              //原始图像数据
    StParam.nSrcDataLen    = st_img_info_.nFrameLen;         //原始图像数据长度
    StParam.enSrcPixelType = st_img_info_.enPixelType;       //原始图像数据的像素格式
    StParam.nWidth         = st_img_info_.nWidth;            //图像宽
    StParam.nHeight        = st_img_info_.nHeight;           //图像高
    //目标数据pImage
    StParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;     //需要保存的像素格式类型，转换成BGR格式
    StParam.nDstBufferSize = st_img_info_.nWidth*st_img_info_.nHeight*4+2048;             //存储节点的大小
    unsigned char* p_image  = (unsigned char*)malloc(st_img_info_.nWidth*st_img_info_.nHeight*4+2048);
    if(p_image==NULL){
        return false;
    }
    StParam.pDstBuffer     = p_image;                        //输出数据缓冲区，存放转换之后的数据

    n_ret_ = MV_CC_ConvertPixelType(handle_, &StParam);
    if (n_ret_ != MV_OK){
        printf("MV_CC_ConvertPixelType failed! n_ret_ [%x]\n", n_ret_);
        return false;
    }
    cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3, p_image);

    if(open_lut_){
        LUT(frame,lut_,img);
    }else{
        frame.copyTo(img);
    }

    free(p_image);
    p_image = NULL;
//    旧录像
//    if (save_res_ == 1){
//        OutputVideo << img;
//    }
    return true;
}
bool HikvisionCam::set_exposure(int t){
    if (t<0){
        n_ret_ = MV_CC_SetEnumValue(handle_,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"ExposureAuto",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    else{
        n_ret_ = MV_CC_SetEnumValue(handle_,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_OFF);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"ExposureAuto",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }

        n_ret_ = MV_CC_SetFloatValue(handle_,"ExposureTime",t);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetExposureTime fail! n_ret_ [%x]\n", n_ret_);
            MVCC_FLOATVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetFloatValue(handle_,"ExposureTime",&CurrentValue);
            printf("current value: %f\n",CurrentValue.fCurValue);
            return -1;
        }
    }
    return true;
}
bool HikvisionCam::DoExposure(){
    n_ret_ = MV_CC_SetEnumValue(handle_,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_ONCE);
    if (MV_OK != n_ret_){
        printf("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
        MVCC_ENUMVALUE CurrentValue = {0};
        n_ret_ = MV_CC_GetEnumValue(handle_,"ExposureAuto",&CurrentValue);
        printf("current value: %d\n",CurrentValue.nCurValue);
        return -1;
    }
    return true;
}
bool HikvisionCam::set_gain(int val){
    if (val <0){
        n_ret_ = MV_CC_SetEnumValue(handle_,"GainAuto",MV_GAIN_MODE_CONTINUOUS);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetGainAuto fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"GainAuto",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    else{
        n_ret_ = MV_CC_SetEnumValue(handle_,"GainAuto",MV_GAIN_MODE_OFF);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetGainAuto fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"GainAuto",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }

        n_ret_ = MV_CC_SetFloatValue(handle_,"Gain",val);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetGain fail! n_ret_ [%x]\n", n_ret_);
            MVCC_FLOATVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetFloatValue(handle_,"Gain",&CurrentValue);
            printf("current value: %f\n",CurrentValue.fCurValue);
            return -1;
        }
    }
    return true;
}
bool HikvisionCam::set_brightness(int val){
    if(val>=0){
        n_ret_ = MV_CC_SetIntValue(handle_,"Brightness",val);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBrightness fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"Brightness",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    return true;
}
bool HikvisionCam::set_white_balance(int r,int g,int b){ //todo(祁）第一个参数和免驱的相反
    if (r<0||g<0||b<0){
        n_ret_ = MV_CC_SetEnumValue(handle_,"BalanceWhiteAuto",MV_BALANCEWHITE_AUTO_CONTINUOUS);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"BalanceWhiteAuto",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    else{
        n_ret_ = MV_CC_SetEnumValue(handle_,"BalanceWhiteAuto",MV_BALANCEWHITE_AUTO_OFF);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"BalanceWhiteAuto",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }

        n_ret_ = MV_CC_SetEnumValue(handle_,"BalanceRatioSelector",0);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"BalanceRatioSelector",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
        n_ret_ = MV_CC_SetIntValue(handle_,"BalanceRatio",r);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioRed fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"BalanceRatio",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }

        n_ret_ = MV_CC_SetEnumValue(handle_,"BalanceRatioSelector",1);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"BalanceRatioSelector",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
        n_ret_ = MV_CC_SetIntValue(handle_,"BalanceRatio",g);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioGreen fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"BalanceRatio",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }

        n_ret_ = MV_CC_SetEnumValue(handle_,"BalanceRatioSelector",2);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetEnumValue(handle_,"BalanceRatioSelector",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
        n_ret_ = MV_CC_SetIntValue(handle_,"BalanceRatio",b);
        if (MV_OK != n_ret_) {
            printf("MV_CC_SetBalanceRatioBlue fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"BalanceRatio",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    return true;
}
bool HikvisionCam::DoWhiteBalance(){
    n_ret_ = MV_CC_SetEnumValue(handle_,"BalanceWhiteAuto",MV_BALANCEWHITE_AUTO_ONCE);
    if (MV_OK != n_ret_){
        printf("MV_CC_DoBalanceWhite fail! n_ret_ [%x]\n", n_ret_);
        MVCC_ENUMVALUE CurrentValue = {0};
        n_ret_ = MV_CC_GetEnumValue(handle_,"BalanceWhiteAuto",&CurrentValue);
        printf("current value: %d\n",CurrentValue.nCurValue);
        return -1;
    }
    return true;
}
bool HikvisionCam::set_hue(int val){
    if(val<0){
        n_ret_ = MV_CC_SetBoolValue(handle_,"HueEnable",false);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetHueEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"HueEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
    }else{
        n_ret_ = MV_CC_SetBoolValue(handle_,"HueEnable",true);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetHueEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"HueEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
        n_ret_ = MV_CC_SetIntValue(handle_,"Hue",val);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetHue fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"Hue",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    return true;
}
bool HikvisionCam::set_saturation(int val){
    if(val<0){
        n_ret_ = MV_CC_SetBoolValue(handle_,"SaturationEnable",false);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetSaturationEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"SaturationEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
    }else{
        n_ret_ = MV_CC_SetBoolValue(handle_,"SaturationEnable",false);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetSaturationEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"SaturationEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
        n_ret_ = MV_CC_SetIntValue(handle_,"Saturation",val);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetSaturation fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"Saturation",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    return true;
}
bool HikvisionCam::set_gamma(float val,int selector){
    MVCC_ENUMVALUE CurrentValue = {0};
    n_ret_ = MV_CC_GetEnumValue(handle_,"PixelFormat",&CurrentValue);
    if(CurrentValue.nCurValue==PixelType_Gvsp_BayerRG8){
        SetLut(val);
    }else{
        if(selector == MV_GAMMA_SELECTOR_SRGB){
            n_ret_ = MV_CC_SetBoolValue(handle_,"GammaEnable", true);
            if (MV_OK != n_ret_){
                printf("MV_CC_SetGammaEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_,"GammaEnable",&CurrentValue);
                printf("current value: %d\n",CurrentValue);
                return -1;
            }
            n_ret_ = MV_CC_SetEnumValue(handle_,"GammaSelector",MV_GAMMA_SELECTOR_SRGB);
            if (MV_OK != n_ret_){
                printf("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
                MVCC_ENUMVALUE CurrentValue = {0};
                n_ret_ = MV_CC_GetEnumValue(handle_,"GammaSelector",&CurrentValue);
                printf("current value: %d\n",CurrentValue.nCurValue);
                return -1;
            }
        }else{
            if(val<0){
                n_ret_ = MV_CC_SetBoolValue(handle_,"GammaEnable",false);
                if (MV_OK != n_ret_){
                    printf("MV_CC_SetGammaEnable fail! n_ret_ [%x]\n", n_ret_);
                    bool CurrentValue;
                    n_ret_ = MV_CC_GetBoolValue(handle_,"GammaEnable",&CurrentValue);
                    printf("current value: %d\n",CurrentValue);
                    return -1;
                }
            }else{
                n_ret_ = MV_CC_SetBoolValue(handle_,"GammaEnable", true);
                if (MV_OK != n_ret_){
                    printf("MV_CC_SetGammaEnable fail! n_ret_ [%x]\n", n_ret_);
                    bool CurrentValue;
                    n_ret_ = MV_CC_GetBoolValue(handle_,"GammaEnable",&CurrentValue);
                    printf("current value: %d\n",CurrentValue);
                    return -1;
                }
                n_ret_ = MV_CC_SetEnumValue(handle_,"GammaSelector",MV_GAMMA_SELECTOR_USER);
                if (MV_OK != n_ret_){
                    printf("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
                    MVCC_ENUMVALUE CurrentValue = {0};
                    n_ret_ = MV_CC_GetEnumValue(handle_,"GammaSelector",&CurrentValue);
                    printf("current value: %d\n",CurrentValue.nCurValue);
                    return -1;
                }
                n_ret_ = MV_CC_SetFloatValue(handle_,"Gamma",val);
                if (MV_OK != n_ret_){
                    printf("MV_CC_SetGamma fail! n_ret_ [%x]\n", n_ret_);
                    MVCC_FLOATVALUE CurrentValue = {0};
                    n_ret_ = MV_CC_GetFloatValue(handle_,"Gamma",&CurrentValue);
                    printf("current value: %f\n",CurrentValue.fCurValue);
                    return -1;
                }
            }
        }
    }
    return true;
}
bool HikvisionCam::set_sharpness(int val){
    if(val<0){
        n_ret_ = MV_CC_SetBoolValue(handle_,"SharpnessEnable",false);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetSharpnessEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"SharpnessEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
    }else{
        n_ret_ = MV_CC_SetBoolValue(handle_,"SharpnessEnable",true);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetSharpnessEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"SharpnessEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
        n_ret_ = MV_CC_SetIntValue(handle_,"Sharpness",val);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetSharpness fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"Sharpness",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    return true;
}
bool HikvisionCam::set_black_level(int val){
    if(val<0){
        n_ret_ = MV_CC_SetBoolValue(handle_,"BlackLevelEnable",false);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBlackLevelEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"BlackLevelEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
    }
    else{
        n_ret_ = MV_CC_SetBoolValue(handle_,"BlackLevelEnable",true);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBlackLevelEnable fail! n_ret_ [%x]\n", n_ret_);
            bool CurrentValue;
            n_ret_ = MV_CC_GetBoolValue(handle_,"BlackLevelEnable",&CurrentValue);
            printf("current value: %d\n",CurrentValue);
            return -1;
        }
        n_ret_ = MV_CC_SetIntValue(handle_,"BlackLevel",val);
        if (MV_OK != n_ret_){
            printf("MV_CC_SetBlackLevel fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_ = MV_CC_GetIntValue(handle_,"BlackLevel",&CurrentValue);
            printf("current value: %d\n",CurrentValue.nCurValue);
            return -1;
        }
    }
    return true;
}

/////////////////////VideoDebug/////////////////
VideoDebug::VideoDebug(cv::String source,int parameter_index) : Camera(parameter_index){
    cam_type_ = kVideoDebug;
    if(!capture_.open(source)){
        char path[25];
        sprintf(path,"../videodebug/%s.avi",source.c_str());
        if(!capture_.open(path)){
            sprintf(path,"../videodebug/%s",source.c_str());
            if(!capture_.open(path)){
                printf("[%d] open video file fail!\n",cam_id_);
            }
        }
    }
}
VideoDebug::VideoDebug(int num,int parameter_index) : Camera(parameter_index){
    cam_type_ = kVideoDebug;
    char path[25];
    sprintf(path,"../videodebug/%d.avi",num);
    if(!capture_.open(path)){
        printf("[%d] open video file fail!\n",cam_id_);
    }
}
bool VideoDebug::get_mat(cv::Mat &img) {
    capture_ >> img;
    if(open_lut_){
        LUT(img,lut_,img);
    }
    return true;
}
void MouseHandle(int event, int x, int y, int flags, void * param){
    if (event == EVENT_LBUTTONDOWN){
        *(int*)param=1;
    }else if(event == EVENT_RBUTTONDOWN){
        *(int*)param=0;
    }
}
//相机标定
int Camera::CameraCalibrate(cv::Mat input)
{
    Size board_size = Size(11,8);
    static vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
    static bool calibrate_flag= false;
    static bool init_flag=true;

    if (input.empty())
        return 0;
    int mouse_flag=-1;
    if(init_flag==true){
        namedWindow("Calibrate");
        setMouseCallback("Calibrate",MouseHandle,&mouse_flag);
        init_flag==false;
    }
    if(camera_calibrate_points_sequence_.size()<15) {
        if (!calibrate_flag) {
            Mat show;
            input.copyTo(show);
            putText( show, "click mouse left ",Point(100,100),0,1.5,Scalar(255,255,255),1);
            putText( show, "to find "+to_string(camera_calibrate_points_sequence_.size()+1)+"of15 Chessboard Corners",Point(100,150),0,1.5,Scalar(255,255,255),1);
            putText( show, "Warning:Make sure whole chessboard in view",Point(100,200),0,1,Scalar(0,0,255),2);
            putText( show, "or it will crash",Point(100,250),0,1,Scalar(0,0,255),2);
            imshow("Calibrate", show);
            waitKey(200);
            if (mouse_flag == 1) {
                Mat view_gray;
                cvtColor(input, view_gray, CV_RGB2GRAY);
                Mat show;
                input.copyTo(show);
                putText( show, "finding...",Point(input.size()/2),0,2,Scalar(0,255,0),2);
                putText( show, "warning :If it crash, Please wait 30 seconds",Point(input.size()/5),0,1,Scalar(0,0,255),1);
                imshow("Calibrate", show);
                waitKey(1);
                if (findChessboardCorners(input, board_size, image_points_buf)){
                    find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5)); //对粗提取的角点进行精确化
                    calibrate_flag= true;

                    drawChessboardCorners(input, board_size, image_points_buf, false); //在图片中标记角点
                    putText(input, to_string(camera_calibrate_points_sequence_.size()+1)+"of15",Point(input.size().width/3,100),0,1.5,Scalar(255,255,255),2);
                    putText(input, "whether to save these ChessboardCorners?",Point(input.size().width/3,200),0,1,Scalar(255,255,255),2);
                    putText( input, "mouse left:save",Point(0,input.size().height/2),0,1.5,Scalar(0,255,0),1);
                    putText( input, "mouse right:delete ",Point(input.size().width*2/3,input.size().height/2),0,1.5,Scalar(255,0,0),1);
                    imshow("Calibrate", input);
                }
            }
            waitKey(1);
        } else {
            waitKey(500);
            if (mouse_flag == 1) {
                camera_calibrate_points_sequence_.push_back(image_points_buf);  //保存亚像素角点
                std::cout << "第" << camera_calibrate_points_sequence_.size() << "张标定图" << std::endl;
                calibrate_flag= false;
            } else if(mouse_flag==0){
                calibrate_flag= false;
            }
        }
    }
    else
    {
        destroyWindow("Calibrate");
        std::cout << "开始标定………………";
        /*棋盘三维信息*/
        cv::Size square_size = cv::Size(2, 2);  /* 实际测量得到的标定板上每个棋盘格的大小 厘米*/
        std::vector<std::vector<cv::Point3f> > object_points; /* 保存标定板上角点的三维坐标 */
        cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
        int point_counts=board_size.width*board_size.height;  // 每幅图像中角点的数量
        cv::Mat dist_coeffs = cv::Mat(5, 1, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
        std::vector<cv::Mat> tvecs_mat;  /* 每幅图像的旋转向量 */
        std::vector<cv::Mat> rvecs_mat; /* 每幅图像的平移向量 */
        /* 初始化标定板上角点的三维坐标 */
        int i, j, t;
        for (t = 0; t < camera_calibrate_points_sequence_.size(); t++)
        {
            std::vector<cv::Point3f> temp_point_set;
            for (i = 0; i < board_size.height; i++)
            {
                for (j = 0; j < board_size.width; j++)
                {
                    cv::Point3f realPoint;
                    /* 假设标定板放在世界坐标系中z=0的平面上 */
                    realPoint.x = i * square_size.width;
                    realPoint.y = j * square_size.height;
                    realPoint.z = 0;
                    temp_point_set.push_back(realPoint);
                }
            }
            object_points.push_back(temp_point_set);
        }

        /* 开始标定 */
        calibrateCamera(object_points, camera_calibrate_points_sequence_, input.size(), camera_matrix, dist_coeffs, rvecs_mat, tvecs_mat, 0);

        std::cout<<camera_matrix<<std::endl;
        std::cout<<dist_coeffs<<std::endl;
        std::cout<<"标定完成！\n";

        //对标定结果进行评价
        std::cout<<"开始评价标定结果………………\n";
        double total_err = 0.0; /* 所有图像的平均误差的总和 */
        double err = 0.0; /* 每幅图像的平均误差 */
        std::vector<cv::Point2f> image_points2; /* 保存重新计算得到的投影点 */
        std::cout<<"\t每幅图像的标定误差：\n";


        for (int i=0;i<camera_calibrate_points_sequence_.size();i++)
        {
            std::vector<cv::Point3f> tempPointSet=object_points[i];
            /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */

            projectPoints(tempPointSet,rvecs_mat[i],tvecs_mat[i],camera_matrix,dist_coeffs,image_points2);

            /* 计算新的投影点和旧的投影点之间的误差*/
            std::vector<cv::Point2f> tempImagePoint = camera_calibrate_points_sequence_[i];
            cv::Mat temp_image_point_mat = cv::Mat(1,tempImagePoint.size(),CV_32FC2);
            cv::Mat image_points2mat = cv::Mat(1,image_points2.size(), CV_32FC2);
            for (int j = 0 ; j < tempImagePoint.size(); j++)
            {
                image_points2mat.at<cv::Vec2f>(0,j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
                temp_image_point_mat.at<cv::Vec2f>(0,j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
            }
            std::cout<<"flag"<<std::endl;
            err = norm(image_points2mat, temp_image_point_mat, cv::NORM_L2);
            total_err += err/=  point_counts;
            std::cout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<std::endl;
        }
        std::cout<< "总体平均误差：" << total_err /camera_calibrate_points_sequence_.size()<<"像素"<<std::endl;
        std::cout<<"评价完成！"<<std::endl;

        std::string file_address="../config/cam_internal.yaml";
        cv::FileStorage fs(file_address,cv::FileStorage::APPEND);
        fs<<"matrix"<<camera_matrix;
        fs<<"dist_coeffs"<<dist_coeffs;
        fs.release();

        return 1;
    }
    return 0;
}

////////////////////////Videocorder////////////////
Videocorder::Videocorder(string path){
    path_=path;//todo(祁）判断路径文件夹是否存在不存在创建文件夹 3.30
    gettimeofday(&start_time_,NULL);
}
void Videocorder::Recorder(Mat src){
    if(init_flag_){
        char path[64];
        sprintf(path,"%s%s.avi",path_.c_str(),
                Parameter.date);
        video_.open(path, CV_FOURCC('M', 'J', 'P', 'G'), 50, Size(src.size()));
        video_ << src;
        init_flag_= false;
    }else{
        video_ << src;
    }
}
void Videocorder::Recorder(Mat src, struct timeval now_time){

    if(init_flag_){
        char path[64];
        sprintf(path,"%s%s.avi",path_.c_str(),
                Parameter.date);
        video_.open(path, CV_FOURCC('M', 'J', 'P', 'G'), 50, Size(src.size()));
        cout<<"src.size()=="<<src.size()<<endl;
        video_ << src;
        init_flag_= false;

    }else{
        static int i;
        i++;
        if(!(i%4)) {
            double time = (now_time.tv_sec - start_time_.tv_sec) + (now_time.tv_usec - start_time_.tv_usec) / 1000000.0;
            putText(src, to_string(time), Point(50, 50), 1, 1, Scalar(255, 255, 255));
            video_ << src;
            i=0;
        }
    }
}

