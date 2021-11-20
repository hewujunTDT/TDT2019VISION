
#ifndef T_DT2019VISION_DEBUG_H
#define T_DT2019VISION_DEBUG_H

#include "macro.h"
#include "readparameter.h"
#include "camera.h"
#include "kalman.h"
struct ArmorDebugInformation{
    Mat camtvec=Mat::zeros(Size(3,1),CV_64F);//平移向量
    Mat camrvec=Mat::zeros(Size(3,1),CV_64F);//转换矩阵
    Mat *src;
    Mat *lightbar_thre;
    vector<Armor> armors;
    vector<LightBar> lightbars;
    Rect roi_area;
    RobotTarget target=RobotTarget();
    Polor3f predict_polors_[4]; //四个装甲板的预测位置极坐标
    Polor3f oriented_polor_;
    Polor3f  axis_polor_; //目标轴心位置的极坐标

    Vec3f diffeular={0,0,0};
    Vec3d difftran={0,0,0};

    ReciveMessage reciveMessage;
    SendMessage sendMessage;


    _1KalmanFilter kf_[5];
    Size2f diffsize=Size(0,0);
    Size2f realdiff=Size(0,0);

    double timenow;

    float tmpparam0;
    float tmpparam1;
    float tmpparam2=0;
    float tmpparam3=0;
    float tmpparam4=0;


};

struct BuffDebugInformation{
    Mat camtvec=Mat::zeros(Size(3,1),CV_32F);//平移向量
    Mat camrvec=Mat::zeros(Size(3,1),CV_32F);//转换矩阵
    Mat *src;
    Mat *subcolor_img;
    Mat *binary_img;
    vector<EnergyBuffArmor> buffarmors=vector<EnergyBuffArmor> (5);

    Vec3f diffeular={0,0,0};
    Vec3d difftran={0,0,0};
    ReciveMessage reciveMessage;
    SendMessage sendMessage;
    Point3f predict_point={0,0,0};
    BuffTarget target;

    Mat measure=Mat::zeros(Size(1,1),CV_32F);
    Mat kfpre=Mat::zeros(Size(1,1),CV_32F);
    Mat kfpost=Mat::zeros(Size(1,1),CV_32F);

    Size2f diffsize=Size(0,0);
    Size2f realdiff=Size(0,0);

    double timenow;

    float tmpparam0=0;
    float tmpparam1=0;
    float tmpparam2=0;
    float tmpparam3=0;
    float tmpparam4=0;
    float tmpparam5=0;
    float tmpparam6=0;
    float tmpparam7=0;
    float tmpparam8=0;
};
struct SetVal{
    int val1=180;
    int *lightbar_threval=&val1;
    int val2_split=120;
    int *buff_split_threval=&val2_split;

    int val2_gray=120;
    int *buff_gray_threval=&val2_gray;
    int val3=5;
    int *dilatesize_val=&val3;
    int val4=1;
    int *erodesize_val=&val4;
    float tval0=0,tval1=0;
    float *tmpval0=&tval0;
    float *tmpval1=&tval1;
    Mode detectmode=ArmorMode;

    int cam_val0=2000;
    int *cam_brightness=&cam_val0;

    double cam_val1=-1;
    double *cam_gain=&cam_val1;

    double cam_val2=-1;
    double *cam_gamma=&cam_val2;

    bool  cam_set= false;
    bool WORK= true;
    bool QUIT= false;
};

class Debugger{
public:

    Debugger(Mat camera_matrix,Mat dist_coeffs,GlobalParameter parameter);
    ~Debugger(){;};
    void  set_armor_infomation(ArmorDebugInformation &information);
    void  set_buff_infomation(BuffDebugInformation &information);

    inline SetVal get_setval(){return setval_;}
    inline void set_setval(SetVal &setval){setval_=setval;}
    void draw(ArmorDebugInformation &information);
    void arrange_layout(ArmorDebugInformation &information );

    void draw(BuffDebugInformation &information);
    void arrange_layout(BuffDebugInformation &information );

    /***
  *  @brief     显示
***/
    void show();

    void CorrectAll(const SetVal &setval ,Camera *camera,GlobalParameter &Parameter);
private:
    Mat camera_matrix_;
    Mat dist_coeffs_;

    Mat main_src_;
    Mat ml_images_=Mat::zeros(Size(96,28),CV_8UC3);
    Mat gray_image_;
    Mat binary_image_;
    Mat kalman_image_;
    Mat image_3d_;

    Mat frame;

    bool gray_check= false;
    bool normal_check= true;


    bool targetsolve_check= true;
    bool armordetect_check= true;
    bool ml_check= true;
    bool fire_check= true;
    bool predict_check= true;
    bool kalman_check= false;
    bool page0= true;
    bool page1= false;
    bool page2= false;

    bool tpage0= true;
    bool tpage1= false;
    bool tpage2= false;

    vector<Mat*> tmat;

    bool pausejudge= false;
    int form=0;
    vector <ArmorDebugInformation*> armor_infomations_;
    vector <BuffDebugInformation*> buff_infomations_;
    SetVal setval_;

    Mode lastmode=EnergyBuffMode;

    Point3f worldtocamera(Point3f worldpoint,Vec3f diffeular,Vec3d difftran);

    void clearall();

};

extern ArmorDebugInformation G_armorinformation;
extern BuffDebugInformation G_buffinformation;


extern SetVal G_setval;

#endif //T_DT2019VISION_DEBUG_H
