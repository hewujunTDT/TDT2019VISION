#ifndef T_DT2019VISION_TRYENERGYBUFF
#define T_DT2019VISION_TRYENERGYBUFF
#include "opencv2/core.hpp"
#include "macro.h"
#include "camera.h"
#include "armordetect.h"
#include "readparameter.h"
#include "debug.h"
using namespace cv;
class EnergyBuffDetector{
public:
    EnergyBuffDetector(const Camera &camera){
        my_color_=Color(2-camera.get_enemy_colour());
    }
    vector<EnergyBuffArmor> Get(Mat &src);
    inline Point2f set_last_circle_center(const Point2f center){last_circle_center=center;}
    inline Point2f get_last_circle_center(void){ return last_circle_center;}
    inline float set_last_radius(const Point2f circle_center,const Point2f armor_center){last_radius=getDistance(circle_center,armor_center);}
    inline float get_last_radius(void){ return last_radius;}
private:
    Color my_color_;
    int dector_threshold_value_;

    Point2f last_circle_center=Point2f(-1,-1);
    float last_radius=-1;
    float last_armor_area=-1;

    Ptr<KNearest> knn_Model_r = StatModel::load<KNearest>("../model/knn(r3).xml");
    Ptr<KNearest> knn_Model_flowwater = StatModel::load<KNearest>("../model/knn(flowwater3).xml");
    HOGDescriptor *flowwater_hog = new HOGDescriptor(Size(96, 28), Size(32,28), Size(8, 7), Size(16, 14),9);
    HOGDescriptor *r_hog = new HOGDescriptor(Size(28, 28), Size(14, 14), Size(7, 7), Size(7, 7), 9);

    /**
    * @brief   检测装甲版
    * @param   src    输入图像
    * @param   armors 装甲板vector,检测到到的所有armor
    */
    void Detect(vector<EnergyBuffArmor> &armors,const Mat &src);

    /**
    * @brief   判断是不是流水灯
    * @param   src    输入原图
    * @param   buff_armor 输入的装甲版
    * @return  EnergyBuffType //能量机关种类 1是流水灯
    */
    EnergyBuffType FlowWaterLight(EnergyBuffArmor &buff_armor ,const Mat& src);

    /**
    * @brief   找圆心
    * @param   Mat bin  入二值图,以后可能改为轮廓vector;
    * @param   RotatedRec  寻找区域
    * @return Point2f 返回找到的圆心
    */
    Point2f DectorCircle(EnergyBuffArmor &buff_armor, const vector<vector<cv::Point>> &contours ,vector<Vec4i> hierarchy,Mat &src);

    /**
     * @brief   预测time_ms毫秒后装甲版的位置
     * @param buff_armor   装甲板,会赋值其成员变量ml_image_
     * @param src   原图
     * @return  void
     */
    void SetMlRoi(EnergyBuffArmor &buff_armor,const Mat &src);


};

#endif //T_DT2019VISION_TRYENERGYBUFF