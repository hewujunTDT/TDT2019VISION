/*****************************************************************************
 * @brief        装甲版识别
 * @version      1.0.0.1
 *
 * @author
 * @qq
 *
 *----------------------------------------------------------------------------
 * Change History :
 * <Date>     | <Version> | <Author> | <Description>
 *----------------------------------------------------------------------------
 * 2019/02/20 | 1.0.0.1   | 黎容熙 Mark   | 代码规范
 *----------------------------------------------------------------------------
 *
*****************************************************************************/

#ifndef UNTITLED_ARMORDETECT_H
#define UNTITLED_ARMORDETECT_H

#include "mutex"
#include "macro.h"
#include "camera.h"

using std::vector;
using namespace cv;
using namespace cv::ml;

/**
 * @brief        通过调用detecter类获取图片所有装甲版
 *               并决策返回一个需要击打的装甲版
 */
class ArmorDetector {
public:
    explicit ArmorDetector(const Camera &camera);
    ~ArmorDetector();
    /***
  *  @brief     装甲识别调用函数
  *  @input    src  相机输入图像
  *  @input    ReciveMessage  下位机输入信息
  *  @output   Aromr    通过装甲识别的装甲板（如果没有则返回一个空装甲板）
  *  @author   李思祁
  ***/
    vector<Armor> Get(Mat &src,ReciveMessage recive_message);//之后要加入下位机的输入

private:
    Point offset_;
    Rect roi_rect_;
    bool same_armor_;
    Color enemy_color_;

     Armor last_armor_;
private:
    void RoiFilter(Mat &input,Point &offset);
    RobotType Decide(vector<Armor> &armors,ReciveMessage &recive_message);

};

class Detector{
public:
    virtual void Detect(Mat &Src,vector<Armor> &OutputArmors)=0;
protected:
    Color enemy_color_;
    Point offset;
    Detector(Color color,Point offset_);

    /***
 *  @brief    调用机器学习函数
 *  @input    input_armor  装甲板
 *  @input    last_target  历史信息
 *  @input    enemy_color  敌方颜色
 *  @output   Aromr  robot_type  通过机器学习的装甲板和装甲板种类
 *  @author   卢品安
 ***/
    void TnetDL(vector<Armor> &input_armors);
/**
 * @brief 设置机器学习区域
 * @param armor
 * @param src
 */
    void SetMlRoi(Armor &armor,const Mat &src);
/**
 * @brief 判断是否为上一帧目标
 * @param armor
 * @param last_armor
 * @return 与上一帧目标相似度
 */
    float FastTargetLock(Armor &armor,Armor last_armor);
};


//Detecter里的算法函数要有复用性，所以输出输入要明确，内聚性要高
//Detecter的接口函数应有且只有一个Detect（Mat &Src，vector<Armor> OutputArmors）OutoutArmors里的装甲板应经过优先级排序后再输出给机器学习

class LightbarDetector:public Detector {
public:
    LightbarDetector(Color color,Point offset,Rect &roi_area,Armor &last_Armor):Detector(color,offset){
        this->last_Armor_=last_Armor;
        roi_area_ = roi_area;
    }
    virtual void Detect(Mat &src,vector<Armor> &output_armors);
private:
    Rect roi_area_;
    Armor last_Armor_;

private:
    /**
     * @brief    灯条封装
     * @param src 原图
     * @param lightbars 封装好输出的灯条
     * @author 马凯
     */
    void ArmorDetector_GetLightBar(Mat &src,vector<LightBar> &lightbars);
    /**
     * @brief 封装装甲板
     * @param src 原图
     * @param lightbars 输入封装好的灯条
     * @param output_armors 输出封装好的装甲板
     */
    void ArmorDetector_GetArmor(const Mat &src,vector<LightBar> &lightbars,vector<Armor> &output_armors);
    /**
     * @brief 判断装甲板颜色
     * @param lightbar 输入封装好的灯条
     * @param src 原图
     * @return 1:敌人颜色 0：不是敌人颜色
     */
    int JudgeColor(LightBar &lightbar,Mat &src);
    /**
     * @brief 通过二值图来初步判断是否为贴纸
     * @param inputimage
     * @param outputimage
     * @param thre 传出贴纸二值化阈值
     * @param lr 判断左右灯条 左为0,右为1
     * @return 传入图像是否为贴纸
     */
    bool  region_otsu_threshold(const Mat &inputimage, Mat &outputimage ,int &thre,int lr);
    /**
     * @author   郭梓楠
     * @qq       3426445602
     */
    bool  GetEligibility(const LightBar &lightbar1, const LightBar &lightbar2);
    /**
     * @brief 采集两个灯条中间贴纸区域
     * @param lightbar1 一个灯条
     * @param lightbar2 另一个灯条
     * @return search_rect:用于贴纸处理的直矩形
     */
    Rect get_search_rect(const LightBar &lightbar1, const LightBar &lightbar2);
    /**
     * @brief 单灯条贴纸搜寻区域
     * @param lightbar 单灯条
     * @param right_or_left right=1,left=0
     * @return search_rotatedrect:用于贴纸处理的旋转矩形
     */
    RotatedRect get_single_rotatedrect(const LightBar &lightbar,bool right_or_left);



};
/**
 * @brief:       无灯条检测，当目标装甲版被打击导致灯条熄灭的情况下使用。
 * @version      1.0.0.1
 * @author       黄志豪
 */
class NumberStikerDetector:public Detector {
public:
    NumberStikerDetector(Color color,Point &offset,Rect &roi_area,Armor &last_armor):Detector(color,offset)
    {
        enemy_color_=color;
        this->last_armor_=last_armor;
        roi_rect_ = roi_area;
        last_threshold_=last_armor.get_num_threshold();
    }
    /**
     * @brief   在无灯条情况下的贴纸检测
     * @param   src  检测图像，一般是一个感兴趣区域
     * @param   output_armors 输出的armor数组，数组中存了检测出来的贴纸
     * @return  void
     *
     * @author   黄志豪
     * @qq       1780035691
     */
    void Detect(Mat &src,vector<Armor> &output_armors) override;

    Armor last_armor_;
    Rect roi_rect_;
    int last_threshold_;

};





#endif //UNTITLED_ArmorDetector_H
