/*************************************************************************
 * @brief          target resolver
 *                 用于使用pnp算法对目标的 x，y，z 距离的解算
 * @version        1.0.0.1
 * @authors        何武军   王钰深   郭梓楠
 * -----------------------------------------------------------------------
 *   Change Hisitory ：
 *   <Date>     | <Verision> | <Author> |<Descripition>
 * -----------------------------------------------------------------------
 *   2019/02/16 |  1.0.0.1  |   何武军   | 修改宏定义以及代码规范的命名格式
 *
 ************************************************************************/

#ifndef T_DT2019VISION_TARGETRESOLVER_H
#define T_DT2019VISION_TARGETRESOLVER_H

#include "armordetect.h"
#include "macro.h"
#include "opencv2/video/tracking.hpp"
#include<sys/time.h>

extern PointList G_PointList;
class TargetResolver{
public:

    /**
    * @brief  构造函数
    */
    explicit TargetResolver(Camera *camera);

    /**
     * @brief  车体目标解算函数对外接口
     * @param armors  获得的armor类目标,一个目标可能检测到一到两个装甲板所以用vector容器
     * @param Src     获取的源图象
     * @return   output_target  输出Target类的目标
     */
    RobotTarget RobotResolver(vector<Armor> &armors,ReciveMessage &recive_message);

    /**
     * @brief  大神符目标解算函数对外接口
     * @param buff_armor  buff_armor,size为5,第0个是流水的,后边依次是其他的四个大神符装甲板,按顺时针顺序
         * 用多个装甲板去pnp结算可以大大提高结果精准度
     * @return   BuffTarget  输出BuffTarget类的目标
     */
    BuffTarget EnergyBuffResolver(vector<EnergyBuffArmor> &buff_armor,ReciveMessage &recive_message);

private:

    /**
      * @brief   直接得到物体在世界坐标系下的位姿
      * @param &rvec  相机坐标系到物体坐标系的旋转向量 将其更正为 世界坐标系到物体坐标系的旋转向量
      * @param &tvec  相机坐标系到物体坐标系的平移向量 将其更正为 世界坐标系到物体坐标系的平移向量
      * @param shootPlatform  云台
      */
    void AbsoluteCoordinateTransformation(Mat &rvec,Mat &tvec,ShootPlatform shootPlatform,bool to_world= true,int flag=0);//0,装甲版，1，能量机关，2吊射

    /**
      * @brief  对pnpnsolve结果评估
      * @param worldPoint  获得的armor类目标
      * @param imagePoint     获取的源图象
      * @return   float  0-1 越接近1代表 结算结果到图像上的映射与输入点的距离 越接近1越距离越近
      */
    float PnpAssess(Mat matrix,Mat dist_coeffs,Mat rot_matrix,Mat tran_vector,vector<Point3f> worldPoint,vector<Point2f> imagePoint);


    int calc_linkid(vector<Armor> &Armors);


    Mat camera_matrix_;
    Mat dist_coeffs_;

    vector<Armor> last_input_armors_;

};

#endif //T_DT2019VISION_TARGETRESOLVER_H
