#include <macro.h>
#include "readparameter.h"
//#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
//#include <stdio.h>
#include <stdlib.h>
//#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/stat.h>

////////////////////////////// LightBar /////////////////////////////////

LightBar::LightBar(RotatedRect rotrect, Color color) {
    empty_=false;
    color_=color;
    rotated_rect_=rotrect;
    center_ = rotated_rect_.center;
    area_=static_cast<int>(rotated_rect_.size.area());
    if(fabs(rotated_rect_.angle)<45){
        angle_ = rotated_rect_.angle;
        width_ = static_cast<int>(rotated_rect_.size.width);
        height_ = static_cast<int>(rotated_rect_.size.height);
        center_ = rotated_rect_.center;
        Point2f vertices[4];
        rotated_rect_.points(vertices);
        bl_ = vertices[0];
        tl_ = vertices[1];
        tr_ = vertices[2];
        br_ = vertices[3];
    } else{
        angle_ = 90 + rotated_rect_.angle;
        width_ = static_cast<int>(rotated_rect_.size.height);
        height_ = static_cast<int>(rotated_rect_.size.width);
        Point2f vertices[4];
        rotated_rect_.points(vertices);
        center_ = rotated_rect_.center;
        br_ = vertices[0];
        bl_ = vertices[1];
        tl_ = vertices[2];
        tr_ = vertices[3];
    }
    ratio_width_to_height_=width_/height_;
    judge_search_=0;
    bounding_rect_ = rotated_rect_.boundingRect();
}
LightBar::LightBar() {
    empty_=true;
    angle_ = 0;
    width_ = 0;
    height_ = 0;
    judge_search_=0;
    area_=0;
    angle_=0;
    center_=Point(0,0);
    tl_=Point2f(0,0);
    tr_=Point2f(0,0);
    bl_=Point2f(0,0);
    br_=Point2f(0,0);

    bounding_rect_=Rect(Point(0,0),Point(0,0));
    rotated_rect_=RotatedRect(Point2f(0,0),Size2f(0,0),0);
    color_=UNKNOW;
    judge_search_=0;
    ratio_width_to_height_=0;

}
LightBar::~LightBar() = default;

////////////////////////////// NumberStiker /////////////////////////////////

NumberStiker::NumberStiker(RotatedRect &rotated_rect){

    empty_= false;
    rotated_rect_=rotated_rect;
    bounding_rect_=rotated_rect.boundingRect();
    center_=rotated_rect_.center;
    if(fabs(rotated_rect_.angle)<45){
        angle_ = rotated_rect_.angle;
        width_ = static_cast<int>(rotated_rect_.size.width);
        height_ = static_cast<int>(rotated_rect_.size.height);
        center_ = rotated_rect_.center;
        Point2f vertices[4];
        rotated_rect_.points(vertices);
        bl_ = vertices[0];
        tl_ = vertices[1];
        tr_ = vertices[2];
        br_ = vertices[3];
    } else{
        angle_ = 90 + rotated_rect_.angle;
        width_ = static_cast<int>(rotated_rect_.size.height);
        height_ = static_cast<int>(rotated_rect_.size.width);
        Point2f vertices[4];
        rotated_rect_.points(vertices);
        center_ = rotated_rect_.center;
        br_ = vertices[0];
        bl_ = vertices[1];
        tl_ = vertices[2];
        tr_ = vertices[3];
    }
}
NumberStiker::NumberStiker() {
    empty_=true;
    bounding_rect_=Rect(0,0,0,0);
    center_=Point(0,0);
    width_=0;
    height_=0;
}
NumberStiker::~NumberStiker() = default;

////////////////////////////// Armor /////////////////////////////////

Armor::Armor() {
    form_[0]=form_[1]=form_[2]=false;
    aromor_type_=Empty;
    empty_=true;
    armor_rect_=Rect();
    robot_type_=TYPEUNKNOW;//机器人种类
    armor_angle_=0;//装甲板角度
    armor_rotatedrect_=RotatedRect();//装甲板的旋转矩形
    numberstiker_=NumberStiker();
    left_lightbar_=LightBar();
    right_lightbar_=LightBar();
    distance_to_screen_center=0;
    ml_roi_=Mat();
    num_threshold_=50;
}
Armor::Armor(NumberStiker numberstiker){
    numberstiker_=numberstiker;
    form_[2] = true;
    form_[0]=form_[1]=false;
    empty_= false;
    aromor_type_=NoLightbar;
    robot_type_=TYPEUNKNOW;
    armor_rotatedrect_=numberstiker.get_rotated_rect();
    armor_rect_=numberstiker_.get_rotated_rect().boundingRect();
    num_threshold_=0;

}
Armor::Armor(NumberStiker numberstiker,LightBar single_lightbar) {
    form_[2]=true;
    aromor_type_=Lightbar;
    empty_=false;
    numberstiker_=numberstiker;
    robot_type_=TYPEUNKNOW;
    num_threshold_=0;
    if(single_lightbar.get_center().x<numberstiker.get_center().x){
        form_[0]=true;form_[1]= false;
        left_lightbar_=single_lightbar;
    }else {
        form_[1]=true;form_[0]= false;
        right_lightbar_=single_lightbar;
    }
    if(!numberstiker_.get_rotated_rect().size.empty())
        armor_rect_=numberstiker_.get_rotated_rect().boundingRect()
                    |single_lightbar.get_bounding_rect();
}
Armor::Armor(NumberStiker numberstiker,LightBar left_lightbar,LightBar right_lightbar){
    form_[0]=true;
    form_[1]=true;
    form_[2]=true;
    aromor_type_=Lightbar;
    robot_type_=TYPEUNKNOW;
    empty_=false;
    numberstiker_=numberstiker;
    num_threshold_=0;
    if(right_lightbar.get_center().x>left_lightbar.get_center().x){
        right_lightbar_=right_lightbar;
        left_lightbar_=left_lightbar;
    } else{
        right_lightbar_=left_lightbar;
        left_lightbar_=right_lightbar;
    }

    //之后进行扩展对不同的输入情况采用不同方法确定装板矩形框
    if(!numberstiker_.get_bounding_rect().empty())
        armor_rect_=numberstiker_.get_bounding_rect()
                    |left_lightbar_.get_bounding_rect()
                    |right_lightbar_.get_bounding_rect();

    else if(!numberstiker_.get_rotated_rect().size.empty())
        armor_rect_=numberstiker_.get_rotated_rect().boundingRect()
                    |left_lightbar_.get_bounding_rect()
                    |right_lightbar_.get_bounding_rect();

}
Armor::~Armor() = default;
/**********************************
 * @函数名 get_armor_angle
 * @简介
 * @传入参数
 * @传出参数
 **********************************/
float Armor::get_armor_angle() {
    float armor_angle ;
    if(form_[0]||form_[1]) {

        if (left_lightbar_.get_height() > right_lightbar_.get_height()) {
            armor_angle = left_lightbar_.get_angle();
        } else {
            armor_angle = right_lightbar_.get_angle();
        }
    } else{
        armor_angle=numberstiker_.get_angle();
    }
    return armor_angle;
}
int Armor::get_distance_to_screen_center(){       //根据勾股定理算出装甲板中心到屏幕中心的距离
    int dis_x = cvRound(armor_rect_.x+armor_rect_.width/2-Parameter.cam_parameter[0].width/2);
    int dis_y = cvRound(armor_rect_.y+armor_rect_.height/2-Parameter.cam_parameter[0].height/2);
    distance_to_screen_center = dis_x*dis_x+dis_y*dis_y;
    return distance_to_screen_center;
}
RotatedRect  Armor::get_armor_rotatedrect() {
    RotatedRect armor_plate;
    if(form_[2]) {
        if(form_[0]&&form_[1]){
            Point left_lightbar_center = left_lightbar_.get_center();
            Point right_lightbar_center = right_lightbar_.get_center();
            distance = getDistance(left_lightbar_center, right_lightbar_center);//两个矩形框的距离
            LightBar better_lightbar = left_lightbar_.get_height() > right_lightbar_.get_height() ?
                                       left_lightbar_:right_lightbar_;
            Size size;
            size.height = max(cvRound(better_lightbar.get_height() * 2.2),cvRound(numberstiker_.get_height()*1.1));
            size.width = min(cvRound(distance - 1.8* better_lightbar.get_width()),cvRound(size.height/1.4));
            size.width = max(size.width,cvRound(numberstiker_.get_width()*1.05));
            float angle = better_lightbar.get_rotatedrect().angle;
            if(angle<-45){
                size=Size(size.height,size.width);
            }
            Point center = (left_lightbar_center + right_lightbar_center) / 2;
            armor_plate= RotatedRect (center, size, angle);

        } else if(form_[0]||form_[1]) {
            LightBar better_lightbar = form_[1] ?
                                       right_lightbar_ : left_lightbar_;
            Size size;
            Point center =numberstiker_.get_center();
            distance = 2*getDist_P2L(center,(better_lightbar.tl()+better_lightbar.tr())/2,
                                     better_lightbar.get_center());//两个矩形框的距离
            size.height = max(cvRound(better_lightbar.get_height() * 2.2),cvRound(numberstiker_.get_height()*1.1));
            size.width = min(cvRound(distance - 1.8 * better_lightbar.get_width()),cvRound(size.height/1.4));
            size.width = max(size.width,cvRound(numberstiker_.get_width()*1.05));
            float angle = better_lightbar.get_rotatedrect().angle;
            if(angle<-45){
                size=Size(size.height,size.width);
            }
            armor_plate= RotatedRect (center, size, angle);
        }
        else{

            armor_plate= numberstiker_.get_rotated_rect();
            armor_plate.size.width*=1.1;
            armor_plate.size.height*=1.1;
        }
    }


    return armor_plate;
}
void Armor::calc_pnppoints(vector<Point2f> &pnp_points){
    //从左到右依次是 左灯条{0上定点,1下顶点,2中心点} 数字贴纸{3左中心点,4右中心点,5上中心点,6下中心点,7中心点} 右灯条{8上定点,9下顶点,10中心点}
    pnp_points=vector<Point2f>(11,Point2f(-1,-1));
    if(form_[0]&&form_[1]&&form_[2]){
        pnp_points[5]=(numberstiker_.tl()+numberstiker_.tr())/2;
        pnp_points[6]=(numberstiker_.bl()+numberstiker_.br())/2;
        pnp_points[7]=numberstiker_.get_center();
        pnp_points[1]=(left_lightbar_.bl()+left_lightbar_.br())/2;
        pnp_points[0]=(left_lightbar_.tl()+left_lightbar_.tr())/2;
        pnp_points[9]=(right_lightbar_.bl()+right_lightbar_.br())/2;
        pnp_points[8]=(right_lightbar_.tl()+right_lightbar_.tr())/2;
    } else if((form_[0]||form_[1])&&form_[2]) {
        pnp_points[5]=(numberstiker_.tl()+numberstiker_.tr())/2;
        pnp_points[6]=(numberstiker_.bl()+numberstiker_.br())/2;
        if(left_lightbar_.get_area()>right_lightbar_.get_area()){
            pnp_points[1]=(left_lightbar_.bl()+left_lightbar_.br())/2;
            pnp_points[0]=(left_lightbar_.tl()+left_lightbar_.tr())/2;
            pnp_points[2]=left_lightbar_.get_center();
        }else{
            pnp_points[9]=(right_lightbar_.bl()+right_lightbar_.br())/2;
            pnp_points[8]=(right_lightbar_.tl()+right_lightbar_.tr())/2;
            pnp_points[10]=right_lightbar_.get_center();
        }
    }  else{
        pnp_points[3]=(numberstiker_.tl()+numberstiker_.bl())/2;
        pnp_points[4]=(numberstiker_.tr()+numberstiker_.br())/2;
        pnp_points[5]=(numberstiker_.tl()+numberstiker_.tr())/2;
        pnp_points[6]=(numberstiker_.bl()+numberstiker_.br())/2;
    }
}

////////////////////////////// Plane //////////////////////////////

FinitePlane::FinitePlane() {
    empty_= true;
    world_point_=Point3f(0,0,0);
    image_point_=Point2f(0,0);
    region_size_=Size(0,0);
}
FinitePlane::FinitePlane(Mat rot_vector,Mat tran_vector,Armor &armor,float pnp_assess) {

    empty_= false;
    rvec_=rot_vector.clone();
    tvec_=tran_vector.clone();
    region_size_ = armor.get_numberstiker().get_rotated_rect().size;
    image_point_ = armor.get_numberstiker().get_center();
    pnp_assess_ = pnp_assess;
    tran_vector.convertTo(tran_vector,CV_32F);
    world_point_=*(tran_vector.ptr<Point3f>());
    euler_angle_=rvecToEulerAngles(rot_vector);
    polor_point_=to_polor(world_point_);

    if(armor.get_left_lightbar().empty()){
        llheight_=0;
    } else{
        llheight_=armor.get_left_lightbar().get_height();
    }
    if(armor.get_right_lightbar().empty()){
        rlheight_=0;
    } else{
        rlheight_=armor.get_right_lightbar().get_height();
    }
}
FinitePlane::FinitePlane(Mat rot_vector,Mat tran_vector,EnergyBuffArmor &armor,float pnp_assess) {

    empty_= false;
    region_size_ = armor.get_rotatedrect().size;
    image_point_ = armor.get_rotatedrect().center;
    pnp_assess_ = pnp_assess;

    Mat rotmatix;
    Rodrigues(rot_vector,rotmatix);//旋转向量转换为旋转矩阵
    vector<double>armor_centor={0,70,14.5};//镜头到轴补偿,不同车不一样
    Mat centor=rotmatix*Mat(armor_centor)+tran_vector;
    centor.convertTo(centor,CV_32F);
    world_point_=*(centor.ptr<Point3f>());
    euler_angle_=rvecToEulerAngles(rot_vector);
    polor_point_=to_polor(world_point_);

    rvec_=rot_vector;
    tvec_=tran_vector;
}

////////////////////////////// Target //////////////////////////////

Target::Target(){
    empty_= true;
}
Target::Target(Mat &rvec, Mat &tvec,vector <FinitePlane> &planes,ReciveMessage recive_message){
    empty_= false;
    planes_=planes;
    recive_message_=recive_message;
    rvec_=rvec;
    tvec_=tvec;
}
BuffTarget::BuffTarget( Mat &rvec, Mat &tvec,vector<FinitePlane> &planes, ReciveMessage recive_message):Target(rvec,tvec,planes,recive_message){
    empty_= false;

    recive_message_=recive_message;
    circle_world_point_=Point3f(*(tvec.ptr<Point3d>()));
    circle_world_polor_=to_polor(circle_world_point_);
    idex_=0;
}
void BuffTarget::correctself(Mat state) {
    float _y=state.at<float>(0);
    if(fabs(sin(circle_world_polor_.pitch))>0.2){
        circle_world_polor_.distance=_y/sin(circle_world_polor_.pitch);
    } else{
        circle_world_polor_.distance=sqrt(_y*_y+circle_world_point_.x*circle_world_point_.x+circle_world_point_.z*circle_world_point_.z);
    }
    Point3f circle_world_point=to_sprectangular(circle_world_polor_);
    Point3f armor_world_point=planes_[0].get_world_point();
    armor_world_point+=circle_world_point-circle_world_point_;
    circle_world_point_=circle_world_point;
    planes_[0].set_world_point(armor_world_point);
    Polor3f armor_world_polor=to_polor(armor_world_point);
    planes_[0].set_polor_point(armor_world_polor);
    tvec_=Mat(Point3d(circle_world_point_));
}
RobotTarget::RobotTarget(Mat &rvec, Mat &tvec,vector<FinitePlane> &planes, RobotType robotType, ReciveMessage recive_message) :Target(rvec,tvec,planes,recive_message){
    robot_type_=robotType;
}
void RobotTarget::correctself(Mat state) {
    float _dist=state.at<float>(0);
    Polor3f world_polor=planes_[0].get_polor_point();
    world_polor.distance=_dist;
    Point3f world_point=to_sprectangular(world_polor);
    planes_[0].set_polor_point(world_polor);
    planes_[0].set_world_point(world_point);
    tvec_=Mat(Point3d(world_point));
}

float RobotTarget::get_axis_yaw() {
    if(axis_yaw_!= -1){
        return axis_yaw_;
    }
    float axis_yaw_=0;
    if (planes_[1].empty()){
        float diffangel=planes_[0].get_euler_angle()[1]-planes_[0].get_polor_point().yaw;
        if(fabs(diffangel)>kPI/7)diffangel=planes_[0].get_euler_angle()[1]+planes_[0].get_polor_point().yaw;
        float ddist=sin(diffangel)*30;
        axis_yaw_=planes_[0].get_polor_point().yaw;
        if(planes_[0].get_llh()>planes_[0].get_rlh()){
            axis_yaw_+=-fabs(asin(ddist/planes_[0].get_polor_point().distance));
        } else{
            axis_yaw_+=fabs(asin(ddist/planes_[0].get_polor_point().distance));

        }

//        Vec3d p0={0,0,30};
//        Mat p=Mat(p0);//物体坐标系中预测点的位置
//        Mat rotmatix;
//        Mat tvec=planes_[0].get_tvec();
//        Rodrigues(planes_[0].get_rvec(), rotmatix);//旋转向量转换为旋转矩阵
//        p=rotmatix*p+tvec;//预测点在世界坐标系中的位置
//
//
//        p.convertTo(p,CV_32F);
//        Point3f axis = *(p.ptr<Point3f>());//转化为Point3f
//        axis_yaw_=to_polor(axis).yaw;


    } else{
        const float &a1=planes_[0].get_region_size().area();
        const float &a2=planes_[1].get_region_size().area();
        float k1=a1/(a1+a2);
        axis_yaw_=k1*planes_[0].get_polor_point().yaw+(1-k1)*planes_[1].get_polor_point().yaw;

        if(planes_[0].get_polor_point().yaw*planes_[1].get_polor_point().yaw<0&&fabs(planes_[0].get_polor_point().yaw)>kPI/2) {
            float yaw1,yaw2;
            if(planes_[0].get_polor_point().yaw>0){
                yaw1=kPI-planes_[0].get_polor_point().yaw;
                yaw2=-kPI-planes_[1].get_polor_point().yaw;
            } else{
                yaw1=-kPI-planes_[0].get_polor_point().yaw;
                yaw2=kPI-planes_[1].get_polor_point().yaw;
            }
            axis_yaw_=yaw1+(1-k1)*yaw2;
            if(axis_yaw_>0){
                axis_yaw_=kPI-axis_yaw_;
            } else{
                axis_yaw_=-kPI-axis_yaw_;
            }

        }
        //axis_yaw_=axis_yaw_*flag;
//        G_armorinformation.tmpparam0=planes_[0].get_polor_point().yaw*k1;
//        G_armorinformation.tmpparam1=planes_[1].get_polor_point().yaw*(1-k1);
//        G_armorinformation.tmpparam2=planes_[0].get_polor_point().yaw;
//        G_armorinformation.tmpparam3=planes_[1].get_polor_point().yaw;
//
//        G_armorinformation.tmpparam4=axis_yaw_;
        //G_armorinformation.tmpparam4=flag;
    }


    return axis_yaw_;

}

////////////////////////////// EnergyBuffArmor //////////////////////////////

EnergyBuffArmor:: EnergyBuffArmor(){
    ml_image_ = Mat::zeros(28,96,CV_8U);
    angle=0;
    type_=None;
    empty_= true;
}

EnergyBuffArmor::EnergyBuffArmor(RotatedRect armor_rotatedrect,bool up_OR_down) {
    rotatedrect_=armor_rotatedrect;
    Point2f vertices[4];
    rotatedrect_.points(vertices);
    center_=rotatedrect_.center;
    if(rotatedrect_.size.width<rotatedrect_.size.height){
        height_=rotatedrect_.size.width;
        width_=rotatedrect_.size.height;
        if(up_OR_down){
            tl_=vertices[2];
            tr_=vertices[3];
            bl_=vertices[1];
            br_=vertices[0];
        } else{
            tl_=vertices[0];
            tr_=vertices[1];
            bl_=vertices[3];
            br_=vertices[2];
        }
    } else{
        width_ =armor_rotatedrect.size.width;
        height_=armor_rotatedrect.size.height;
        if(up_OR_down){
            tl_=vertices[1];
            tr_=vertices[2];
            bl_=vertices[0];
            br_=vertices[3];
        } else {
            tl_ = vertices[3];
            tr_ = vertices[0];
            bl_ = vertices[2];
            br_ = vertices[1];
        }
    }
    ml_image_ = Mat::zeros(28,96,CV_8U);
    empty_= false;
}

RotatedRect EnergyBuffArmor::get_circle_rect_() const {
    Point2f diff=(tl_-bl_)*width_/height_;
    RotatedRect rrect(center_-3*diff,Size2f(width_*1.6f,width_*1.6f),rotatedrect_.angle);
    return rrect;
}

RotatedRect EnergyBuffArmor::get_ml_rect_() const {
    Point2f mlcenter=circle_*3/7+center_*4/7;
    RotatedRect mlrect;
    if(rotatedrect_.size.width<rotatedrect_.size.height){
        mlrect =RotatedRect(mlcenter,Size2f(width_*1.8f,width_/2.f),rotatedrect_.angle);
    } else{
        mlrect =RotatedRect(mlcenter,Size2f(width_/2.f,width_*1.8f),rotatedrect_.angle);

    }
    return mlrect;
}

void EnergyBuffArmor::set_circle(Point2f circle) {
    circle_=circle;
    Point2f diff=center_-circle;
    angle=atan2(diff.x,-diff.y);
}

BulletSpeedSolve::BulletSpeedSolve() {
    last_time=0;
    last_speed=2.7f;
}

float BulletSpeedSolve::solve_speed(float bullet_speed, double current_time) {
    if(bullet_speed==0){
        return 2.7f;
    }
    if(saved_speed.empty()&&bullet_speed!=last_speed){
        saved_speed.push_back(bullet_speed);
        last_time=current_time;
        last_speed=bullet_speed;
        return bullet_speed;
    }
    if(current_time-last_time>=5000|fabs(last_speed- bullet_speed)>0.9f){
        last_time=current_time;
        saved_speed.clear();
        return last_speed;
    }else{
        if(bullet_speed==last_speed){
            return last_speed;
        }
        saved_speed.push_back(bullet_speed);
        last_speed=bullet_speed;
        float result_speed=0;
        for(int i=0;i<saved_speed.size();i++){
            result_speed+=saved_speed[i];
        }
        if(saved_speed.size()==5){
            saved_speed.erase(saved_speed.begin());
            return result_speed/(float)(saved_speed.size()+1.0f);
        }
        return result_speed/(float)saved_speed.size();
    }

}

int getDiskfreespace(){
    struct statfs diskInfo;

    statfs("/home", &diskInfo);
    unsigned long long blocksize = diskInfo.f_bsize;    //每个block里包含的字节数
    unsigned long long totalsize = blocksize * diskInfo.f_blocks;   //总的字节数，f_blocks为block的数目
    unsigned long long freeDisk = diskInfo.f_bfree * blocksize; //剩余空间的大小
    unsigned long long availableDisk = diskInfo.f_bavail * blocksize;   //可用空间大小
//    cout<<"Total_size = "<<(totalsize>>30)<<" GB"<<endl;
//    cout<<"Disk_free = "<<(freeDisk>>30)<<" GB "<<endl;
//    cout<<"Disk_available = "<<(availableDisk>>30)<<" GB "<<endl;
    return availableDisk>>30;
}

unsigned char get_find_object(){
    int fd;
    unsigned char buf[5];
    if (mkfifo("fifo1", 0666) < 0 && errno != EEXIST) // 创建FIFO管道
        perror("Create FIFO Failed");
    if ((fd = open("/home/tdt/桌面/FIFO", O_RDONLY)) < 0)  // 以读打开FIFO
    {
        perror("Open FIFO Failed");
        exit(1);
    }
    // 读取FIFO管道
    if ((read(fd, buf, 5)) > 0) {
//                printf("Read message: %s", buf);
        printf("Read message: %s", buf);
    }
    //sleep(0.01);
//    unsigned char temp=buf[0];
//    cout<<"temp "<<temp<<endl;

    close(fd);  // 关闭FIFO文件
    return buf[0];
}

