#include "kalman.h"
#include "tnet.h"
#include "macro.h"
#include "lob.h"


//#define AIR_OPEN

#define ML_OPEN

//#define LOB_DEBUG

void VasualKalman(int no_kalman_y, int kalman_y,int per);
BridgeLob::BridgeLob(const Camera &camera) {
    base_color_ = Color(camera.get_enemy_colour());
    cam_matrix_ = camera.get_matrix();
    dist_coeffs_ = camera.get_dist_coeffs();
    wigth_=camera.get_width();
    height_=camera.get_height();

    KF_[0]=_1KalmanFilter(1,1,1,100, false);
    KF_[0].set_processNoiseCov(10);
    KF_[0].set_measurementNoiseCov(3000);
    KF_[0].correct(5500,false);
}


#ifndef AIR_OPEN
//void BridgeLob::Get(ReciveMessage recive_message,SendMessage send_message,int where){
//    double distance;
//    double angle;
//    if(where==SELFBRIDGEMODE){
//        double distance = 20162.7273;
//        double angle = -14.35;
//    }else if(where==SPRINGMODE){
//        double distance = 24180;
//        double angle = -5.18;
//        bridge_height_=0;
//    }
//
//    shoot_platform_ = recive_message.shoot_platform;
//
//
//    const float kG =9.8;
//
//    double tanpitch1=(-distance+sqrt(distance*distance-4*(-0.5*kG*1000*
//                                                          (distance*distance/(bullet_speed_*1000*bullet_speed_*1000))*
//                                                          (-0.5*kG*1000*(distance*distance/(bullet_speed_*1000*bullet_speed_*1000))+
//                                                           (base_height_-bridge_height_-car_height)))))/(-kG*1000*(distance*distance
//                                                                                                                   /(bullet_speed_*1000*bullet_speed_*1000)));
//    double tanpitch2=(-distance-sqrt(distance*distance-4*(-0.5*kG*1000*
//                                                          (distance*distance/(bullet_speed_*1000*bullet_speed_*1000))*
//                                                          (-0.5*kG*1000*(distance*distance/(bullet_speed_*1000*bullet_speed_*1000))+
//                                                           (base_height_-bridge_height_-car_height)))))/(-kG*1000*(distance*distance/(bullet_speed_*1000*bullet_speed_*1000)));
//    double best_tanpitch=tanpitch1;
//
//    if(fabs(tanpitch2)<fabs(tanpitch1))    // ????????
//    {
//        best_tanpitch=tanpitch2;
//    }
//    send_message.pitch=(atan(best_tanpitch)-shoot_platform_.pitch)/3.1415926*180.;
//    send_message.yaw=angle-shoot_platform_.yaw/3.1415926*180.;
//    send_message.no_object=false;
//}

bool BridgeLob::Get(Mat src, ReciveMessage recive_message,SendMessage &send_message,int where) {
    // ???????????????
//        cout<<"pyaw                      "<<recive_message.shoot_platform.yaw<<endl;
    if(where==BRIDGEMODE){
        bridge_height_=1050;
    }else if(where==FREEWAYMODE){
        bridge_height_=150;
    }
    //bullet_speed_=recive_message.bulletspeed;
    roi_rect_=roi_filter(src,last_target_);
    if(!BaseDectector(src)){
        last_target_={0,0};
        send_message.pitch=0;
        send_message.yaw=0;
        send_message.beat= false;
        send_message.no_object=true;
        return false;
    }
    last_target_=base_armor_;
    shoot_platform_ = recive_message.shoot_platform;
    bullet_speed_=recive_message.bulletspeed;
    DistanceFinder();
    AngleResolver(send_message);
    return true;
}

Rect BridgeLob::roi_filter(Mat src,Point last_target) {
    Rect src_rect;
    src_rect.tl()={0,0};
    src_rect.width=src.cols;
    src_rect.height=src.rows;
    if(last_target.x==0|last_target.y==0){
        return  src_rect;
    }else{
        Rect result_rect(last_target.x-200,last_target.y-150,400,300);
        result_rect=result_rect&src_rect;
        return result_rect;
    }
}

#ifndef ML_OPEN
bool BridgeLob::BaseDectector(Mat src) {
    //Mat roi=src(Rect(wigth_/5*2,height_/3,wigth_/5,height_/3)).clone();
    Mat roi;
    Mat detect_mat;
    Mat debug;
    src.copyTo(roi);
    src.copyTo(debug);
    roi=roi(roi_rect_);
    //medianBlur(roi,roi,3);

    int threshold_vale =40;
//    Mat gray;
    vector<Mat> channles;
    split(roi,channles);
//    cvtColor(roi,gray,COLOR_BGR2GRAY);  //????
    //threshold(gray,gray,Parameter.ArmorDetect_lightbarthre,255,THRESH_BINARY);//??????? ?????
//    threshold(gray,gray,45,255,THRESH_BINARY);
    detect_mat= (channles[0] - channles[1]) | (channles[2] - channles[1]);
    //detect_mat &= gray;
    threshold(detect_mat,detect_mat,threshold_vale,255,THRESH_BINARY);//??????? ?????
    Mat element_3 = getStructuringElement(MORPH_RECT,Size(3,3));
    Mat element_5 = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(detect_mat,detect_mat,MORPH_CLOSE,element_3);

    /**RGB
    cvtColor(roi, roi, COLOR_BGR2HSV);
    if (base_color_== RED) {
        Mat aImg,bImg;
//        inRange(roi, Scalar(170, 60, 170), Scalar(180, 130, 255),aImg);
//        inRange(roi, Scalar(0, 85, 180), Scalar(30, 255, 255),bImg);
//        roi = aImg+bImg;
        inRange(roi, Scalar(0, 125, 180), Scalar(35, 255, 255),roi);
        dilate(roi, roi, getStructuringElement(MORPH_RECT, Size(3, 3)));
    }
    else{
        inRange(roi, Scalar(88, 5, 188), Scalar(120, 255, 255),roi);
        dilate(roi, roi, getStructuringElement(MORPH_RECT, Size(3, 3)));
    }

    Mat element_3 = getStructuringElement(MORPH_RECT,Size(3,3));
    Mat element_5 = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(roi,roi,MORPH_CLOSE,element_3);
//    imshow("HSV",roi);
     */

/**
    int threshold_vale = Parameter.ArmorDetect_lightbarthre;
    vector<Mat> channles;
    split(roi,channles);
    Mat binarization = (channles.at(2)-channles.at(0))&(channles.at(2)-channles.at(1));
    //Mat binarization = (channles.at(0)-channles.at(2))&(channles.at(0)-channles.at(1));
    imshow("binarization",binarization);
    threshold(binarization,binarization,threshold_vale,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
    //dilate(binarization,binarization,element);
    Mat element_3 = getStructuringElement(MORPH_RECT,Size(3,3));
    Mat element_5 = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(binarization,binarization,MORPH_DILATE,element_5);
    morphologyEx(binarization,binarization,MORPH_ERODE,element_3);
    //morphologyEx(binarization,binarization,MORPH_CLOSE,element_3);
*/

    vector<vector<Point>> vContours;
    vector<Vec4i> vHierarchy;
    vector<Point> light;
    vector<LightBar> horizontal_light;
    vector<LightBar> vertical_light;
    vector<LightBar> left_light;
    vector<LightBar> right_light;
    vector<Point> base;
    vector<double> lenght;
    Point2f vertex[4];
    RotatedRect box;
    vector<RotatedRect> fbox;
    findContours(detect_mat, vContours, vHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE,roi_rect_.tl());
#ifdef LOB_DEBUG
    for (int i = 0; i < vContours.size(); i++) {
        drawContours(debug, vContours, i, Scalar(0, 0, 255), 1);
    }
#endif

    // ?????
    for (int i = 0; i < vContours.size(); i++) {
        double area = contourArea(vContours[i],false);
        box = minAreaRect(vContours[i]);
        LightBar temp_light(box,Color(UNKNOW));
        //cout<<temp_light.get_angle()<<"    "<<temp_light.get_ratio()<<endl;
        if((temp_light.get_ratio()>=1 ||
            (temp_light.get_ratio()<0.7&&temp_light.get_angle()==45)) &&
           area >10 && area <400){
            horizontal_light.push_back(temp_light);
            Point2f vertices[4];
            box.points(vertices);
#ifdef LOB_DEBUG
//            for(int j=0;j<4;j++){
//                line(debug,vertices[j],vertices[(j+1)%4],Scalar(255,255,0));
//            }
#endif
            //continue;
        }
        if(fabs(temp_light.get_angle())<25 && temp_light.get_ratio()<1.4 &&
           area>25 &&area<300){
            vertical_light.push_back(temp_light);
            Point2f vertices[4];
            box.points(vertices);
#ifdef LOB_DEBUG
//            for(int j=0;j<4;j++){
//                line(debug,vertices[j],vertices[(j+1)%4],Scalar(0,255,0));
//            }
#endif
            //continue;
        }
    }
    for (int i=0;i<horizontal_light.size();i++){
        for(int j=0;j<vertical_light.size();j++){
            int refer_lenght = vertical_light[j].get_height();
            Point horizontal = horizontal_light[i].get_center();
            Point vertical = vertical_light[j].get_center();
            if(abs(horizontal.x-vertical.x)>3.5*refer_lenght ||
               abs(horizontal.x-vertical.x)<0.34*refer_lenght ||
               abs(horizontal.y-vertical.y)>refer_lenght*1) continue;
            if(horizontal.x-vertical.x<0){
                left_light.push_back(vertical_light[j]);
                Point2f vertices[4];
                vertical_light[j].get_rotatedrect().points(vertices);
#ifdef LOB_DEBUG
                for(int j=0;j<4;j++){
                    line(debug,vertices[j],vertices[(j+1)%4],Scalar(0,0,255));
                }
#endif
            }else{
                right_light.push_back(vertical_light[j]);
                Point2f vertices[4];
                vertical_light[j].get_rotatedrect().points(vertices);
#ifdef LOB_DEBUG
                for(int j=0;j<4;j++){
                    line(debug,vertices[j],vertices[(j+1)%4],Scalar(0,0,255));
                }
#endif
            }
        }
    }
    for (int i=0;i<left_light.size();i++){
        for (int j=0;j<right_light.size();j++){
            int refer_lenght = max(left_light[i].get_height(),right_light[j].get_height());
            Point left=left_light[i].get_center();
            Point right=right_light[j].get_center();
            if(left.x>right.x) {
                continue;
            }
            if(right.x-left.x<2.5*refer_lenght||right.x-left.x>12*refer_lenght||
               abs(right.y-left.y)>1.5*refer_lenght)
                continue;//{cout<<"bbb"<<endl;imshow("deub",debug);;waitKey(0);continue;}
            if(abs(left_light[i].get_area()-right_light[j].get_area())>
               (left_light[i].get_area()+right_light[j].get_area())/2.5)
                continue;//{cout<<"ccc"<<endl;imshow("debug",debug);;waitKey(0);continue;}
            Point temp_base((right.x+left.x)/2,(right.y+left.y)/2);
            base.push_back(temp_base);
            lenght.push_back(pow(pow(left.x-right.x,2)+pow(left.y-right.y,2),0.5));
#ifdef LOB_DEBUG
            line(debug,left,right,Scalar(255,255,255));
#endif
        }
    }
    if(base.size()==0){
        cout<<"can not find base"<<endl;
#ifdef LOB_DEBUG
        imshow("debug",debug);
        waitKey(1);
#endif
        return false;
    }else{
        Point out_base;
        double out_lenght;
        for(int i=0;i<base.size();i++){
            int mark=height_*height_+wigth_*wigth_+1;
            int center_dis=pow(wigth_/2-base[i].x,2)+pow(height_*3/4-base[i].y,2);
            if(mark>center_dis){
                out_base=base[i];
                out_lenght=lenght[i];
            }
        }
        circle(debug,out_base,20,Scalar(255,0,0),2);
        base_armor_=out_base;
        light_dis_=out_lenght;
#ifdef LOB_DEBUG
        imshow("debug",debug);
        waitKey(1);
#endif
        return true;
    }
}
#endif

#ifdef ML_OPEN
bool BridgeLob::BaseDectector(Mat src) {
    //Mat roi=src(Rect(wigth_/5*2,height_/3,wigth_/5,height_/3)).clone();
    Mat roi;
    Mat detect_mat;
    Mat debug;
    src.copyTo(roi);
    src.copyTo(debug);
    roi=roi(roi_rect_);

    int threshold_vale =Parameter.ArmorDetect_lightbarthre;
    Mat gray;
    vector<Mat> channles;
    split(roi,channles);
    cvtColor(roi,gray,COLOR_BGR2GRAY);  //????
    detect_mat= (channles[0] - channles[1]) | (channles[2] - channles[1]);
    detect_mat |= gray;
    threshold(detect_mat,detect_mat,threshold_vale,255,THRESH_BINARY);//??????? ?????
//    imshow("111",detect_mat);
    vector<vector<Point>> vContours;
    vector<Vec4i> vHierarchy;
    vector<LightBar> light;
    vector<Armor> armors;
    vector<double> lenght;
    Armor base;
    double base_lenght;

    vector<RotatedRect> fbox;
    findContours(detect_mat, vContours, vHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE,roi_rect_.tl());
#ifdef LOB_DEBUG
    for (int i = 0; i < vContours.size(); i++) {
        drawContours(debug, vContours, i, Scalar(0, 0, 255), 1);
    }
#endif

    // ?????
    for (int i = 0; i < vContours.size(); i++) {
        double area = contourArea(vContours[i],false);
        if(area<5||area>300){
            continue;
        }
        RotatedRect box;
        box = minAreaRect(vContours[i]);
        LightBar temp_light(box,Color(UNKNOW));
        //cout<<temp_light.get_angle()<<"    "<<temp_light.get_ratio()<<endl;
        if(fabs(temp_light.get_angle()>45)){
            continue;
        }
        if(temp_light.get_ratio()>1.6) {
            continue;
        }
        light.push_back(temp_light);

#ifdef LOB_DEBUG
        Point2f vertices[4];
        box.points(vertices);
        for(int j=0;j<4;j++){
            line(debug,vertices[j],vertices[(j+1)%4],Scalar(255,255,0));
        }
#endif
    }
    sort(light.begin(),light.end(),[](LightBar a,LightBar b)->bool{ return a.get_center().x < b.get_center().x;});

    for(int i=0; i<light.size(); i++)//双层循环 遍历每一对矩形框
    {
        for(int j=i+1; j < light.size(); j++) {
            int refer_length=(light[i].get_height()+light[j].get_height())/2;
            if(light[j].get_center().x-light[i].get_center().x<3*refer_length||
               light[j].get_center().x-light[i].get_center().x>11*refer_length){ //TODO 加入绝对数据
                continue;
            }
            if(fabs(light[j].get_center().y-light[i].get_center().y)>2*refer_length){
                continue;
            }

            Rect search_rect=get_search_rect(light[i],light[j]);
            if(!RectSafety(search_rect, src.size())) continue;   //安全性
            Mat image_numb=src(search_rect);
            vector<Mat> rgb;
            int thre=0;
            cvtColor(image_numb,image_numb,CV_BGR2GRAY);

            if(!region_otsu_threshold(image_numb,image_numb,thre, 0)) continue;

            vector<vector<Point>>contours;
            findContours(image_numb,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,search_rect.tl());

            vector<RotatedRect> Numb_rects;
            LightBar lightbar=light[i].get_area()>light[j].get_area()?light[i]:light[j];
            for(vector<Point> const &contour : contours) {
                if (contour.size()<25 ) continue;//面积太小的不要
                RotatedRect Numb_rect = minAreaRect(contour,(int)lightbar.get_rotatedrect().angle,0);
                Rect brect=Numb_rect.boundingRect();
                //if(brect.width>brect.height)continue;

                if (Numb_rect.size.area() < 0.1* search_rect.area()) continue;

                float dist1=getDistance(Point(Numb_rect.center),light[i].get_center());
                float dist2=getDistance(Point(Numb_rect.center),light[j].get_center());
                if(light[i].get_height()>light[j].get_height()){
                    if(dist1>1.7*dist2||dist2>1.3*dist1)continue;
                } else{
                    if(dist1>1.3*dist2||dist2>1.7*dist1)continue;
                }

                Numb_rects.push_back(Numb_rect);
            }

            NumberStiker numberStiker;
            if(Numb_rects.empty()){
                continue;
            } else {
                sort(Numb_rects.begin(),Numb_rects.end(),
                     [](RotatedRect a,RotatedRect b)->bool{ return a.size.area()>b.size.area();});//以旋转矩形面积从大到小排序
                RotatedRect Numb_rotatedrect=Numb_rects[0];
                numberStiker=NumberStiker(Numb_rotatedrect);//将search封装为贴纸
                Rect numberbr=Numb_rotatedrect.boundingRect();
                if(!RectSafety(numberbr,src.size())) continue;
                numberStiker.set_bounding_rect(numberbr);
            }
            Armor tmp_armor=Armor(numberStiker,light[i],light[j]);//封装一个Armor
            tmp_armor.set_num_threshold(thre);
            SetMlRoi(tmp_armor,src);//设置贴纸区域
            armors.push_back(tmp_armor);
            lenght.push_back(pow(pow(light[j].get_center().x-light[i].get_center().x,2)+
                                 pow(light[j].get_center().y-light[i].get_center().y,2),0.5));
        }
    }

    TnetDL(armors);
    double max_lenght=1800;
    for(int i=0;i<armors.size();i++){
        imshow("ml",armors[i].get_ml_roi());
        cout<<"ml     "<<armors[i].get_robot_type()<<endl;
        if(armors[i].get_robot_type()==BASE && lenght[i]<max_lenght){
            base=armors[i];
            base_lenght=lenght[i];
            max_lenght=lenght[i];
        }
    }

    if(base.empty()){
        cout<<"can not find base"<<endl;
#ifdef LOB_DEBUG
        imshow("debug",debug);
        waitKey(1);
#endif
        return false;
    }else{
        Point out_base=Point((base.get_left_lightbar().get_center().x+base.get_right_lightbar().get_center().x)/2,
                             (base.get_left_lightbar().get_center().y+base.get_right_lightbar().get_center().y)/2);
        circle(debug,out_base,20,Scalar(255,0,0),2);
        base_armor_=out_base;
        light_dis_=base_lenght;
#ifdef LOB_DEBUG
        imshow("debug",debug);
        waitKey(1);
#endif
        return true;
    }
}
#endif

void BridgeLob::DistanceFinder() {
    // ????????
    /*double angle=-Pixel2Angle(height_/2-base_armor_.y,'y',cam_matrix_)
                -shoot_platform_.pitch;  // ?????????????????
    z_distance_=(base_height_-bridge_height_-car_height)/tan(angle);  // ????????????????????
    cout<<"pangle"<<Pixel2Angle(height_/2-base_armor_.y,'y',cam_matrix_)<<endl;
    cout<<"tan"<<tan(angle)<<endl;
    cout<<"dis"<<z_distance_<<endl;*/
    z_distance_=world_light*cam_matrix_.at<double>(1,1)/light_dis_;
    //if(lost_target>10){
    if(get_timenow()-last_time_>300){
        KF_[0].correct(z_distance_,false);
        get_target_=1;
    }else{
        if(get_target_>30){
            if(abs(last_distance-z_distance_)>500){
                z_distance_=last_distance;
            }
        }else{
            get_target_++;
        }

        KF_[0].correct(z_distance_);
    }
    //VasualKalman(z_distance_,KF_[0].get_statePost(),2);
    last_time_=get_timenow();
    z_distance_=KF_[0].get_statePost();
    last_distance=z_distance_;

//    cout<<"light"<<light_dis_<<endl;
    cout<<"dis     "<<z_distance_<<endl;

}
//void BridgeLob::AngleResolver(SendMessage &send_message) {
//    const float kG =9.8;
//
//    double tanpitch1=(-z_distance_+sqrt(z_distance_*z_distance_-4*(-0.5*kG*1000*
//                                                                   (z_distance_*z_distance_/(bullet_speed_*1000*bullet_speed_*1000))*
//                                                                   (-0.5*kG*1000*(z_distance_*z_distance_/(bullet_speed_*1000*bullet_speed_*1000))+
//                                                                    (base_height_-bridge_height_-car_height)))))/(-kG*1000*(z_distance_*z_distance_/(bullet_speed_*1000*bullet_speed_*1000)));
//    double tanpitch2=(-z_distance_-sqrt(z_distance_*z_distance_-4*(-0.5*kG*1000*
//                                                                   (z_distance_*z_distance_/(bullet_speed_*1000*bullet_speed_*1000))*
//                                                                   (-0.5*kG*1000*(z_distance_*z_distance_/(bullet_speed_*1000*bullet_speed_*1000))+
//                                                                    (base_height_-bridge_height_-car_height)))))/(-kG*1000*(z_distance_*z_distance_/(bullet_speed_*1000*bullet_speed_*1000)));
//    double best_tanpitch=tanpitch1;
//
//    if(fabs(tanpitch2)<fabs(tanpitch1))    // ????????
//    {
//        best_tanpitch=tanpitch2;
//    }
//    send_message.pitch=(atan(best_tanpitch)-shoot_platform_.pitch)+Parameter.lob_pitch_offset;
//    send_message.yaw=-Pixel2Angle(wigth_/2-base_armor_.x,'x',cam_matrix_)+Parameter.lob_yaw_offset;
//    send_message.no_object=false;
//
////    cout<<"pitch     "<<atan(best_tanpitch)/3.1415926*180.<<endl;
////    cout<<"platform  "<<shoot_platform_.pitch/3.1415926*180.<<endl;
////    cout<<"set       "<<((atan(best_tanpitch))-shoot_platform_.pitch)/3.1415926*180.<<endl;
////
////    cout<<"distance   "<<target_.get_distance()<<endl;
////    cout<<"pitch      "<<target_.get_pitch()<<endl;
////    cout<<"yaw        "<<target_.get_yaw()<<endl<<endl;
//}

void BridgeLob::AngleResolver(SendMessage &send_message) {
    double yaw=-Pixel2Angle(wigth_/2-base_armor_.x,'x',cam_matrix_);
    double pixel_pitch_angle=Pixel2Angle(height_/2-base_armor_.y,'y',cam_matrix_);
    Polor3f camera_polor={float(z_distance_),float(yaw),float(pixel_pitch_angle)};
    Point3f cmamera_point=to_sprectangular(camera_polor);
    Mat tvec=Mat(Point3d(cmamera_point));

    Vec3f diffeular={shoot_platform_.pitch+0.003f,shoot_platform_.yaw-0.022f,0};//云台角度补偿
    Mat rotmatix_diff=eulerAnglesToRotationMatrix(diffeular);//由欧拉角得到旋转矩阵(相机坐标到世界坐标系)

    Vec3d difftran={0,-45,119};//相机到转轴的位置补偿,不同车不一样,理解为世界坐标系原点在相机坐标系下的位置,并方向取反
    Mat difftranvec=Mat(difftran);
    tvec=rotmatix_diff*(tvec+difftranvec);//相机坐标系中的点在世界坐标系中的位置,即是平移向量

    /////重力补偿/////
    Point3f world_point=Point3f(*(tvec.ptr<Point3d>()));//转化为Point3f
    float x1,y1;//平面直角坐标系,抛物线过点(x1,y1)
    y1=world_point.y;
    x1=sqrt(world_point.x*world_point.x+world_point.z*world_point.z);//平面坐标系中的x1,y1
    Vec2f phi=parabolasolve(Point2f(x1/10,y1/10),bullet_speed_);//过点(x1,y1)解抛物线方程,得到两个出射角的解

    float pitch=fabs(phi[0])<fabs(phi[1])?phi[0]:phi[1] ;

    send_message.yaw=atan2(world_point.x,world_point.z)-shoot_platform_.yaw;
    //cout<<"syaw                      "<<send_message.yaw<<endl;
    cout<<world_point.z<<"     "<<world_point.y<<endl;
    cout<<"ayaw                      "<<atan2(world_point.x,world_point.z)<<endl;
    //cout<<"pyaw                      "<<shoot_platform_.yaw<<endl;
    send_message.pitch=pitch-shoot_platform_.pitch;
    AngleCorrect(send_message.yaw);
    AngleCorrect(send_message.pitch);
    send_message.no_object=0;
    send_message.beat=1;
    cout<<"bull              "<<bullet_speed_<<endl;
//    cout<<"bulletspeed    "<< bullet_speed_<<endl;
//    cout<<"z_dis_         "<<z_distance_<<endl;
//    cout<<"x1             "<<x1<<endl;
//    cout<<"y1             "<<y1<<endl;
//    cout<<"sendpitch      "<<send_message.pitch<<endl;


    /**
    double dis_y=base_height_-bridge_height_-car_height;

    double pixel_pitch_angle=Pixel2Angle(height_/2-base_armor_.y,'y',cam_matrix_);
    double absolute_pitch_angle=shoot_platform_.pitch+pixel_pitch_angle;
    double dis_x=z_distance_*cos(pixel_pitch_angle)+119;

    Vec2f phi=parabolasolve(Point2f(dis_x/10,dis_y/10),bullet_speed_); // 过点(x1,y1)解抛物线方程,得到两个出射角的解
    double pitch_absolute=fabs(phi[0])<fabs(phi[1])?phi[0]:phi[1];
    double pitch=pitch_absolute-shoot_platform_.pitch;


    send_message.pitch=pitch+Parameter.lob_pitch_offset;
    send_message.yaw=-Pixel2Angle(wigth_/2-base_armor_.x,'x',cam_matrix_)+Parameter.lob_yaw_offset;
    send_message.no_object=false;

    cout<<"disy                      "<<dis_y<<endl;
    cout<<"disz                      "<<z_distance_<<endl;

//    cout<<"pitch     "<<atan(best_tanpitch)/3.1415926*180.<<endl;
//    cout<<"platform  "<<shoot_platform_.pitch/3.1415926*180.<<endl;
//    cout<<"set       "<<((atan(best_tanpitch))-shoot_platform_.pitch)/3.1415926*180.<<endl;
//
//    cout<<"distance   "<<target_.get_distance()<<endl;
//    cout<<"pitch      "<<target_.get_pitch()<<endl;
//    cout<<"yaw        "<<target_.get_yaw()<<endl<<endl;
     */
}


//bool BridgeLob::FireCommand() {
//    // ????????
//    if(fabs(target_.get_pitch())<(3.1415926/180*3) &&
//    fabs(target_.get_yaw())<(3.1415926/180*3)){
//        return true;
//    }
//    return false;
//}

Rect BridgeLob::get_search_rect(const LightBar &lightbar1, const LightBar &lightbar2) {   //双灯条获得贴纸直矩形
    //根据LeftRightLightBarSafety灯条中点坐标区分左右灯条
    LightBar left_lightbar=lightbar1;
    LightBar right_lightbar=lightbar2;
    LeftRightLightBarSafety(left_lightbar,right_lightbar);
    Point left_lightbar_center=left_lightbar.get_center();
    Point right_lightbar_center=right_lightbar.get_center();

    int height_min = max(left_lightbar.get_height(),right_lightbar.get_height());
    Point search_rect_tl,search_rect_br;

    search_rect_tl.x=max(left_lightbar.br().x,left_lightbar.tr().x)+6;
    search_rect_tl.y= (left_lightbar_center.y+right_lightbar_center.y)/2 -
                      height_min-abs(left_lightbar_center.y-right_lightbar_center.y);
    search_rect_br.x=min(right_lightbar.bl().x,right_lightbar.tl().x)-6;
    search_rect_br.y=(left_lightbar_center.y+right_lightbar_center.y)/2 +
                     height_min+abs(left_lightbar_center.y-right_lightbar_center.y);

    Rect search_rect=Rect(search_rect_tl,search_rect_br);

    return search_rect;
}


bool BridgeLob::region_otsu_threshold(const Mat &inputimage, Mat &outputimage, int &thre, int lr) {
    bool ret= false;
    Mat sum_row,judge;
    Mat tmp;
    Rect orect;
    if(lr==-1){
        orect=Rect(Point(cvRound(0.5*inputimage.cols),cvRound(0.35*inputimage.rows)),
                   Point(cvRound(0.9*inputimage.cols),cvRound(0.65*inputimage.rows)));
    } else if(lr==1){
        orect=Rect(Point(cvRound(0.1*inputimage.cols),cvRound(0.35*inputimage.rows)),
                   Point(cvRound(0.5*inputimage.cols),cvRound(0.65*inputimage.rows)));
    } else{
        orect=Rect(Point(cvRound(0.3*inputimage.cols),cvRound(0.35*inputimage.rows)),
                   Point(cvRound(0.7*inputimage.cols),cvRound(0.65*inputimage.rows)));
    }
    thre=cvRound(threshold(inputimage(orect),tmp,0,255,THRESH_OTSU));
    threshold(inputimage,outputimage,thre,1,THRESH_BINARY);//将灰度图二值化 寻找灯条

    if(sum(outputimage)[0]>0.6*outputimage.size().area())
        return false;

    reduce(outputimage(Range(outputimage.rows*3/10,outputimage.rows*7/10),
                       Range::all()),sum_row,1, REDUCE_SUM,CV_32F);
    threshold(sum_row,judge,2,1,THRESH_BINARY_INV);
    if (sum(judge)[0] <= 1) {
        threshold(sum_row,judge,outputimage.cols*0.5,1,THRESH_BINARY);
        if (sum(judge)[0] < 0.5*sum_row.rows) {
            ret= true;
        }
    }
    outputimage*=255;
    return ret;
}

void BridgeLob::SetMlRoi(Armor &armor,const Mat &src) {
    Mat armorimg;
    RotatedRect roi=armor.get_armor_rotatedrect();
    Rect roi_mat=roi.boundingRect();
    Rect roi_matcopy=Rect(roi_mat);
    if(RectSafety(roi_matcopy,src.size())){
        Mat roiimg;
        if(roi_matcopy!=roi_mat){
            roiimg=Mat::zeros(roi_mat.size(),CV_8UC3);
            src(roi_matcopy).copyTo(roiimg(Rect(roi_matcopy.tl()-roi_mat.tl(),roi_matcopy.size())));
        } else{
            roiimg=src(roi_mat);
        }
        float angle=armor.get_numberstiker().get_angle();

        Mat rotationmat = getRotationMatrix2D(Point(roi.center)-roi_mat.tl(),angle, 1);      //仿射变换
        warpAffine(roiimg,armorimg,rotationmat,roiimg.size()); //TODO 会把灯条框进去
        Size size_;
        if(roi.angle>=-45){
            size_=roi.size;
        } else{
            size_=Size(cvRound(roi.size.height),cvRound(roi.size.width));
        }
        Rect realsizerect(Point(roi.center)-roi_mat.tl()-Point(size_)/2,size_);
        if(RectSafety(realsizerect,armorimg.size()))
        {  armorimg = armorimg(realsizerect);
        } else{
            armorimg = Mat::zeros(28,28,CV_8UC3);
        }
    } else{
        armorimg = Mat::zeros(28,28,CV_8UC3);
    }

    Mat rgb[3];
    split(armorimg,rgb);
//    armorimg = rgb[2-enemy_color_];
    cvtColor(armorimg,armorimg,CV_BGR2GRAY);

    resize(armorimg,armorimg,Size(28,28));
    armor.set_ml_roi(armorimg);
}

void BridgeLob::TnetDL(vector<Armor> &input_armors){


    vector<Mat> ml_rois;
    vector<int> results;
    for(Armor &armor :input_armors){
        Mat armorimg=armor.get_ml_roi();
        armorimg.convertTo(armorimg,CV_32F);
        normalize(armorimg,armorimg);
        ml_rois.push_back(armorimg);
    }
    static Tnet tdtnet(0);
    tdtnet.predict(ml_rois,results);
    for(int i=0; i<input_armors.size(); i++) {
        if(input_armors[i].get_robot_type()==SENTRY) continue;
        if(results[i]==7){
            input_armors[i].set_robot_type(SENTRY);
        }else if(results[i]==8){
            input_armors[i].set_robot_type(BASE);
        }else
            input_armors[i].set_robot_type(RobotType(results[i]));
    }
}
#endif



#ifdef AIR_OPEN
AirLob::AirLob(const Camera &camera) {
    base_color_ = Color(camera.get_enemy_colour());
    cam_matrix_ = camera.get_matrix();
    dist_coeffs_ = camera.get_dist_coeffs();
    wigth_=camera.get_width();
    height_=camera.get_height();

    KF_[0]=_1KalmanFilter(1,1,1,100, true);
    KF_[0].set_processNoiseCov(0.0001f);
    KF_[0].set_measurementNoiseCov(0.01f);
    KF_[1]=_1KalmanFilter(1,1,1,100, true);
    KF_[1].set_processNoiseCov(1);
    KF_[1].set_measurementNoiseCov(1);
    KF_[2]=_1KalmanFilter(1,1,1,100, false);
    KF_[2].set_processNoiseCov(225);
    KF_[2].set_measurementNoiseCov(810000);
    KF_[2].correct(9600,false);
    KF_[3]=_1KalmanFilter(1,1,1,100, false);
    KF_[3].set_processNoiseCov(0.0001);
    KF_[3].set_measurementNoiseCov(1);
    last_armor_position={0,0};
}


bool AirLob::Get(Mat src, ReciveMessage recive_message,SendMessage &send_message) {
    shoot_platform_ = recive_message.shoot_platform;
    if(recive_message.shoot_platform.bulletspeed < 3.5 &&recive_message.shoot_platform.bulletspeed>1.5){
        bullet_speed_=recive_message.shoot_platform.bulletspeed;
    }else{
        bullet_speed_=2.6f;
    }
    z_distance_=recive_message.distance;
    roi_rect=roi_filter(src,last_armor_position);
    if(!BaseDectector(src)){
        send_message.no_object=true;
        send_message.yaw=0;
        send_message.pitch=0;
        send_message.beat=0;
        return false;
    }
    DistanceFinder();
    AngleResolver(send_message);

    return true;
}

Rect AirLob::roi_filter(Mat src,Point last_armor) {
    Rect src_rect;
    src_rect.tl()={0,0};
    src_rect.width=src.cols;
    src_rect.height=src.rows;
    if(last_armor.x==0|last_armor.y==0){
        return  src_rect;
    }else{
        Rect result_rect(last_armor.x-150,last_armor.y-150,300,300);
        result_rect=result_rect&src_rect;
        return result_rect;
    }
}

vector<vector<Point>> AirLob::second_detect(Mat roi_mat, Rect roi) {
    medianBlur(roi_mat,roi_mat,5);
    Mat detect_mat;

    int threshold_vale = 40;
    vector<Mat> channles;
    split(roi_mat,channles);
    detect_mat= (channles[0] - channles[1]) | (channles[2] - channles[1]);
    threshold(detect_mat,detect_mat,threshold_vale,255,THRESH_BINARY);//??????? ?????
    Mat element_3 = getStructuringElement(MORPH_RECT,Size(3,3));
    Mat element_5 = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(detect_mat,detect_mat,MORPH_CLOSE,element_3);


    /**RGB
    int threshold_vale = 30;
    vector<Mat> channles;
    split(roi_mat,channles);
    if(base_color_==RED) {
        detect_mat = (channles.at(2) - channles.at(0)) & (channles.at(2) - channles.at(1));
    }else{
        detect_mat = (channles.at(0) - channles.at(2)) & (channles.at(0) - channles.at(1));
    }
    threshold(detect_mat,detect_mat,threshold_vale,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
    dilate(detect_mat,detect_mat,element);
    //morphologyEx(detect_mat,detect_mat,MORPH_OPEN,element);
    */


    /**HSV
    detect_mat=roi_mat;
    cvtColor(detect_mat, detect_mat, COLOR_BGR2HSV);
    if (base_color_== RED) {
   //     Mat aImg,bImg;
//        inRange(detect_mat, Scalar(172, 175, 175), Scalar(180, 255, 255),aImg);
//        inRange(detect_mat, Scalar(0, 125, 114), Scalar(33, 255, 255),bImg);
//        detect_mat = aImg+bImg;
        inRange(detect_mat, Scalar(0, 165, 100), Scalar(30, 255, 255),detect_mat);
        dilate(detect_mat, detect_mat, getStructuringElement(MORPH_RECT, Size(3, 3)));
    }
    else{
        inRange(detect_mat, Scalar(88, 5, 188), Scalar(120, 255, 255),detect_mat);
        dilate(detect_mat, detect_mat, getStructuringElement(MORPH_RECT, Size(3, 3)));
    }
    Mat element_3 = getStructuringElement(MORPH_RECT,Size(3,3));
    Mat element_5 = getStructuringElement(MORPH_RECT,Size(5,5));
    //morphologyEx(detect_mat,detect_mat,MORPH_OPEN,element_3);
    //morphologyEx(roi,roi,MORPH_ERODE,element_3);
    */

#ifdef LOB_DEBUG
    imshow("detect",detect_mat);
    waitKey(1);
#endif
    vector<vector<Point>> second_Contours;
    vector<Vec4i> second_Hierarchy;
    findContours(detect_mat,second_Contours,second_Hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,roi.tl());
    return second_Contours;
}


bool AirLob::BaseDectector(Mat src) {
    Mat roi,debug;
    src.copyTo(roi);
    src.copyTo(debug);
    roi=roi(roi_rect);

    vector<vector<Point>> vContours;
    vector<Vec4i> vHierarchy;
    vector<LightBar> lightbars;
    Point2f vertex[4];
    RotatedRect box;
    vector<RotatedRect> fbox;
    vContours=second_detect(roi,roi_rect);
//    for (int i = 0; i < vContours.size(); i++) {
//        drawContours(debug, vContours, i, Scalar(0, 0, 255), 1);
//    }

    // 筛选横灯条
    for (int i = 0; i < vContours.size(); i++) {
        double area = contourArea(vContours[i],false);
        if (area<5  || area>300) continue;
        box = minAreaRect(vContours[i]);
        Point2f vertices1[4];//        Point2f vertices1[4];
        box.points(vertices1);
        for(int j=0;j<4;j++){
            line(debug,vertices1[j],vertices1[(j+1)%4],Scalar(255,0,0));
        }
        box.points(vertices1);
//        for(int j=0;j<4;j++){
//            line(debug,vertices1[j],vertices1[(j+1)%4],Scalar(255,0,0));
//        }
        LightBar lightbar(box,Color(UNKNOW));
//        if(lightbar.get_ratio()<1.30) {  //根据长宽比和角度筛选灯条
//            continue;
//        }
        lightbars.push_back(lightbar);
#ifdef LOB_DEBUG
        Point2f vertices[4];
        box.points(vertices);
        for(int j=0;j<4;j++){
            line(debug,vertices[j],vertices[(j+1)%4],Scalar(0,255,0));
        }
#endif
    }
    vector<vector<LightBar>> halfbase;
    for(int i=0;i<lightbars.size();i++){
        for(int j = i+1;j<lightbars.size();j++){
            int max_width=max(lightbars[i].get_width(),lightbars[j].get_width());
            //cout<<"!!"<<lightbars[i].get_width()<<","<<lightbars[i].get_height()<<endl;
            if(fabs(lightbars[i].get_center().y-lightbars[j].get_center().y)> 3*max_width ||
               fabs(lightbars[i].get_center().x-lightbars[j].get_center().x)>1*max_width){
                continue;
            }
            vector<LightBar> temp_group;
            temp_group.push_back(lightbars[i]);
            temp_group.push_back(lightbars[j]);
            halfbase.push_back(temp_group);
            circle(debug,lightbars[i].get_center(),10,Scalar(0,0,255),2);
            circle(debug,lightbars[j].get_center(),10,Scalar(0,0,255),2);
        }
    }
    vector<Point> base;
    vector<double> lenght;
    for(int i=0;i<halfbase.size();i++){
        for(int j = i+1;j<halfbase.size();j++){
            int max_width=pow(max(pow(halfbase[i][0].get_center().x-halfbase[i][1].get_center().x,2)+pow(halfbase[i][0].get_center().y-halfbase[i][1].get_center().y,2),
                                  pow(halfbase[j][0].get_center().x-halfbase[j][1].get_center().x,2)+pow(halfbase[j][0].get_center().y-halfbase[j][1].get_center().y,2)),0.5);
            LightBar footlight[2],headlight[2];
            footlight[0]= (halfbase[i][0].get_center().y>halfbase[i][1].get_center().y) ? halfbase[i][0] : halfbase[i][1];
            footlight[1]= (halfbase[j][0].get_center().y>halfbase[j][1].get_center().y) ? halfbase[j][0] : halfbase[j][1];
            headlight[0]= (halfbase[i][0].get_center().y<halfbase[i][1].get_center().y) ? halfbase[i][0] : halfbase[i][1];
            headlight[1]= (halfbase[j][0].get_center().y<halfbase[j][1].get_center().y) ? halfbase[j][0] : halfbase[j][1];
            if (footlight[0].get_area()<25  || footlight[1].get_area()<25) continue;
//            if(footlight[0].get_width()<1.5*footlight[0].get_height() ||
//                footlight[1].get_width()<1.5*footlight[1].get_height())
//                continue;
            if(fabs(footlight[0].get_center().y-footlight[1].get_center().y)>1.5*max_width ||
               fabs(headlight[0].get_center().y-headlight[1].get_center().y)>2*max_width ||
               fabs(footlight[0].get_center().x-footlight[1].get_center().x)>4*max_width ||
               fabs(footlight[0].get_center().x-footlight[1].get_center().x)<min(max_width/2,25) ||
               fabs(headlight[0].get_center().x-headlight[1].get_center().x)<min(max_width/2,25) ){
                continue;
            }
            int length_temp=pow(pow(footlight[0].get_center().x-footlight[1].get_center().x,2)+pow(footlight[0].get_center().y-footlight[1].get_center().y,2),0.5);
            cout<<length_temp<<endl;
            if (length_temp<29.5||length_temp>50.5)  continue;
            Point center((footlight[0].get_center().x+footlight[1].get_center().x)/2,(footlight[0].get_center().y+footlight[1].get_center().y)/2);
            base.push_back(center);
            lenght.push_back(length_temp);
        }
    }

    if(base.size()==0){
        cout<<"can not find base"<<endl;
        last_armor_position={0,0};
#ifdef LOB_DEBUG
        imshow("debug",debug);
        waitKey(1);
#endif
        return false;
    }else{
        Point out_base;
        double out_lenght;
        int mark=height_*height_+wigth_*wigth_+1;
        for(int i=0;i<base.size();i++){
            int center_dis=pow(last_base_.x-base[i].x,2)+pow(last_base_.y-base[i].y,2);
            if(mark>center_dis){
                out_base=base[i];
                out_lenght=lenght[i];
            }
        }
#ifdef LOB_DEBUG
        circle(debug,out_base,20,Scalar(255,0,0),2);
#endif
        base_armor_=out_base;
        last_base_=base_armor_;
        light_dis_=out_lenght;
        deviation_=out_lenght*2/3;

        last_armor_position=out_base;
#ifdef LOB_DEBUG
        imshow("debug",debug);
        waitKey(1);
#endif
        return true;
    }
}

void AirLob::DistanceFinder() {
//    if (z_distance_>7000){
//        return;
//    }
    z_distance_=world_light*cam_matrix_.at<double>(1,1)/light_dis_;
    //if(lost_target > 10){
    if(get_timenow()-last_time_ > 300){
        KF_[2].correct(z_distance_,false);
//        last_distance=z_distance_;
        get_target_=1;
    }else{
        if(get_target_>30){
            if(abs(last_distance-z_distance_)>800){
//            cout<<last_distance<<endl<<z_distance_<<endl;
                z_distance_=last_distance;
            }
        }else{
            get_target_++;
        }
        KF_[2].correct(z_distance_);
    }

    //VasualKalman(z_distance_,KF_[2].get_statePost(),2);
    z_distance_=KF_[2].get_statePost();
    last_distance=z_distance_;
    last_time_=get_timenow();
    cout<<"cam_dis     "<<z_distance_<<endl;

}


void AirLob::AngleResolver(SendMessage &send_message) {
    double yaw=-Pixel2Angle(wigth_/2-base_armor_.x+deviation_,'x',cam_matrix_);
    double pixel_pitch_angle=Pixel2Angle(height_/2-base_armor_.y,'y',cam_matrix_);
    Polor3f camera_polor={float(z_distance_+shoot_platform_.offset_lob),float(yaw),float(pixel_pitch_angle)};
    Point3f cmamera_point=to_sprectangular(camera_polor);
    Mat tvec=Mat(Point3d(cmamera_point));

    Vec3f diffeular={shoot_platform_.pitch+0.008f,shoot_platform_.yaw-0.017f,0};//云台角度补偿
    Mat rotmatix_diff=eulerAnglesToRotationMatrix(diffeular);//由欧拉角得到旋转矩阵(相机坐标到世界坐标系)

    Vec3d difftran={0,58.96,42.89};//相机到转轴的位置补偿,不同车不一样,理解为世界坐标系原点在相机坐标系下的位置,并方向取反
    Mat difftranvec=Mat(difftran);
    tvec=rotmatix_diff*(tvec+difftranvec);//相机坐标系中的点在世界坐标系中的位置,即是平移向量

    /////重力补偿/////
    Point3f world_point=Point3f(*(tvec.ptr<Point3d>()));//转化为Point3f
    float x1,y1;//平面直角坐标系,抛物线过点(x1,y1)
    y1=world_point.y+height_offset_;
    x1=sqrt(world_point.x*world_point.x+world_point.z*world_point.z);//平面坐标系中的x1,y1
    Vec2f phi=parabolasolve(Point2f(x1/10,y1/10),bullet_speed_);//过点(x1,y1)解抛物线方程,得到两个出射角的解

    float pitch=fabs(phi[0])<fabs(phi[1])?phi[0]:phi[1] ;

    send_message.yaw=atan2(world_point.x,world_point.z)-shoot_platform_.yaw;
    send_message.pitch=pitch-shoot_platform_.pitch;
    AngleCorrect(send_message.pitch);
    send_message.no_object=0;
    cout<<"bulletspeed    "<< bullet_speed_<<endl;
    cout<<"z_dis_         "<<z_distance_<<endl;
    cout<<"deviation      "<<deviation_<<endl;
    cout<<"x1             "<<x1<<endl;
    cout<<"y1             "<<y1<<endl;
    cout<<"worldx         "<<world_point.x<<endl;
    cout<<"worldy         "<<world_point.y<<endl;
    cout<<"sendpitch      "<<send_message.pitch<<endl;
}
#endif

void VasualKalman(int no_kalman_y, int kalman_y,int per) {
    static int x;
    static Mat kalman_nokalman_(500,2000,CV_8UC3,Scalar(255,255,255));
    x+=5;
    if(x>1999) {
        kalman_nokalman_=(500,2000,CV_8UC3,Scalar(255,255,255));

    }
    x=x%2000;
    int y_kalman;
    int y_nokalman;
    y_kalman=(kalman_y/500.);
    y_nokalman=(no_kalman_y/500.);
    Point kalman_point(x,y_kalman);
    Point no_kalman_point(x,y_nokalman);

    circle( kalman_nokalman_ , no_kalman_point, 5,Scalar(255, 0, 0) ,
            1, 8, 0 );
    circle( kalman_nokalman_ , kalman_point,5,Scalar(0, 0, 255) ,
            1, 8, 0 );
    //cout<<"kalman_point"<<kalman_point<<endl;
    //cout<<"no_kalman"<<no_kalman_point<<endl;
    imshow("kalman",kalman_nokalman_ );
    //imshow("????",nokalman_picture);
    waitKey(1);
}




//void VasualKalman(int no_kalman_y, int kalman_y,int per=1) {
//    static int x;
//    static Mat kalman_nokalman_(500,2000,CV_8UC3,Scalar(255,255,255));
//    x+=5;
//    if(x>1999) {
//        kalman_nokalman_=(500,2000,CV_8UC3,Scalar(255,255,255));
//
//    }
//    x=x%2000;
//    int y_dis = (kalman_y-no_kalman_y)*per;
//
//
//
//    if(y_dis>=0){
//        Point dis(x,y_dis);
//        circle( kalman_nokalman_ ,dis,5,Scalar(0, 0, 255) ,
//                1, 8, 0 );
//    }else{
//        Point dis(x,-y_dis);
//        circle( kalman_nokalman_ , dis, 5,Scalar(255, 0, 0) ,
//                1, 8, 0 );
//    }
//
//
//    //cout<<"kalman_point"<<kalman_point<<endl;
//    //cout<<"no_kalman"<<no_kalman_point<<endl;
//    imshow("kalman",kalman_nokalman_ );
//    //imshow("????",nokalman_picture);
//    waitKey(1);
//}