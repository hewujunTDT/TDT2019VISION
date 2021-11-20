#include"targetresolver.h"
#include "debug.h"


//////////////////////////////TargetResolver/////////////////////////////////////////

PointList G_PointList=PointList();//全局变量初始化,结构PointList储存了大符和装甲板在物体坐标系中的点

TargetResolver::TargetResolver(Camera* camera){
    camera_matrix_=camera->get_matrix();//内参
    dist_coeffs_=camera->get_dist_coeffs();//畸变矩阵
}

void TargetResolver::AbsoluteCoordinateTransformation(Mat &rvec, Mat &tvec,ShootPlatform shootPlatform,bool to_world,int flag) {
    Vec3f diffeular;//0,装甲板，1，能量机关，2，吊射
    if(flag==1) {
        diffeular= {shootPlatform.pitch, shootPlatform.yaw - 0.003f, 0};//云台角度补偿
    }
    if(flag==0)
    {
        diffeular={shootPlatform.pitch+0.01f, shootPlatform.yaw - 0.0035f, 0};//云台角度补偿
    }
    if(flag==2)
    {
        diffeular={shootPlatform.pitch, shootPlatform.yaw - 0.003f, 0};//云台角度补偿
    }
    Vec3d difftran={0,-4.5,11.9};//相机到转轴的位置补偿,不同车不一样,理解为世界坐标系原点在相机坐标系下的位置,并方向取反
    if(to_world){
        Mat difftranvec=Mat(difftran);
        Mat rotmatix_diff ,rotmatix;

        rotmatix_diff=eulerAnglesToRotationMatrix(diffeular);//由欧拉角得到旋转矩阵(相机坐标到世界坐标系)
        tvec=rotmatix_diff*(tvec+difftranvec);//相机坐标系中的点在世界坐标系中的位置,即是平移向量
        Rodrigues(rvec,rotmatix);//旋转向量转换为旋转矩阵(物体到相机)
        Rodrigues(rotmatix_diff*rotmatix,rvec);//待验证(物体到相机*相机到世界 得到物体到世界的旋转矩阵 然后得到旋转向量)
#ifdef DEBUG
        if(G_setval.WORK) {
            G_buffinformation.diffeular = diffeular;
            G_buffinformation.difftran = difftran;
            G_armorinformation.diffeular = diffeular;
            G_armorinformation.difftran = difftran;
        }
#endif
    } else{
        diffeular=-diffeular;
        difftran=-difftran;
        Mat difftranvec=Mat(difftran);
        Mat rotmatix_diff ,rotmatix;

        rotmatix_diff=eulerAnglesToRotationMatrix(diffeular, false);//由欧拉角得到旋转矩阵(相机坐标到世界坐标系)
        tvec=rotmatix_diff*tvec+difftranvec;//相机坐标系中的点在世界坐标系中的位置,即是平移向量
        Rodrigues(rvec,rotmatix);//旋转向量转换为旋转矩阵(物体到相机)
        Rodrigues(rotmatix_diff*rotmatix,rvec);//待验证(物体到相机*相机到世界
    }


}

float TargetResolver::PnpAssess(Mat matrix,Mat dist_coeffs,Mat rot_vector,Mat tran_vector,vector<Point3f> worldPoints,vector<Point2f> imagePoint){
    //PnpAssess要好好做一下,可以把数字的二值图考虑进去(现在相机帧率比较高,六米之内不会糊掉),没有可靠的评估后面很难进行.
    vector<Point2f> image_points;
    projectPoints(worldPoints,rot_vector,tran_vector,matrix,dist_coeffs,image_points);//映射到像素坐标系上
    return  getSimilarity(Mat(image_points),Mat(imagePoint));
}

//////////RobotTarget///////////

int TargetResolver::calc_linkid(vector<Armor> &armors) {
    int linkid=0;//linkid,意为当前序列为0装甲板在last_input_armors_中的序列
    if(!last_input_armors_.empty()&&!armors.empty()){
        if(last_input_armors_[0].get_robot_type()==armors[0].get_robot_type()){
            bool samejudge= false;
            for(int i0=0;i0<last_input_armors_.size();i0++){
                for(int i1=0;i1<armors.size();i1++){
                    const RotatedRect &a=last_input_armors_[i0].get_numberstiker().get_rotated_rect();
                    const RotatedRect &b=armors[i1].get_numberstiker().get_rotated_rect();
                    float _karea=a.size.area()/b.size.area();
                    if(_karea>2||_karea<0.5)continue;//矩形大小不能差太多
                    float dist=getDistance(a.center,b.center);
                    if(dist<(float(a.boundingRect().height)+float(b.boundingRect().height))/1.2){//距离近
                        samejudge= true;
                        linkid=(i0-i1+4)%4;
                        break;
                    }
                }
                if(samejudge)break;
            }
            if(!samejudge){//没有找到任何一对相同的装甲板
                const RotatedRect &a=last_input_armors_[0].get_armor_rotatedrect();
                const RotatedRect &b=armors[0].get_armor_rotatedrect();
                if(a.center.x>b.center.x){
                    linkid=3;
                } else{
                    linkid=1;
                }
            }
        }
    }
    return linkid;
}

RobotTarget TargetResolver::RobotResolver(vector<Armor> &armors,ReciveMessage &recive_message){
    //装甲板从左到右排序
    sort(armors.begin(),armors.end(),[](Armor a,Armor b)->
            bool{ return a.get_numberstiker().get_center().x < b.get_numberstiker().get_center().x;});

    vector <FinitePlane> planes(4);
    uint ucount=0;
    Mat rvec,tvec;//旋转向量
    for(Armor &armor:armors){
        if(armor.empty()) continue;
        vector<Point2f> pnp_points;//pnp_points长度为11且与WorldPoints_List顺序相同
        vector<Point3f> world_points;//用于pnp的物体坐标系中的点
        vector<Point2f> image_points;//用于pnp的像素坐标系中的点

        armor.calc_pnppoints(pnp_points);//Armor成员函数得到自身可以用去结算的点
        for(int i=0;i<pnp_points.size();i++){
            if(pnp_points[i].x==-1) continue;
            image_points.push_back(pnp_points[i]);
            world_points.push_back(G_PointList.RobotWorldPoints_List[int(armor.get_robot_type())][i]);
        }//获得相对应世界坐标点和图像坐标点
        try {//todo 现在pnp有偶然性报错的情况,原因位置,再次出现报错时,会cout图像点,记得观测一下
            solvePnP(world_points, image_points,camera_matrix_, dist_coeffs_, rvec, tvec, false);//调用PNP函数
        }catch (...){//如果报错会进入一下代码
            cout<<"error,pnp fail"<<endl;
            cout<<image_points<<endl;
            continue;
        }

#ifdef DEBUG
        if(G_setval.WORK) {
            G_armorinformation.camrvec=rvec.clone();
            G_armorinformation.camtvec=tvec.clone();
        }
#endif

//        float pnp_assess=PnpAssess( camera_matrix_,dist_coeffs_,rvec,tvec, world_points,image_points);//评估pnp
        TargetResolver::AbsoluteCoordinateTransformation(rvec,tvec, recive_message.shoot_platform, true,0);//统一坐标到绝对坐标系
        FinitePlane plane(rvec,tvec,armor,1);//构造一个平面类,若干个FinitePlane(即装甲板平面)组成一个RobotTarget目标
        planes[ucount]=plane;
        ucount++;
    }


    if(planes.empty())return RobotTarget();//由于armordetect的处理理论上不会为空出现,但是模块自身不能完全信赖外部模块
    RobotTarget target(rvec,tvec,planes,armors[0].get_robot_type(),recive_message);//构造目标类
    int linkid=calc_linkid(armors);
    target.set_linkid(linkid);
    last_input_armors_=armors;//更新last_input_armors_

    return target;
}

//////////BuffTarget///////////

BuffTarget TargetResolver::EnergyBuffResolver(vector<EnergyBuffArmor> &buff_armors, ReciveMessage &recive_message) {

    if(buff_armors[0].empty()||buff_armors[0].get_type()!=FlowWater) return BuffTarget();
    vector<Point2f> image_points;
    vector<Point3f> world_points;
//    for(int i=0;i<buff_armors.size();i++){
//        if(buff_armors[i].empty())continue;
//        if(buff_armors[i].get_type()==FlowWater&&i==0){
//            //构造图像点
//            image_points.push_back(buff_armors[0].get_circle());//圆心
//            image_points.push_back(buff_armors[0].center());//装甲版中心点
//            image_points.push_back(buff_armors[0].tl());//装甲版左上顶点
//            image_points.push_back(buff_armors[0].tr());//装甲版右上顶点
//            //构造现实点
//            world_points.insert(world_points.end(),G_PointList.EBWorldPoints_List[i].begin(),G_PointList.EBWorldPoints_List[i].end());
//
//        } else if(buff_armors[i].get_type()==NoFlowWater&&i>0){
//            if(G_PointList.EBWorldPoints_List[i].size()!=2)continue;
//            //构造图像点qqq
//            image_points.push_back((buff_armors[i].tl()+buff_armors[i].bl())/2);//装甲版中心点
//            image_points.push_back((buff_armors[i].tr()+buff_armors[i].br())/2);//装甲版中心点
//            //构造现实点
//            world_points.insert(world_points.end(),G_PointList.EBWorldPoints_List[i].begin(),G_PointList.EBWorldPoints_List[i].end());
//        } else{ continue;}
//
//    }

    image_points.push_back(buff_armors[0].get_circle()+Point2f(0,0));//圆心
    image_points.push_back(buff_armors[0].get_circle()+Point2f(50,50));//装甲版中心点
    image_points.push_back(buff_armors[0].get_circle()+Point2f(-50,-50));//装甲版中心点
    image_points.push_back(buff_armors[0].get_circle()+Point2f(50,-50));//装甲版左上顶点
    image_points.push_back(buff_armors[0].get_circle()+Point2f(-50,50));//装甲版右上顶点

    world_points.emplace_back(0,0,0);//圆心
    world_points.emplace_back(20,20,0);//圆心
    world_points.emplace_back(-20,-20,0);//圆心
    world_points.emplace_back(20,-20,0);//圆心
    world_points.emplace_back(-20,20,0);//圆心


    //开始解算
    Mat rvec,tvec;//旋转向量 旋转矩阵
    solvePnP(world_points,image_points,camera_matrix_,dist_coeffs_,rvec,tvec,false);//pnp解算

    //定距离
    Point3f camera_point = Point3f(*(tvec.ptr<Point3d>()));//转化为Point3f
    Polor3f camera_polor=to_polor(camera_point);
    camera_polor.distance=740;
    camera_point=to_sprectangular(camera_polor);
    tvec=Mat(Point3d(camera_point));


#ifdef DEBUG
    if(G_setval.WORK) {
        G_buffinformation.camrvec=rvec.clone();
        G_buffinformation.camtvec=tvec.clone();
    }
#endif
//    float pnp_assess=PnpAssess( camera_matrix_,dist_coeffs_,rvec,tvec, world_points,image_points);//评估
    float pnp_assess=1;//评估
    AbsoluteCoordinateTransformation(rvec,tvec, recive_message.shoot_platform,true,1);//统一坐标到绝对坐标系


    Point3f world_point = Point3f(*(tvec.ptr<Point3d>()));//转化为Point3f
    bool on_bridge=fabs(world_point.y)<200;//判断是否在桥上
    if(true){
        Mat rot_mat;
        //定欧拉角
        float pixel_angle= kPI+buff_armors[0].get_angle();
        Vec3f eular_angle={-0.15f,0,pixel_angle};
        if(!on_bridge)
        {
            eular_angle={0,0,pixel_angle};
        }
        float pixel_r= getDistance(buff_armors[0].center(),buff_armors[0].get_circle());

        Mat rot_vector_cam,tran_vector_cam;
        float diff_dis=100;
        Vec3f eular_angle_tmp= eular_angle;

        bool good= false;

        for(int i=0;i<20;i++){

            vector<Point2f> pro_impoints;
            //矫正picth和yaw欧拉角
//            if(i<3)continue;
            if(on_bridge&&i>0&&i<18){
                Vec3f eular_angle_copy=eular_angle;
                float stride=0.015f;
                good= false;


                eular_angle_tmp[0]+=stride;


                rot_mat=eulerAnglesToRotationMatrix(eular_angle_tmp, true);
                Rodrigues(rot_mat, rot_vector_cam);
                rvec=rot_vector_cam.clone();
                rot_vector_cam=rot_vector_cam.clone();
                tran_vector_cam=tvec.clone();
                AbsoluteCoordinateTransformation(rot_vector_cam,tran_vector_cam,recive_message.shoot_platform, false,1);
                pro_impoints.clear();
                projectPoints(G_PointList.EBWorldPoints_List[0],rot_vector_cam,tran_vector_cam,camera_matrix_,dist_coeffs_,pro_impoints);
                float tmp_diff_dis=getDistance(pro_impoints[1],pro_impoints[0])-pixel_r;
//                cout<<getDistance(pro_impoints[1],pro_impoints[0])<<"ee"<<pixel_r<<"dad"<<eular_angle_tmp<<endl;
                if(fabs(tmp_diff_dis)<=diff_dis){
                    good= true;
                    diff_dis=fabs(tmp_diff_dis);
                    eular_angle=eular_angle_tmp;
                }

            } else{
                good= true;

                rot_mat=eulerAnglesToRotationMatrix(eular_angle, true);
                Rodrigues(rot_mat, rot_vector_cam);
                rot_vector_cam=rot_vector_cam.clone();
                tran_vector_cam=tvec.clone();
                AbsoluteCoordinateTransformation(rot_vector_cam,tran_vector_cam,recive_message.shoot_platform, false,1);
                pro_impoints.clear();
                projectPoints(G_PointList.EBWorldPoints_List[0],rot_vector_cam,tran_vector_cam,camera_matrix_,dist_coeffs_,pro_impoints);
            }

            Point diff=pro_impoints[1]-pro_impoints[0];
            float angle= static_cast<float>(atan2(diff.x,-diff.y));
            eular_angle[2]+=getDiffangle(buff_armors[0].get_angle(),angle);

            rot_mat=eulerAnglesToRotationMatrix(eular_angle,true);
            Rodrigues(rot_mat, rot_vector_cam);

            if(good){
                rvec=rot_vector_cam.clone();
            }

            if(getDistance(pro_impoints[1],buff_armors[0].get_rotatedrect().center)<0.3){
                break;
            }
        }
    }

    if(true){
        AbsoluteCoordinateTransformation(rvec,tvec, recive_message.shoot_platform,false,1);//还原到相机坐标系

        //矫正距离
        float pixel_r= getDistance(buff_armors[0].center(),buff_armors[0].get_circle());
        for(int i=0;i<4;i++){

            vector<Point2f> pro_impoints;
            projectPoints(G_PointList.EBWorldPoints_List[0],rvec,tvec,camera_matrix_,dist_coeffs_,pro_impoints);
            float diff_r=getDistance(pro_impoints[1],pro_impoints[0])-pixel_r;

            camera_point = Point3f(*(tvec.ptr<Point3d>()));//转化为Point3f
            camera_polor=to_polor(camera_point);
            camera_polor.distance+=diff_r;
            camera_point=to_sprectangular(camera_polor);
            tvec=Mat(Point3d(camera_point));
        }

        //真正的世界坐标系
        AbsoluteCoordinateTransformation(rvec,tvec, recive_message.shoot_platform,true,1);//统一坐标到绝对坐标系
    }


    FinitePlane plane(rvec,tvec,buff_armors[0],pnp_assess);//流水灯buff_armors构造FinitePlane
    vector<FinitePlane> planes={plane};//为了保持一致性 而且以后可能会涉及到其它的BuffArmor 所以用vector构造
    BuffTarget buffTarget(rvec,tvec,planes,recive_message);//大神符目标
    image_points.clear();world_points.clear();

    float instant_angle = buff_armors[0].get_angle();
    buffTarget.set_instant_angle(instant_angle);

    return buffTarget;
}