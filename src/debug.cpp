#define CVUI_IMPLEMENTATION
#include <iostream>
#include "macro.h"
#include "camera.h"
#include "debug.h"
#include "cvui.h"
#include "readparameter.h"
#include "targetresolver.h"
#define WINDOW_NAME "DEBUG"
#define SRCWIDTH 880
//#define SRCWIDTH 825
#define  MaxSize 150
//////////////////////////////////////////////////////////////////////////////////////////

string doubleToString(const double &dbNum){
    char *chCode;
    chCode = new(std::nothrow)char[20];
    sprintf(chCode, "%.2lf", dbNum);  // .2 是控制输出精度的，两位小数
    string strCode(chCode);
    delete []chCode;

    return strCode;

}
///////////////////////////////////////////////////////////////////////////////////////////

ArmorDebugInformation G_armorinformation=ArmorDebugInformation();
BuffDebugInformation G_buffinformation=BuffDebugInformation();
SetVal G_setval=SetVal();
Debugger:: Debugger(Mat camera_matrix,Mat dist_coeffs,GlobalParameter parameter){
    *setval_.dilatesize_val=parameter.EnergyBuffDetect_dialesize;
    *setval_.erodesize_val=parameter.EnergyBuffDetect_erodesize;
    *setval_.buff_split_threval=parameter.EnergyBuffDetect_split_thre;
    *setval_.buff_gray_threval=parameter.EnergyBuffDetect_gray_thre;
    *setval_.lightbar_threval=parameter.ArmorDetect_lightbarthre;
    *setval_.cam_gamma=parameter.cam_parameter[0].gamma;
    *setval_.cam_gain=parameter.cam_parameter[0].gain;
    *setval_.cam_brightness=parameter.cam_parameter[0].brightness;
    setval_.WORK=parameter.DebugWork;

    camera_matrix_=camera_matrix;
    dist_coeffs_=dist_coeffs;
    cvui::init(WINDOW_NAME);}
Point3f Debugger::worldtocamera(Point3f worldpoint,Vec3f diffeular,Vec3d difftran){
    Mat predictp=Mat(Point3d(worldpoint));
    diffeular=-diffeular;
    difftran=-difftran;
    Mat rotmatix_diff=eulerAnglesToRotationMatrix(diffeular, false);
    Mat difftranvec=Mat(difftran);
    Mat camerap=rotmatix_diff*predictp+difftranvec;
    camerap.convertTo(camerap,CV_32F);

    Point3f camera = *(camerap.ptr<Point3f>());
    return camera;
}
void Debugger::clearall() {
    for(int i=0;i<armor_infomations_.size();i++){
        delete  armor_infomations_[i]->src;
        delete armor_infomations_[i]->lightbar_thre;
        delete armor_infomations_[i];
    }
    for(int i=0;i<buff_infomations_.size();i++){
        delete  buff_infomations_[i]->src;
        delete buff_infomations_[i]->binary_img;
        delete buff_infomations_[i]->subcolor_img;
        delete buff_infomations_[i];
    }
    vector<BuffDebugInformation*>().swap(buff_infomations_);
    vector<ArmorDebugInformation*>().swap(armor_infomations_);
    destroyAllWindows();
}

void Debugger::CorrectAll(const SetVal &setval, Camera*camera, GlobalParameter &Parameter)  {
    if(G_setval.cam_set){
        camera->set_brightness(*G_setval.cam_brightness);
        camera->set_gain(int(*G_setval.cam_gain));
        camera->set_gamma(float(*G_setval.cam_gamma));
        Parameter.cam_parameter[0].brightness=*G_setval.cam_brightness;
        Parameter.cam_parameter[0].gain=int(*G_setval.cam_gain);
        Parameter.cam_parameter[0].gamma=float(*G_setval.cam_gain);

        Mat tip=cv::Mat(cv::Size(150, 50), CV_8UC3);
        tip=Scalar(49,52,49);
        String text1="Succcess";
        cv::putText(tip, text1,Point(15,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(201,198,201), 2, 8, 0);
        imshow("tip",tip);
        waitKey(500);
        cv::destroyWindow("tip");
    }
    Parameter.ArmorDetect_lightbarthre=*setval.lightbar_threval;
    Parameter.EnergyBuffDetect_split_thre=*setval.buff_split_threval;
    Parameter.EnergyBuffDetect_gray_thre=*setval.buff_gray_threval;
    Parameter.EnergyBuffDetect_erodesize=*setval.erodesize_val;
    Parameter.EnergyBuffDetect_dialesize=*setval.dilatesize_val;

}
///////////////////////////////////////ArmorDebugInformation////////////////////////////////////////////////////

void Debugger::set_armor_infomation(ArmorDebugInformation &information)  {
    ArmorDebugInformation *tmp=new ArmorDebugInformation(information);
    if(armor_infomations_.size()>MaxSize){
        delete  armor_infomations_[0]->src;
        delete armor_infomations_[0]->lightbar_thre;
        delete armor_infomations_[0];
        armor_infomations_.erase(armor_infomations_.begin());
    }
    armor_infomations_.push_back(tmp);
}
void Debugger::draw(ArmorDebugInformation &information) {
    frame = cv::Mat(cv::Size(SRCWIDTH+480,690), CV_8UC3);
    frame=cv::Scalar(49, 52, 49);
    vector<Armor> armors=information.armors;
    vector<LightBar> lightbars=information.lightbars;

    main_src_=information.src->clone();
    if(!gray_check){
        if(armordetect_check){
            rectangle(main_src_,information.roi_area,Scalar(100,100,0),1);
            for (int i=0;i<lightbars.size();i++){
                Point2f* vertices = new cv::Point2f[4];
                lightbars[i].get_rotatedrect().points(vertices);
                //逐条边绘制
                for (int j = 0; j < 4; j++) {
                    cv::line(main_src_, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),2);
                }
                delete  [] vertices;
                String text1=to_string(lightbars[i].get_judge_search());

                cv::putText(main_src_, text1,lightbars[i].tr(), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, 1, 0);

            }
            for (int i=0;i<armors.size();i++){

                Point2f* vertices = new cv::Point2f[4];
                armors[i].get_armor_rotatedrect().points(vertices);

                //逐条边绘制
                Scalar sca;
                sca=cv::Scalar(150, 250, 250);

                if(armors[i].get_robot_type()==TYPEUNKNOW){
                    sca=Scalar(0,60,50);
                }
                if(armors[i].get_similar_to_last()>0.8){
                    sca=Scalar(0,200,100);
                }
                if(armors[i].get_form()[0]&&armors[i].get_form()[1]){
                    sca[0]+=100;
                }

                if(armors[i].get_robot_type()!=TYPEUNKNOW){
                    for (int j = 0; j < 4; j++)
                    {
                        cv::line(main_src_, vertices[j], vertices[(j + 1) % 4], sca,1);
                    }
                    String text1=to_string( int(armors[i].get_robot_type()));
                    cv::putText(main_src_, text1,armors[i].get_armor_rotatedrect().boundingRect().tl(), cv::FONT_HERSHEY_COMPLEX, 0.8, sca, 2, 8, 0);
                    vector<Point2f> pnppoints;
                    armors[i].calc_pnppoints(pnppoints);
                    for(int j=0;j<pnppoints.size();j++){
                        if(pnppoints[j].x!=-1)
                            cv::circle(main_src_,pnppoints[j],6,Scalar(234,232,160),2);

                    }
                    pnppoints.clear();
                } else{
                    for (int j = 0; j < 4; j++){
                        cv::line(main_src_, vertices[j], vertices[(j + 1) % 4], sca,2);
                    }
                }
                if(armors[i].get_robot_type()!=TYPEUNKNOW){
                    if(!armors[i].get_left_lightbar().empty()){
                        cv::line(main_src_, armors[i].get_left_lightbar().get_center(), armors[i].get_numberstiker().get_center(), Scalar(124,234,54),2);
                    }
                    if(!armors[i].get_right_lightbar().empty()){
                        cv::line(main_src_, armors[i].get_right_lightbar().get_center(), armors[i].get_numberstiker().get_center(), Scalar(124,234,54),2);
                    }
                }

                delete  [] vertices;

            }
        }

        RobotTarget target=information.target;
        if(!target.empty()){
            vector<Point3f> worldpoints=G_PointList.RobotWorldPoints_List[int(target.get_robot_type())];
            for(int i=0;i<worldpoints.size();i++){
                Point3f tmp=worldpoints[i];
                Mat aa= Mat(Point3d(tmp));
                Mat rotmatix;
                Rodrigues(information.target.get_rvec(), rotmatix);//旋转向量转换为旋转矩阵

                Mat worldp=rotmatix*aa+information.target.get_tvec();
                Point3f a = Point3f(*(worldp.ptr<Point3d>()));
                Point3f predict_point = worldtocamera(a,information.diffeular,information.difftran);
                worldpoints[i]=predict_point;
            }

            Mat rv=Mat::zeros(Size(3,1),CV_32F);
            Mat tv=Mat::zeros(Size(3,1),CV_32F);
            vector<Point2f> impoints;
            projectPoints(worldpoints,rv,tv,camera_matrix_,dist_coeffs_,impoints);
            Scalar scalar={0,255,255};

            if(targetsolve_check){
                for(int i=0;i<impoints.size();i++){

                    cv::circle(main_src_,impoints[i],5,scalar,-1);
                }
            }

            impoints.clear();
            vector<Point2f> pre_impoints;
            vector<Point3f> pre_worldPoints;
            Point3f axis_point = worldtocamera(to_sprectangular(information.axis_polor_),information.diffeular,information.difftran);
            Point3f o_p = worldtocamera(to_sprectangular(information.oriented_polor_),information.diffeular,information.difftran);
            pre_worldPoints.push_back(axis_point);

            pre_worldPoints.push_back(o_p);
            for(int i1=0;i1<4;i1++){
                if(information.predict_polors_[i1].distance==-1)continue;
                Point3f predict_point = worldtocamera(to_sprectangular(information.predict_polors_[i1]),information.diffeular,information.difftran);
                pre_worldPoints.push_back(predict_point);

            }

            projectPoints(pre_worldPoints,rv,tv,camera_matrix_,dist_coeffs_,pre_impoints);

            if(predict_check){
                Scalar sca;
                for(int i=0;i<pre_impoints.size();i++) {

                    if(i==0){
                        sca=cv::Scalar(255, 100, 0);
                    } else if(i==1){
                        sca=cv::Scalar(0, 100, 255);

                    } else{
                        sca=cv::Scalar(0, 255, 0);

                    }

                    cv::line(main_src_, pre_impoints[i] + Point2f(-8, -8), pre_impoints[i] + Point2f(8, 8),
                             sca, 2);
                    cv::line(main_src_, pre_impoints[i] + Point2f(8, -8), pre_impoints[i] + Point2f(-8, 8),
                             sca, 2);
                }
            }


            impoints.clear();
            Point3f predict_point=to_sprectangular(information.oriented_polor_);
            ShootPlatform platform=information.reciveMessage.shoot_platform;
            float dist_z= sqrt(predict_point.z*predict_point.z+predict_point.x*predict_point.x);
            float t=dist_z/(cos(platform.pitch)*2.7);
            float dist_y=-tan(platform.pitch)*dist_z+0.00098*t*t/2;
            float dist_x=dist_z* sin(platform.yaw);
            dist_z=dist_z* cos(platform.yaw);
            dist_z=MAX(80,dist_z);
            Point3f firecneter=Point3f(dist_x,dist_y,dist_z);

            Point3f firepoint = worldtocamera(firecneter,information.diffeular,information.difftran);
            Point3f firecneterleft=firepoint+Point3f(-information.diffsize.width,0,0);
            Point3f firecneterdown=firepoint+Point3f(0,-information.diffsize.height,0);

            worldpoints={firepoint,firecneterleft,firecneterdown};
            projectPoints(worldpoints,rv,tv,camera_matrix_,dist_coeffs_,impoints);
            RotatedRect box=RotatedRect(impoints[0],Size2f(impoints[0].x-impoints[1].x,impoints[0].y-impoints[2].y),0);
            if(fire_check){
                Rect brect=box.boundingRect();
                if(RectSafety(brect,main_src_.size())){
//                    ellipse(main_src_, box, Scalar(0,0,255), 1, CV_AA);
                }
            }
            worldpoints.clear();
            impoints.clear();
            pre_impoints.clear();
            pre_worldPoints.clear();
        }
    } else{
        cvtColor(main_src_,gray_image_,COLOR_BGR2GRAY);

        binary_image_=gray_image_.clone();
        information.lightbar_thre->copyTo(binary_image_(information.roi_area));
        cvtColor(gray_image_,gray_image_,COLOR_GRAY2BGR);
        cvtColor(binary_image_,binary_image_,COLOR_GRAY2BGR);
    }


    if(ml_check){
        sort(armors.begin(),armors.end(),
             [](Armor a,Armor b)->
                     bool{return int(a.get_robot_type())> int(b.get_robot_type());});
        ml_images_=Mat::zeros(Size(28*2,28*4),CV_8U);
        ml_images_=Scalar(49, 52, 49);
        Rect roiiter=Rect(Point(0,0),Size(28,28));
        for(int kk=0;kk<MIN(armors.size(),8);kk++){
            Mat armorimg=armors[kk].get_ml_roi();
            resize(armorimg,armorimg,Size(28,28));
            armorimg.copyTo(ml_images_(roiiter));
            if(kk%2==0){
                roiiter+=Point(28,0);
            } else{
                roiiter+=Point(-28,28);
            }
        }
        cvtColor(ml_images_,ml_images_,COLOR_GRAY2BGR);
//    resize(ml_images_,ml_images_,Size(128*2,128*4));


        for(int kk=0;kk<MIN(armors.size(),12);kk++){
            String text1=to_string(int(armors[kk].get_robot_type()));
            Point loc;
            if(kk%2==0){
                loc=Point(5,kk*14+8);
            } else{
                loc=Point(5+28,(kk-1)*14+8);
            }
            if(int(armors[kk].get_robot_type())!=0){
                cv::putText(ml_images_, text1,loc, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 255), 1, 1, 0);

            } else{
                cv::putText(ml_images_, "0",loc, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 155, 0), 1, 1, 0);
            }
        }

    }

    armors.clear();
    lightbars.clear();

    if(kalman_check){

        kalman_image_ = cv::Mat(950, 900, CV_8UC3);
        kalman_image_=Scalar(46,36,32);
        Rect arearect=Rect(Point(kalman_image_.cols-100,10+5),Size(85,60));
        cvui::rect(kalman_image_, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xffffff);
        circle( kalman_image_ , arearect.tl()+Point(15,15), 8,Scalar(220, 233, 225) ,1, 8, 0 );
        circle( kalman_image_ , arearect.tl()+Point(15,45), 8,Scalar(41, 222, 244) ,1, 8, 0 );
        cvui::printf(kalman_image_, arearect.x+25, arearect.y+15-5, "Measure");
        cvui::printf(kalman_image_, arearect.x+25, arearect.y+45-5, "State");
        std::vector<float > measurevalues;
        std::vector<float > prevalues;
        std::vector<float > postvalues;
        for(int k=0;k<5;k++){
            for(ArmorDebugInformation* & tmp:armor_infomations_){
                measurevalues.push_back(tmp->kf_[k].get_measurement());
                postvalues.push_back(tmp->kf_[k].get_statePost());
                prevalues.push_back(tmp->kf_[k].get_statePre());
                if(tmp==&information) break;
            }
            float uplimit=3.14;
            float downlimit=-3.14;

            int axis_y=(k)*180+100;
            int h=180;
            cv::line(kalman_image_,Point(80,axis_y),Point(700,axis_y),Scalar(56,255,33),1);
            cvui::printf(kalman_image_, 720,axis_y-15, "Measure   (%.3f)", information.kf_[k].get_measurement());
            cvui::printf(kalman_image_, 720,axis_y, "StatePre  (%.3f)", information.kf_[k].get_statePre());
            cvui::printf(kalman_image_, 720,axis_y+15, "StatePost (%.3f)", information.kf_[k].get_statePost());
            for(int i=0;i<measurevalues.size();i++ ){
                circle(kalman_image_ , Point(i*4+80, (axis_y-(measurevalues[i]-(uplimit+downlimit)/2)*h/(uplimit-downlimit))), 3,Scalar(220, 233, 225) ,
                       1, 8, 0 );

                circle( kalman_image_ , Point(i*4+80,(axis_y-(postvalues[i]-(uplimit+downlimit)/2)*h/(uplimit-downlimit))), 3,Scalar(41, 222, 244) ,
                        1, 8, 0 );
            }
            measurevalues.clear();
            prevalues.clear();
            postvalues.clear();
        }

    } else{

    }
}
void Debugger::arrange_layout(ArmorDebugInformation &information) {

    Rect arearect;
    Mat srccopy;
    int informationw=185;
    int pad=1;

    resize(*information.src,srccopy,Size(SRCWIDTH,cvRound(main_src_.size().height*SRCWIDTH/main_src_.size().width)));

    if(true){
        ///////////////////
        arearect=Rect(Point(5,20+5),Size(informationw,90));
        cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Cursor");
        int mx=cvui::mouse().x-(informationw+10);
        int my=cvui::mouse().y-25;
        mx=cvRound(mx*information.src->size().width/SRCWIDTH);
        my=cvRound(my*information.src->size().width/SRCWIDTH);
        if(mx<0||mx>information.src->size().width||my<0||my>information.src->size().height){
            mx=0;my=0;
        }
        cvui::printf(frame, arearect.x+5, arearect.y+20, "  X,Y (%d,%d)", mx, my);
        cvui::printf(frame, arearect.x+5, arearect.y+45, "  B,G,R (%d,%d,%d)", information.src->at<Vec3b>(my,mx)[0], information.src->at<Vec3b>(my,mx)[1],information.src->at<Vec3b>(my,mx)[2]);
        cvui::printf(frame, arearect.x+5, arearect.y+70, "  Time (%.3lf)ms", information.timenow);

        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);
        ///////////////////
        arearect=Rect(Point(arearect.x,arearect.br().y+20+5),Size(informationw,544));
        cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Informations");
        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);
        static int last_page0,last_page1,last_page2;
        last_page0=page0;last_page1=page1;last_page2=page2;
        cvui::checkbox(frame,arearect.x+pad+25,arearect.y+pad+10,"0", &page0);
        cvui::checkbox(frame,arearect.x+pad+75,arearect.y+pad+10,"1", &page1);
        cvui::checkbox(frame,arearect.x+pad+125,arearect.y+pad+10,"2", &page2);

        if(page0==1&&last_page0==0){
            page1=0;page0=1;page2=0;
        }else if(page1==1&&last_page1==0){
            page1=1;page0=0;page2=0;
        }
        else if(page2==1&&last_page2==0){
            page1=0;page0=0;page2=1;
        } else{
            page1=last_page1;page0=last_page0;page2=last_page2;

        }
        if(page0){

            arearect=Rect(arearect.x-pad+6,arearect.y+40,arearect.width+pad*2-12,90);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "     ReceiveMessage");
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Picth,Yaw (%.3f,%.3f)", information.reciveMessage.shoot_platform.pitch, information.reciveMessage.shoot_platform.yaw);
            cvui::printf(frame, arearect.x+8, arearect.y+50, "Lock_Command (%d)", information.reciveMessage.lock_command);
            cvui::printf(frame, arearect.x+8, arearect.y+70,information.reciveMessage.mode==0?"Mode (ArmorMode)":"Mode (EnergyBuffMode)");

            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "      SendMessage");

            cvui::printf(frame, arearect.x+8, arearect.y+30, "Picth,Yaw (%.3f,%.3f)", information.sendMessage.pitch*57.3, information.sendMessage.yaw*57.3);
            cvui::printf(frame, arearect.x+8, arearect.y+50, "Beat (%d)", information.sendMessage.beat);
            cvui::printf(frame, arearect.x+8+60, arearect.y+50,"NoObject (%d)" ,information.sendMessage.no_object);
            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,320, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "    Position&Posture");
            cv::line(frame,arearect.tl()+Point(10,25),arearect.tl()+Point(100,25),Scalar(178,190,137),1);
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Plane0Center(X,Y,Z)");
            Point3f plane0point(0,0,0);
            Polor3f plane0polor={0,0,0};
            Vec3f plane0eulur={0,0,0};
            Point3f predict_point={0,0,0};
            Polor3f predict_polor={0,0,0};
            if(!information.target.get_planes().empty()){
                plane0point=information.target.get_planes()[0].get_world_point();
                plane0polor=information.target.get_planes()[0].get_polor_point();
                plane0eulur=information.target.get_planes()[0].get_euler_angle();
                predict_point=to_sprectangular(information.oriented_polor_);
                predict_polor=information.oriented_polor_;
            }
            cvui::printf(frame, arearect.x+8, arearect.y+50, "(%.1f, %.1f, %.1f)", plane0point.x, plane0point.y,plane0point.z);
            cvui::printf(frame, arearect.x+8, arearect.y+70, "Plane0(Dist, Yaw, Pitch)");
            cvui::printf(frame, arearect.x+8, arearect.y+90, "(%.1f, %.3f, %.3f)", plane0polor.distance, plane0polor.yaw,plane0polor.pitch);
            cvui::printf(frame, arearect.x+8, arearect.y+110, "EularAngle(P,Y,R)");

            cvui::printf(frame, arearect.x+8, arearect.y+130, "(%.2f, %.2f, %.2f)", plane0eulur[0],plane0eulur[1],plane0eulur[2]);
            cv::line(frame,arearect.tl()+Point(10,145),arearect.tl()+Point(100,145),Scalar(178,190,137),1);

            cvui::printf(frame, arearect.x+8, arearect.y+150, "PredictPoint(X, Y, Z)");
            cvui::printf(frame, arearect.x+8, arearect.y+170, "(%.1f, %.1f, %.1f)", predict_point.x, predict_point.y,predict_point.z);
            cvui::printf(frame, arearect.x+8, arearect.y+190, "Predict(Dist, Yaw, Pitch)");
            cvui::printf(frame, arearect.x+8, arearect.y+210, "(%.1f, %.3f, %.3f)", predict_polor.distance,predict_polor.yaw,predict_polor.pitch);


        }
        else if(page1) {
            arearect=Rect(arearect.x-pad+6,arearect.y+40,arearect.width+pad*2-12,110);

            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "   CameraCoordinate");
            Mat camrvec=information.camrvec;
            Mat camtvec=information.camtvec;
            Vec3f eular=rvecToEulerAngles(camrvec);
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Tv (%.1f,%.1f,%.1f)", camtvec.at<double>(0),camtvec.at<double>(1),camtvec.at<double>(2));
            cvui::printf(frame, arearect.x+8, arearect.y+50, "Rv (%.2f,%.2f,%.2f)", camrvec.at<double>(0),camrvec.at<double>(1),camrvec.at<double>(2));
            cvui::printf(frame, arearect.x+8, arearect.y+70, "EularAngle(P,Y,R)");
            cvui::printf(frame, arearect.x+8, arearect.y+90, "(%.2f,%.2f,%.2f)", eular[0],eular[1],eular[2]);

            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,90);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "        Kalman");
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Measure   (%.3f)", information.kf_[4].get_measurement());
            cvui::printf(frame, arearect.x+8, arearect.y+50, "StatePre  (%.3f)", information.kf_[4].get_statePre());
            cvui::printf(frame, arearect.x+8, arearect.y+70, "StatePost (%.3f)", information.kf_[4].get_statePost());
            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "      FireCommand");

            cvui::printf(frame, arearect.x+8, arearect.y+30, "FireRadius (%.1f,%.1f)", information.diffsize.width, information.diffsize.height);
            cvui::printf(frame, arearect.x+8, arearect.y+50, "DiffSize (%.1f,%.1f)", information.realdiff.width, information.realdiff.height);
            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,140, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "  TemporaryParameters");
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Parameter0 (%.3f)", information.tmpparam0);
            cvui::printf(frame, arearect.x+8, arearect.y+50, "Parameter1 (%.3f)", information.tmpparam1);
            cvui::printf(frame, arearect.x+8, arearect.y+70, "Parameter2 (%.3f)", information.tmpparam2);
            cvui::printf(frame, arearect.x+8, arearect.y+90, "Parameter3 (%.3f)", information.tmpparam3);
            cvui::printf(frame, arearect.x+8, arearect.y+110, "Parameter4 (%.3f)", information.tmpparam4);
        }else if(page2){
            if(ml_check){
                Mat mlmat;
                resize(ml_images_,ml_images_,Size(cvRound(arearect.width*0.9),cvRound(arearect.width*2*0.9)));
                cvui::printf(frame, arearect.x+8, arearect.y+40, "      ml_images");

                arearect=Rect(Point(arearect.x+10,arearect.y+55),ml_images_.size());
                //                cvui::window(frame, arearect.x-pad ,arearect.y-20-pad ,arearect.width+pad*2,arearect.height+20+pad*2, "ml_imgs");
                cvui::rect(frame, arearect.x-5 ,arearect.y-20 ,arearect.width+10,arearect.height+22, 0xfce6c9);
                ml_images_.copyTo(frame(arearect));
            }
        }

    }
    ///////////////////
    if(gray_check){
        resize(gray_image_,gray_image_,Size(SRCWIDTH,cvRound(gray_image_.size().height*SRCWIDTH/gray_image_.size().width)));
        resize(binary_image_,binary_image_,Size(SRCWIDTH,cvRound(binary_image_.size().height*SRCWIDTH/binary_image_.size().width)));
        arearect=Rect(Point(informationw+10,20+5),gray_image_.size());
        cvui::window(frame, arearect.x-pad ,arearect.y-20-pad ,arearect.width+pad*2,arearect.height+20+pad*2, "Src");
        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);
        cvui::button(frame, arearect.x, arearect.y,binary_image_ ,gray_image_,srccopy);

    } else{

        resize(main_src_,main_src_,Size(SRCWIDTH,cvRound(main_src_.size().height*SRCWIDTH/main_src_.size().width)));

        arearect=Rect(Point(informationw+10,20+5),main_src_.size());
        cvui::window(frame, arearect.x-pad ,arearect.y-20-pad ,arearect.width+pad*2,arearect.height+20+pad*2, "Src");
        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);
        cvui::button(frame, arearect.x, arearect.y, main_src_, main_src_, srccopy);
    }
    ///////////////////
    static  bool  kalmanwin;

    if(kalman_check) {
        kalmanwin= true;

        imshow("kalman",kalman_image_);

    } else{
        if(kalmanwin){
            cv::destroyWindow("kalman");
            kalmanwin= false;
        }

    }

    ///////////////////

    arearect=Rect(Point(SRCWIDTH+informationw+15,5+20),Size(270,115));
    pad=20;
    cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Control");
    cvui::rect(frame, arearect.x, arearect.y, arearect.width , arearect.height, 0xffffff);
    ///////////////////////////////
    arearect=Rect(Point(SRCWIDTH+informationw+15,arearect.br().y+5+20),Size(arearect.width,175));
    pad=20;
    cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Form");
    cvui::rect(frame, arearect.x, arearect.y, arearect.width , arearect.height, 0xffffff);

    if(setval_.detectmode==ArmorMode) {
        cvui::button(frame, arearect.x+15, arearect.y+10,180,30, "ArmorMode");
        if(!pausejudge){
            if (cvui::button(frame, arearect.x+180+20, arearect.y+10,50,30, ">>")) {
                lastmode=EnergyBuffMode;
            }
        }
    }else{
        cout<<pausejudge<<endl;

        cvui::button(frame, arearect.x+15, arearect.y+10,180,30, "EnergyBuffMode");
        if(!pausejudge){
            if (cvui::button(frame, arearect.x+180+20, arearect.y+10,50,30, ">>")) {
                lastmode=ArmorMode;
            }
        }
    }

    static bool last_gray,last_normal;
    last_gray=gray_check;last_normal=normal_check;
    cvui::checkbox(frame,arearect.x+pad+80,195+pad,"Gray", &gray_check);
    cvui::checkbox(frame,arearect.x+pad,195+pad,"Normal", &normal_check);
    if((gray_check&&!last_gray)||(!normal_check&&last_normal)){
        normal_check=0,gray_check=1;
    }
    else if((normal_check&&!last_normal)||(!gray_check&&last_gray)) {

        normal_check=1;gray_check=0;
    } else{

        normal_check=last_normal;gray_check=last_gray;
    }

    if(!gray_check){
        cvui::checkbox(frame,arearect.x+pad,230+pad,"TargetSolve", &targetsolve_check);
        cvui::checkbox(frame,arearect.x+pad,260+pad,"ArmorDetect", &armordetect_check);
        cvui::checkbox(frame,arearect.x+pad,290+pad,"Kalman", &kalman_check);
        cvui::checkbox(frame,arearect.x+pad+140,230+pad,"FireCommand", &fire_check);
        cvui::checkbox(frame,arearect.x+pad+140,260+pad,"Predict", &predict_check);

    } else {
        cvui::checkbox(frame,arearect.x+pad,290+pad,"Kalman", &kalman_check);


    }
    ///////////////////////////////
    arearect=Rect(Point(SRCWIDTH+informationw+15,arearect.br().y+5+20),Size(arearect.width,320));
    pad=20;
    cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Tools");
    cvui::rect(frame, arearect.x, arearect.y, arearect.width , arearect.height, 0xffffff);
    static int tlast_page0,tlast_page1,tlast_page2;
    tlast_page0=tpage0;tlast_page1=tpage1;tlast_page2=tpage2;
    cvui::checkbox(frame,arearect.x+pad+30,arearect.y+pad,"0", &tpage0);
    cvui::checkbox(frame,arearect.x+pad+100,arearect.y+pad,"1", &tpage1);
    cvui::checkbox(frame,arearect.x+pad+170,arearect.y+pad,"2", &tpage2);

    if(tpage0==1&&tlast_page0==0){
        tpage0=1;tpage1=0;tpage2=0;
    }else if(tpage1==1&&tlast_page1==0){
        tpage0=0;tpage1=1;tpage2=0;
    }
    else if(tpage2==1&&tlast_page2==0){
        tpage0=0;tpage1=0;tpage2=1;
    } else{
        tpage0=tlast_page0;tpage1=tlast_page1;tpage2=tlast_page2;

    }
    if(tpage0){
        cvui::rect(frame, arearect.x +10,arearect.y+45 ,arearect.width-20,130, 0x9BCD9B);
        cvui::printf(frame,  arearect.x+30+5, arearect.y+60, "threhold:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+50,130, setval_.lightbar_threval, 20, 255);
        cvui::rect(frame, arearect.x +10,arearect.y+180 ,arearect.width-20,130, 0x9BCD9B);
        cvui::printf(frame, arearect.x+30+5, arearect.y+190, "tmpval0:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+180,130, setval_.tmpval0, (float)0, (float)100);
        cvui::printf(frame, arearect.x+30+5, arearect.y+250, "tmpval1:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+240,130, setval_.erodesize_val, 0, 5);

    }else if(tpage1){
        cvui::rect(frame, arearect.x +10,arearect.y+45 ,arearect.width-20,170, 0x9BCD9B);
        cvui::printf(frame,  arearect.x+80+5, arearect.y+55, "HikvisionCamera");
        cvui::counter(frame,arearect.x+20+5+100, arearect.y+80-5,setval_.cam_brightness,50);
        cvui::printf(frame,  arearect.x+20+5, arearect.y+80, "brightness");
        cvui::counter(frame,arearect.x+20+5+100, arearect.y+110-5,setval_.cam_gain,0.1);
        cvui::printf(frame,  arearect.x+20+5, arearect.y+110, "gain");
        cvui::counter(frame,arearect.x+20+5+100, arearect.y+140-5,setval_.cam_gamma,0.1);
        cvui::printf(frame,  arearect.x+20+5, arearect.y+140, "gamma");
        if(cvui::button(frame, arearect.x+25, arearect.y+165,220,40, "Set To HikvisionCamera")){
            setval_.cam_set= true;
        } else{
            setval_.cam_set= false;
        }

    } else if(tpage2) {
        cvui::rect(frame, arearect.x + 10, arearect.y + 45, arearect.width - 20, 170, 0x9BCD9B);
        if (cvui::button(frame, arearect.x + 25, arearect.y + 60, 220, 40, "Save Ml_Samples")) {
            for (int i = 0; i < armor_infomations_.size(); i++) {
                for (int j = 0; j < armor_infomations_[i]->armors.size(); j++) {
                    if (armor_infomations_[i]->armors[j].empty())continue;
                    Mat tmp = armor_infomations_[i]->armors[j].get_ml_roi();
                    String path;
                    path = "../debug/samples/armordetect/" +
                           to_string(int(armor_infomations_[i]->armors[j].get_robot_type())) +
                           "/" + to_string(int(get_timenow() * 1000)) + ".png";
                    cv::imwrite(path, tmp);
                }

            }
            cout << "SAVE FINISHED" << endl;

        }
        if (cvui::button(frame, arearect.x + 25, arearect.y + 110, 220, 40, "Save History_Video")) {
            Mat tip=cv::Mat(cv::Size(150, 50), CV_8UC3);
            tip=Scalar(49,52,49);
            String text1="Saving";
            cv::putText(tip, text1,Point(15,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(201,198,201), 2, 8, 0);
            imshow("tip",tip);
            waitKey(100);

            String path;
            path = String("../debug/videos/") +Parameter.date+to_string(int(get_timenow()))+".avi";
            VideoWriter _video;
            _video.open(path, CV_FOURCC('M', 'J', 'P', 'G'), 50, Size(armor_infomations_[0]->src->size()));
            for (int i = 0; i < armor_infomations_.size(); i++) {
                _video << *(armor_infomations_[i]->src);
            }
            tip=Scalar(49,52,49);
            String text2="Succcess";
            cv::putText(tip, text2,Point(15,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(255,128,128), 2, 8, 0);
            imshow("tip",tip);
            waitKey(300);
            cv::destroyWindow("tip");

        }
        if (cvui::button(frame, arearect.x + 25, arearect.y + 160, 220, 40, "Save All_Parameters ")) {
            ReadParameter::WriteParameter();
            Mat tip=cv::Mat(cv::Size(150, 50), CV_8UC3);
            tip=Scalar(49,52,49);
            String text1="Succcess";
            cv::putText(tip, text1,Point(15,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(201,198,201), 2, 8, 0);
            imshow("tip",tip);
            waitKey(500);
            cv::destroyWindow("tip");
        }
        cvui::rect(frame, arearect.x + 200, arearect.y + 280, 60, 30, 0xFF3344);
        if (cvui::button(frame, arearect.x + 200 + 3, arearect.y + 280 + 3, 54, 24, "Quit")) {
            setval_.WORK= false;
            setval_.QUIT= true;
        }
    }

}

////////////////////////////////////////BuffDebugInformation/////////////////////////////////////////////////////

void Debugger::set_buff_infomation(BuffDebugInformation &information) {
    BuffDebugInformation *tmp=new BuffDebugInformation(information);
    if(buff_infomations_.size()>MaxSize){
        delete  buff_infomations_[0]->src;
        delete  buff_infomations_[0]->subcolor_img;
        delete  buff_infomations_[0]->binary_img;
        delete buff_infomations_[0];
        buff_infomations_.erase(buff_infomations_.begin());

    }
    buff_infomations_.push_back(tmp);
}
void Debugger::draw(BuffDebugInformation &information) {
    frame = cv::Mat(cv::Size(SRCWIDTH+480, 690), CV_8UC3);
    frame=cv::Scalar(49, 52, 49);
    main_src_=information.src->clone();

    gray_image_=*information.subcolor_img;
    binary_image_=*information.binary_img;

    ml_images_=cv::Mat::zeros(Size(96,28),CV_8UC3);

    cvtColor(gray_image_,gray_image_,COLOR_GRAY2BGR);
    cvtColor(binary_image_,binary_image_,COLOR_GRAY2BGR);
    EnergyBuffArmor armor=information.buffarmors[0];
    if(armor.empty()) return;
    if(!armor.empty()&&armordetect_check){
        for(int i=0;i<5;i++){
            EnergyBuffArmor armor2=information.buffarmors[i];
            if(armor2.empty()) continue;
            //逐条边绘制
            int color=Parameter.enemy_colour;
            cv::circle(main_src_,armor2.tl(),5,Scalar(189,100,150),-1);
            cv::circle(main_src_,armor2.tr(),5,Scalar(255,0,255),-1);
            cv::circle(main_src_,armor2.bl(),5,Scalar(0,255,255),-1);
            cv::circle(main_src_,armor2.br(),5,Scalar(0,255,0),-1);
            Point2f* vertices = new cv::Point2f[4];
            if(i==0){
                armor2.get_circle_rect_().points(vertices);
                //逐条边绘制

                for (int j = 0; j < 4; j++) {
                    cv::line(main_src_, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255, 50, 0),1);
                }
            }

            Point2f* vertices2 = new cv::Point2f[4];

            armor2.get_ml_rect_().points(vertices2);
            //逐条边绘制
            for (int j = 0; j < 4; j++) {
                if(armor2.get_type()==FlowWater){
                    cv::line(main_src_, vertices2[j], vertices2[(j + 1) % 4], cv::Scalar(100, 255, 100),1);
                } else{
                    cv::line(main_src_, vertices2[j], vertices2[(j + 1) % 4], cv::Scalar(255, 50, 50),1);
                }
            }
            delete  []vertices2;
            delete  []vertices;
            cv::circle(main_src_,armor2.get_circle(),8,Scalar(100,255,100),3);
            if(armor2.get_type()==FlowWater) {

                cv::putText(main_src_, doubleToString(double(armor2.get_angle())),
                            armor2.get_rotatedrect().boundingRect().tl(), cv::FONT_HERSHEY_COMPLEX, 0.8,
                            Scalar(100, 255, 100), 2, 8, 0);
            }else{

                    cv::putText(main_src_, doubleToString(double(armor2.get_angle())),
                                armor2.get_rotatedrect().boundingRect().tl(), cv::FONT_HERSHEY_COMPLEX, 0.8,
                                Scalar(255, 50, 50), 2, 8, 0);
                }
        }

    }



    if(ml_check){
        ml_images_=armor.get_ml_image();
        cvtColor(ml_images_,ml_images_,COLOR_GRAY2BGR);

    }

    if(!information.target.empty()){
        vector<Point3f> worldPoints;
        worldPoints=G_PointList.EBWorldPoints_List[0];
        Vec3f diffeular=-information.diffeular;//云台角度补偿
        Vec3d difftran=-information.difftran;//镜头到轴补偿,不同车不一样
        for(int i=0;i<5;i++){
            for(int j=1;j<4;j++) {
                vector<double > diffrot = {0,0,i * 72 * 3.1415f / 180};//云台角度补偿
                Mat diffrotmat = Mat(diffrot);
                Mat rotmatix;
                Rodrigues(diffrotmat, rotmatix);//旋转向量转换为旋转矩阵
                Mat aa= Mat(Point3d(worldPoints[j]));
                Mat p = rotmatix*aa ;
                Rodrigues(information.target.get_rvec(), rotmatix);//旋转向量转换为旋转矩阵

                Mat worldp=rotmatix*p+information.target.get_tvec();
                Point3f a = Point3f(*(worldp.ptr<Point3d>()));

                a=worldtocamera(a,information.diffeular,information.difftran);

                worldPoints.push_back(a);
                diffrot.clear();
            }
        }
        for(int i=0;i<4;i++){
            vector<double > diffrot = {0,0,i * 72 * 3.1415f / 180};//云台角度补偿
            Mat diffrotmat = Mat(diffrot);
            Mat rotmatix;
            Rodrigues(diffrotmat, rotmatix);//旋转向量转换为旋转矩阵
            Mat aa= Mat(Point3d(worldPoints[i]));
            Mat p = rotmatix*aa ;
            Rodrigues(information.target.get_rvec(), rotmatix);//旋转向量转换为旋转矩阵

            Mat worldp=rotmatix*p+information.target.get_tvec();
            Point3f a = Point3f(*(worldp.ptr<Point3d>()));

            a=worldtocamera(a,information.diffeular,information.difftran);

            worldPoints[i]=a;
        }

        Point3f predict_point = worldtocamera(information.predict_point,information.diffeular,information.difftran);


        Mat rv=Mat::zeros(Size(3,1),CV_32F);
        Mat tv=Mat::zeros(Size(3,1),CV_32F);
        BuffTarget target=information.target;
        vector<Point2f> impoints;
        projectPoints(worldPoints,rv,tv,camera_matrix_,dist_coeffs_,impoints);
        vector<Point2f> pre_impoints;
        vector<Point3f> pre_worldPoints={predict_point};

        projectPoints(pre_worldPoints,rv,tv,camera_matrix_,dist_coeffs_,pre_impoints);
        if(predict_check){
            cv::line(main_src_, pre_impoints[0]+Point2f(-8,-8), pre_impoints[0]+Point2f(8,8), cv::Scalar(0,100,255),2);
            cv::line(main_src_, pre_impoints[0]+Point2f(8,-8), pre_impoints[0]+Point2f(-8,8), cv::Scalar(0,100,255),2);
        }
        Scalar scalar={200,255,100};
        if(targetsolve_check){
            for(int i=0;i<impoints.size();i++){
                if((i-1)%3==0&&i>0){
                    scalar[0]-=20;
                    scalar[1]-=20;
                    scalar[2]+=20;
                }
                cv::circle(main_src_,impoints[i],5,scalar,2);
            }
        }
        impoints.clear();
        predict_point=information.predict_point;
        ShootPlatform platform=information.reciveMessage.shoot_platform;
        float dist_z= sqrt(predict_point.z*predict_point.z+predict_point.x*predict_point.x);
        float t=dist_z/(cos(platform.pitch)*platform.bulletspeed);
        float dist_y=-tan(platform.pitch)*dist_z+0.00098*t*t/2;
        float dist_x=dist_z* sin(platform.yaw);
        dist_z=dist_z* cos(platform.yaw);
        Point3f firecneter=Point3f(dist_x,dist_y,dist_z);

        Point3f firepoint = worldtocamera(firecneter,information.diffeular,information.difftran);
        Point3f firecneterleft=firepoint+Point3f(-information.diffsize.width,0,0);
        Point3f firecneterdown=firepoint+Point3f(0,-information.diffsize.height,0);

        worldPoints={firepoint,firecneterleft,firecneterdown};
        projectPoints(worldPoints,rv,tv,camera_matrix_,dist_coeffs_,impoints);
        RotatedRect box=RotatedRect(impoints[0],Size2f(impoints[0].x-impoints[1].x,impoints[0].y-impoints[2].y),0);
        if(fire_check){
            Rect brect=box.boundingRect();
            if(RectSafety(brect,main_src_.size())){
//                ellipse(main_src_, box, Scalar(0,0,255), 1, CV_AA);
            }
            circle(main_src_,box.center,2,Scalar(0,0,255),2);
        }
        worldPoints.clear();
        impoints.clear();
        pre_impoints.clear();
        pre_worldPoints.clear();
    }

    static  bool  kalmanwin;

    if(kalman_check){
        kalmanwin= true;
        kalman_image_ = cv::Mat(250, 800, CV_8UC3);
        kalman_image_=Scalar(46,36,32);
        std::vector<float > measurevalues;
        std::vector<float > prevalues;
        std::vector<float > postvalues;

        for(BuffDebugInformation*& tmp:buff_infomations_){
            measurevalues.push_back((tmp->measure.at<float>(0)));
            postvalues.push_back(tmp->kfpost.at<float>(0));
            prevalues.push_back(tmp->kfpre.at<float>(0));

        }
        float avr=cv::mean(Mat(measurevalues))[0];
        Rect arearect=Rect(Point(kalman_image_.cols-100,10+5),Size(85,60));

        cvui::rect(kalman_image_, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xffffff);
        circle( kalman_image_ , arearect.tl()+Point(15,15), 8,Scalar(220, 233, 225) ,1, 8, 0 );
        circle( kalman_image_ , arearect.tl()+Point(15,45), 8,Scalar(41, 222, 244) ,1, 8, 0 );
        cvui::printf(kalman_image_, arearect.x+25, arearect.y+15-5, "Measure");
        cvui::printf(kalman_image_, arearect.x+25, arearect.y+45-5, "State");

        for(int i=0;i<measurevalues.size();i++ ){
            circle( kalman_image_ , Point(i*4+80,5*(measurevalues[i]-avr)+125), 3,Scalar(220, 233, 225) ,
                    1, 8, 0 );
            circle( kalman_image_ , Point(i*4+80,5*(postvalues[i]-avr)+125), 3,Scalar(41, 222, 244) ,
                    1, 8, 0 );
        }

        imshow("kalman",kalman_image_);
        measurevalues.clear();
        prevalues.clear();
        postvalues.clear();
    } else{
        if(kalmanwin){
            cv::destroyWindow("kalman");
            kalmanwin= false;
        }
    }
}
void Debugger::arrange_layout(BuffDebugInformation &information) {

    Rect arearect;
    Mat srccopy;
    resize(*information.src,srccopy,Size(SRCWIDTH,cvRound(main_src_.size().height*SRCWIDTH/main_src_.size().width)));
    int pad=1;
    int informationw=185;



    ///////////////////
    if(true){
        arearect=Rect(Point(5,20+5),Size(informationw,90));
        cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Cursor");
        int mx=cvui::mouse().x-(informationw+10);
        int my=cvui::mouse().y-25;
        mx=cvRound(mx*information.src->size().width/SRCWIDTH);
        my=cvRound(my*information.src->size().width/SRCWIDTH);
        if(mx<0||mx>information.src->size().width||my<0||my>information.src->size().height){
            mx=0;my=0;
        }
        cvui::printf(frame, arearect.x+5, arearect.y+20, "  X,Y (%d,%d)", mx, my);
        cvui::printf(frame, arearect.x+5, arearect.y+45, "  B,G,R (%d,%d,%d)", information.src->at<Vec3b>(my,mx)[0], information.src->at<Vec3b>(my,mx)[1],information.src->at<Vec3b>(my,mx)[2]);
        cvui::printf(frame, arearect.x+5, arearect.y+70, "  Time (%.3lf)ms", information.timenow);

        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);

        arearect=Rect(Point(arearect.x,arearect.br().y+20+5),Size(informationw,544));
        cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Informations");
        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);
        static int last_page0,last_page1,last_page2;
        last_page0=page0;last_page1=page1;last_page2=page2;
        cvui::checkbox(frame,arearect.x+pad+25,arearect.y+pad+10,"0", &page0);
        cvui::checkbox(frame,arearect.x+pad+75,arearect.y+pad+10,"1", &page1);
        cvui::checkbox(frame,arearect.x+pad+125,arearect.y+pad+10,"2", &page2);

        if(page0==1&&last_page0==0){
            page1=0;page0=1;page2=0;
        }else if(page1==1&&last_page1==0){
            page1=1;page0=0;page2=0;
        }
        else if(page2==1&&last_page2==0){
            page1=0;page0=0;page2=1;
        } else{
            page1=last_page1;page0=last_page0;page2=last_page2;

        }

        if(page0){

            arearect=Rect(arearect.x-pad+6,arearect.y+40,arearect.width+pad*2-12,90);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "     ReceiveMessage");
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Yaw,Picth(%.3f,%.3f)", information.reciveMessage.shoot_platform.yaw, information.reciveMessage.shoot_platform.pitch);
            cvui::printf(frame, arearect.x+8, arearect.y+50, "Lock_Command (%d)", information.reciveMessage.lock_command);
            cvui::printf(frame, arearect.x+8, arearect.y+70,information.reciveMessage.mode==0?"Mode (ArmorMode)":"Mode (EnergyBuffMode)");

            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "      SendMessage");

            cvui::printf(frame, arearect.x+8, arearect.y+30, "Yaw,Picth(%.3f,%.3f)", information.sendMessage.yaw*57.3f, information.sendMessage.pitch*57.3f);
            cvui::printf(frame, arearect.x+8, arearect.y+50, "Beat (%d)", information.sendMessage.beat);
            cvui::printf(frame, arearect.x+8+60, arearect.y+50,"NoObject (%d)" ,information.sendMessage.no_object);
            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,320, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "    Position&Posture");
            BuffTarget target=information.target;
            Point3f circle_point,armor_point;
            Vec3f eularangel={0,0,0};
            Point3f predict_point={0,0,0};
            if(!target.empty()){
                circle_point=Point3f(*(target.get_tvec().ptr<Point3d>()));
                armor_point=target.get_planes()[0].get_world_point();
                eularangel=target.get_planes()[0].get_euler_angle();
                predict_point=information.predict_point;
            } else{
                circle_point=Point3f(0,0,0);
                armor_point={0,0,0};
            }
            Polor3f circle_polor=to_polor(circle_point);
            Polor3f armor_polor=to_polor(armor_point);
            Polor3f predict_polor=to_polor(predict_point);

            cv::line(frame,arearect.tl()+Point(10,25),arearect.tl()+Point(100,25),Scalar(178,190,137),1);
            cvui::printf(frame, arearect.x+8, arearect.y+30, "CircleCenter(X,Y,Z)");
            cvui::printf(frame, arearect.x+8, arearect.y+50, "(%.1f, %.1f, %.1f)", circle_point.x, circle_point.y,circle_point.z);
            cvui::printf(frame, arearect.x+8, arearect.y+70, "Circle(Dist, Yaw, Pitch)");
            cvui::printf(frame, arearect.x+8, arearect.y+90, "(%.1f, %.3f, %.3f)", circle_polor.distance,circle_polor.yaw,circle_polor.pitch);
            cv::line(frame,arearect.tl()+Point(10,105),arearect.tl()+Point(100,105),Scalar(178,190,137),1);
            cvui::printf(frame, arearect.x+8, arearect.y+110, "ArmorCenter(X, Y, Z)");
            cvui::printf(frame, arearect.x+8, arearect.y+130, "(%.1f, %.1f, %.1f)", armor_point.x, armor_point.y,armor_point.z);
            cvui::printf(frame, arearect.x+8, arearect.y+150, "Armor(Dist, Yaw, Pitch)");
            cvui::printf(frame, arearect.x+8, arearect.y+170, "(%.1f, %.3f, %.3f)", armor_polor.distance,armor_polor.yaw,armor_polor.pitch);
            cv::line(frame,arearect.tl()+Point(10,185),arearect.tl()+Point(100,185),Scalar(178,190,137),1);

            cvui::printf(frame, arearect.x+8, arearect.y+190, "EnergyBuffEularangle");
            cvui::printf(frame, arearect.x+8, arearect.y+210, "(%.2f, %.2f, %.2f)", eularangel[0],eularangel[1],eularangel[2]);
            cv::line(frame,arearect.tl()+Point(10,225),arearect.tl()+Point(100,225),Scalar(178,190,137),1);

            cvui::printf(frame, arearect.x+8, arearect.y+230, "PredictPoint(X, Y, Z)");
            cvui::printf(frame, arearect.x+8, arearect.y+250, "(%.1f, %.1f, %.1f)", predict_point.x, predict_point.y,predict_point.z);
            cvui::printf(frame, arearect.x+8, arearect.y+270, "Predict(Dist, Yaw, Pitch)");
            cvui::printf(frame, arearect.x+8, arearect.y+290, "(%.1f, %.3f, %.3f)", predict_polor.distance,predict_polor.yaw,predict_polor.pitch);


        } else if(page1){
            arearect=Rect(arearect.x-pad+6,arearect.y+40,arearect.width+pad*2-12,110);

            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "   CameraCoordinate");
            Mat camrvec=information.camrvec;
            Mat camtvec=information.camtvec;
            Vec3f eular=rvecToEulerAngles(camrvec);
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Tv (%.1f,%.1f,%.1f)", camtvec.at<double>(0),camtvec.at<double>(1),camtvec.at<double>(2));
            cvui::printf(frame, arearect.x+8, arearect.y+50, "Rv (%.2f,%.2f,%.2f)", camrvec.at<double>(0),camrvec.at<double>(1),camrvec.at<double>(2));
            cvui::printf(frame, arearect.x+8, arearect.y+70, "EularAngle(P,Y,R)");
            cvui::printf(frame, arearect.x+8, arearect.y+90, "(%.2f,%.2f,%.2f)", eular[0],eular[1],eular[2]);

            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,90);;
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "        Kalman");
            cvui::printf(frame, arearect.x+8, arearect.y+30, "Measure   (%.2f)", information.measure.at<float>(0));
            cvui::printf(frame, arearect.x+8, arearect.y+50, "StatePre  (%.2f)", information.kfpre.at<float>(0));
            cvui::printf(frame, arearect.x+8, arearect.y+70, "StatePost (%.2f)", information.kfpost.at<float>(0));
            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,arearect.height, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "      FireCommand");

            cvui::printf(frame, arearect.x+8, arearect.y+30, "FireRadius (%.1f,%.1f)", information.diffsize.width, information.diffsize.height);

            cvui::printf(frame, arearect.x+8, arearect.y+50, "DiffSize (%.1f,%.1f)", information.realdiff.width, information.realdiff.height);
            arearect=Rect(arearect.x,arearect.br().y+5,arearect.width,70);
            cvui::rect(frame, arearect.x ,arearect.y ,arearect.width,210, 0xfce6c9);
            cvui::printf(frame, arearect.x+5, arearect.y+10, "  TemporaryParameters");
            cvui::printf(frame, arearect.x+8, arearect.y+30, "P0[armor__area] (%.3f)", information.tmpparam0);
            cvui::printf(frame, arearect.x+8, arearect.y+50, "P1[circlr_area] (%.3f)", information.tmpparam1);
            cvui::printf(frame, arearect.x+8, arearect.y+70, "P2[big_con_size] (%.3f)", information.tmpparam2);
            cvui::printf(frame, arearect.x+8, arearect.y+90, "P3[sm_con_size] (%.3f)", information.tmpparam3);
            cvui::printf(frame, arearect.x+8, arearect.y+110, "P4[_______kd_r] (%.3f)", information.tmpparam4);
            cvui::printf(frame, arearect.x+8, arearect.y+130, "P5[kd_flowwater] (%.3f)", information.tmpparam5);
            cvui::printf(frame, arearect.x+8, arearect.y+150, "P6[armors__size] (%.3f)", information.tmpparam6);
            cvui::printf(frame, arearect.x+8, arearect.y+170, "P7 (%.3f)", information.tmpparam7);
            cvui::printf(frame, arearect.x+8, arearect.y+190, "P8 (%.3f)", information.tmpparam8);
        } else if(page2){
            arearect=Rect(Point(arearect.x+40,arearect.y+70),ml_images_.size());
//            cvui::window(frame, arearect.x-pad ,arearect.y-20-pad ,arearect.width+pad*2,arearect.height+20+pad*2, "ml_imgs");
            cvui::printf(frame, arearect.x+10, arearect.y-15, "ml_images");
            cvui::rect(frame, arearect.x-10 ,arearect.y-20 ,arearect.width+20,arearect.height+25, 0xfce6c9);
            ml_images_.copyTo(frame(arearect));
        }
    }
    ///////////////////
    if(gray_check){
        resize(gray_image_,gray_image_,Size(SRCWIDTH,cvRound(gray_image_.size().height*SRCWIDTH/gray_image_.size().width)));
        resize(binary_image_,binary_image_,Size(SRCWIDTH,cvRound(binary_image_.size().height*SRCWIDTH/binary_image_.size().width)));
        arearect=Rect(Point(informationw+10,20+5),gray_image_.size());
        cvui::window(frame, arearect.x-pad ,arearect.y-20-pad ,arearect.width+pad*2,arearect.height+20+pad*2, "Src");
        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);
        cvui::button(frame, arearect.x, arearect.y,binary_image_ ,gray_image_,srccopy);

    } else{

        resize(main_src_,main_src_,Size(SRCWIDTH,cvRound(main_src_.size().height*SRCWIDTH/main_src_.size().width)));

        arearect=Rect(Point(informationw+10,20+5),main_src_.size());
        cvui::window(frame, arearect.x-pad ,arearect.y-20-pad ,arearect.width+pad*2,arearect.height+20+pad*2, "Src");
        cvui::rect(frame, arearect.x-pad ,arearect.y-pad ,arearect.width+pad*2,arearect.height+pad*2, 0xffffff);
        cvui::button(frame, arearect.x, arearect.y, main_src_, main_src_, srccopy);
    }

    ///////////////////

    arearect=Rect(Point(SRCWIDTH+informationw+15,5+20),Size(270,115));
    pad=20;
    cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Control");
    cvui::rect(frame, arearect.x, arearect.y, arearect.width , arearect.height, 0xffffff);
    ///////////////////////////////
    arearect=Rect(Point(SRCWIDTH+informationw+15,arearect.br().y+5+20),Size(arearect.width,175));
    pad=20;
    cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Form");
    cvui::rect(frame, arearect.x, arearect.y, arearect.width , arearect.height, 0xffffff);

    if(setval_.detectmode==ArmorMode) {
        cvui::button(frame, arearect.x+15, arearect.y+10,180,30, "ArmorMode");
        if(!pausejudge){
            if (cvui::button(frame, arearect.x+180+20, arearect.y+10,50,30, ">>")) {
                lastmode=EnergyBuffMode;
            }
        }

    }else{
        cvui::button(frame, arearect.x+15, arearect.y+10,180,30, "EnergyBuffMode");
        if(!pausejudge){

            if (cvui::button(frame, arearect.x+180+20, arearect.y+10,50,30, ">>")) {
                lastmode=ArmorMode;
            }
        }
    }

    static bool last_gray,last_normal;
    last_gray=gray_check;last_normal=normal_check;
    cvui::checkbox(frame,arearect.x+pad+80,195+pad,"Gray", &gray_check);
    cvui::checkbox(frame,arearect.x+pad,195+pad,"Normal", &normal_check);
    if((gray_check&&!last_gray)||(!normal_check&&last_normal)){
        normal_check=0,gray_check=1;
    }
    else if((normal_check&&!last_normal)||(!gray_check&&last_gray)) {

        normal_check=1;gray_check=0;
    } else{

        normal_check=last_normal;gray_check=last_gray;
    }

    if(!gray_check){
        cvui::checkbox(frame,arearect.x+pad,230+pad,"TargetSolve", &targetsolve_check);
        cvui::checkbox(frame,arearect.x+pad,260+pad,"ArmorDetect", &armordetect_check);
        cvui::checkbox(frame,arearect.x+pad,290+pad,"Kalman", &kalman_check);
        cvui::checkbox(frame,arearect.x+pad+140,230+pad,"FireCommand", &fire_check);
        cvui::checkbox(frame,arearect.x+pad+140,260+pad,"Predict", &predict_check);

    } else {
        cvui::checkbox(frame,arearect.x+pad,290+pad,"Kalman", &kalman_check);


    }
    ///////////////////////////////
    arearect=Rect(Point(SRCWIDTH+informationw+15,arearect.br().y+5+20),Size(arearect.width,320));
    pad=20;
    cvui::window(frame, arearect.x ,arearect.y-20 ,arearect.width,arearect.height+20, "Tools");
    cvui::rect(frame, arearect.x, arearect.y, arearect.width , arearect.height, 0xffffff);
    static int tlast_page0,tlast_page1,tlast_page2;
    tlast_page0=tpage0;tlast_page1=tpage1;tlast_page2=tpage2;
    cvui::checkbox(frame,arearect.x+pad+30,arearect.y+pad,"0", &tpage0);
    cvui::checkbox(frame,arearect.x+pad+100,arearect.y+pad,"1", &tpage1);
    cvui::checkbox(frame,arearect.x+pad+170,arearect.y+pad,"2", &tpage2);

    if(tpage0==1&&tlast_page0==0){
        tpage0=1;tpage1=0;tpage2=0;
    }else if(tpage1==1&&tlast_page1==0){
        tpage0=0;tpage1=1;tpage2=0;
    }
    else if(tpage2==1&&tlast_page2==0){
        tpage0=0;tpage1=0;tpage2=1;
    } else{
        tpage0=tlast_page0;tpage1=tlast_page1;tpage2=tlast_page2;

    }
    if(tpage0){
        cvui::rect(frame, arearect.x +10,arearect.y+45 ,arearect.width-20,160, 0x9BCD9B);
        cvui::printf(frame,  arearect.x+30+5, arearect.y+60, "split_thre:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+50,130, setval_.buff_split_threval, 0, 255);
        cvui::printf(frame, arearect.x+30+5, arearect.y+110, "dilate:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+100,130, setval_.dilatesize_val, 0, 5);
        cvui::printf(frame, arearect.x+30+5, arearect.y+160, "erode:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+150,130, setval_.erodesize_val, 0, 5);
        cvui::rect(frame, arearect.x +10,arearect.y+210 ,arearect.width-20,100, 0x9BCD9B);
        cvui::printf(frame, arearect.x+30+5, arearect.y+230, "gray_thre:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+220,130, setval_.buff_gray_threval, 0, 255);
        cvui::printf(frame, arearect.x+30+5, arearect.y+270, "tmpval1:");
        cvui::trackbar(frame, arearect.x+90+5, arearect.y+260,130, setval_.tmpval1, float(0), float(100));

    } else if(tpage1){
        cvui::rect(frame, arearect.x +10,arearect.y+45 ,arearect.width-20,170, 0x9BCD9B);
        cvui::printf(frame,  arearect.x+80+5, arearect.y+55, "HikvisionCamera");
        cvui::counter(frame,arearect.x+20+5+100, arearect.y+80-5,setval_.cam_brightness,50);
        cvui::printf(frame,  arearect.x+20+5, arearect.y+80, "brightness");
        cvui::counter(frame,arearect.x+20+5+100, arearect.y+110-5,setval_.cam_gain,0.1);
        cvui::printf(frame,  arearect.x+20+5, arearect.y+110, "gain");
        cvui::counter(frame,arearect.x+20+5+100, arearect.y+140-5,setval_.cam_gamma,0.1);
        cvui::printf(frame,  arearect.x+20+5, arearect.y+140, "gamma");
        if(cvui::button(frame, arearect.x+25, arearect.y+165,220,40, "Set To HikvisionCamera")){
            setval_.cam_set= true;
        } else{
            setval_.cam_set= false;
        }
    } else if(tpage2){
        cvui::rect(frame, arearect.x +10,arearect.y+45 ,arearect.width-20,170, 0x9BCD9B);
        if(cvui::button(frame, arearect.x+25, arearect.y+60,220,40, "Save Ml_Samples")) {
            for(int i=0;i<buff_infomations_.size();i++){
                for(int j=0;j<5;j++){
                    if(buff_infomations_[i]->buffarmors[j].empty())continue;
                    Mat tmp=buff_infomations_[i]->buffarmors[j].get_ml_image();
                    String path;
                    if(j==0){
                        path ="../debug/samples/energybuffdetect/1/"+to_string(int(get_timenow()*1000))+".png";
                    } else{
                        path ="../debug/samples/energybuffdetect/0/"+to_string(int(get_timenow()*1000))+".png";
                    }
                    cv::imwrite(path,tmp);
                }

            }
            cout<<"SAVE FINISHED"<<endl;

        }
        if(cvui::button(frame, arearect.x+25, arearect.y+110,220,40, "Save History_Video")){
            Mat tip=cv::Mat(cv::Size(150, 50), CV_8UC3);
            tip=Scalar(49,52,49);
            String text1="Saving";
            cv::putText(tip, text1,Point(15,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(201,198,201), 2, 8, 0);
            imshow("tip",tip);
            waitKey(100);

            String path;
            path = String("../debug/videos/") +Parameter.date+to_string(int(get_timenow()))+".avi";
            VideoWriter _video;
            _video.open(path, CV_FOURCC('M', 'J', 'P', 'G'), 50, Size(buff_infomations_[0]->src->size()));
            for (int i = 0; i < buff_infomations_.size(); i++) {
                _video << *(buff_infomations_[i]->src);
            }
            tip=Scalar(49,52,49);
            String text2="Succcess";
            cv::putText(tip, text2,Point(15,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(255,128,128), 2, 8, 0);
            imshow("tip",tip);
            waitKey(300);
            cv::destroyWindow("tip");
        }
        if(cvui::button(frame, arearect.x+25, arearect.y+160,220,40, "Save All_Parameters ")) {
            ReadParameter::WriteParameter();
            Mat tip=cv::Mat(cv::Size(150, 50), CV_8UC3);
            tip=Scalar(49,52,49);
            String text1="Succcess";
            cv::putText(tip, text1,Point(15,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(201,198,201), 2, 8, 0);
            imshow("tip",tip);
            waitKey(500);
            cv::destroyWindow("tip");
        }
        cvui::rect(frame, arearect.x +200,arearect.y+280 ,60,30, 0xFF3344);
        if(cvui::button(frame, arearect.x +200+3,arearect.y+280+3 ,54,24, "Quit")) {
            setval_.WORK= false;
            setval_.QUIT= true;

        }
    }


}

///////////////////////////////////
void Debugger::show() {
    static char  code;
    static int dform=1;
    char tmp=(char)waitKey(1);
    if(tmp>0){
        code=tmp;
    }
    if( code == 27 || code == 'q' || code == 'Q' ){

        if(tmp== 27 || tmp == 'q' || tmp == 'Q' ){
            cv::destroyAllWindows();
            Mat tip=cv::Mat(cv::Size(400, 170), CV_8UC3);
            tip=Scalar(61,41,30);
            String text1="'D' for Start and Debug";
            cv::putText(tip, text1,Point(10,30), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(254,254,254), 2, 8, 0);
            text1="'S' for Pause and Debug";
            cv::putText(tip, text1,Point(10,70), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(254,254,254), 2, 8, 0);
            text1="'Q' or 'Esc' for Quit";
            cv::putText(tip, text1,Point(10,110), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(254,254,254), 2, 8, 0);
            text1="'W' and 'E' for Slide";
            cv::putText(tip, text1,Point(10,150), cv::FONT_HERSHEY_SIMPLEX,0.8, Scalar(254,254,254), 2, 8, 0);
            imshow("TIP",tip);
            dform=0;
        }
        return;
    } else if( code == 'D' || code == 'd'){
        if(tmp == 'D' || tmp == 'd'){
            if(!dform){
                cv::destroyAllWindows();
                cvui::init(WINDOW_NAME);
            }
            pausejudge = 0;
            dform=1;
        }
    }else if( code == 'S' || code == 's'){
        if( tmp == 'S' || tmp == 's' ) {
            if(!dform){
                cv::destroyAllWindows();
                cvui::init(WINDOW_NAME);
            }
            pausejudge = 1;
            dform=1;
        }
    } else{
        if(!dform){
            return;
        }
    }

    if(setval_.detectmode==ArmorMode){

        draw(*armor_infomations_.back());
        arrange_layout(*armor_infomations_.back());

        ///////////////////
        Rect arearect;

        arearect=Rect(Point(175+SRCWIDTH+10+20+5,30),Size(250,50));
        if (pausejudge||cvui::button(frame, arearect.x, 30,250,100, "Pause")) {
            pausejudge=true;
            int index=0;
            while (true){

                cvui::update();
                cvui::imshow(WINDOW_NAME,frame);
                char tmpkey=(char)waitKey(1);

                draw(*armor_infomations_[armor_infomations_.size()-1+index]);
                arrange_layout(*armor_infomations_[armor_infomations_.size()-1+index]);
                if (cvui::button(frame, arearect.x, 30,125,50, "Start")) {
                    pausejudge=0;
                    break;
                }
                if (cvui::button(frame, arearect.x+125, 30,125,50, "Next>")) {
                    pausejudge=1;
                    break;
                }
                cvui::trackbar(frame, arearect.x+5, arearect.y+60,240, &index, -int(armor_infomations_.size()-1),0);
                if(tmpkey=='w' or tmpkey=='W'){
                    if(-index>=0&&-index<armor_infomations_.size()-1)
                        index-=1;
                } else if(tmpkey=='e' or tmpkey=='E'){
                    if(-index>0&&-index<armor_infomations_.size())
                        index+=1;
                } else if(tmpkey=='d' or tmpkey=='D'){
                    pausejudge=0;break;
                }
            }
        }
    } else{


        draw(*buff_infomations_.back());
        arrange_layout(*buff_infomations_.back());
        ///////////////////

        Rect arearect;

        arearect=Rect(Point(175+SRCWIDTH+10+20+5,30),Size(250,50));

        if (pausejudge||cvui::button(frame, arearect.x, 30,250,100, "Pause")) {
            pausejudge=1;
            int index=0;
            while (true){
                cvui::update();
                cvui::imshow(WINDOW_NAME,frame);
                char tmpkey=(char)waitKey(1);

                draw(*buff_infomations_[buff_infomations_.size()-1+index]);
                arrange_layout(*buff_infomations_[buff_infomations_.size()-1+index]);
                if (cvui::button(frame, arearect.x, 30,125,50, "Start")) {
                    pausejudge=0;
                    break;
                }
                if (cvui::button(frame, arearect.x+125, 30,125,50, "Next>")) {
                    pausejudge=1;
                    break;
                }

                cvui::trackbar(frame, arearect.x+5, arearect.y+60,240, &index, -int(buff_infomations_.size()-1),0);
                if(tmpkey=='w' or tmpkey=='W'){
                    if(-index>=0&&-index<buff_infomations_.size()-1)
                        index-=1;
                } else if(tmpkey=='e' or tmpkey=='E'){
                    if(-index>0&&-index<buff_infomations_.size())
                        index+=1;
                } else if(tmpkey=='d' or tmpkey=='D'){
                    pausejudge=0;break;
                }
            }
        }
    }
    ///////////////////
    setval_.detectmode=lastmode;
    cvui::update();
    cvui::imshow(WINDOW_NAME,frame);

    if(!setval_.WORK){
        clearall();
    }
}



