
#include "predict.h"
#include "debug.h"
#include "targetresolver.h"

//////////////////////////EnergyBuffPredictor//////////////////////////////

EnergyBuffPredictor::EnergyBuffPredictor() :Predictor() {
    circle_point_=Point3f(-1,-1,-1);}
void EnergyBuffPredictor::DetaConvert(const double angle, const double deta_x, const double deta_y, double &deta_radial, double &deta_tangent)
{
    double l = pow((deta_x * deta_x + deta_y * deta_y), 0.5);
    double alpha = kPI / 2 - angle - atan(deta_y / deta_x);
    deta_radial = cos(alpha) * l;
    deta_tangent = sin(alpha) * l;
}

void EnergyBuffPredictor::set_new_target(BuffTarget &target) {
    if(target.empty())return;

    if(history_targets_.empty()){
        history_targets_.push_back(target);
    }else{
        circle_point_=PredictTime2Point(0,0);

        bool same_armor= false;
        if(getDistance(target.get_planes()[0].get_world_point(),history_targets_.back().get_planes()[0].get_world_point())<50){
            //世界坐标系中距离小于50
            same_armor= true;
        }
        bool same_time=target.get_time_now()-history_targets_.back().get_time_now()<100;

        if(same_armor&&same_time){//位置时间连续
            history_targets_.push_back(target);
        } else{
            vector<BuffTarget>{target}.swap(history_targets_);
        }


        if(history_targets_.size()>50){
            history_targets_.erase(history_targets_.begin());
        }


    }


#ifdef DEBUG
    if(G_setval.WORK) {
        G_buffinformation.target = target;
    }
#endif

}

void EnergyBuffPredictor::judge_clockwise(){
    if(history_targets_.size() > 20){
        int judgement=0;
        for (int index = 1; index < 6; index++){
            int current_index=int(history_targets_.size()-6+index);
            int last_index = index;
            float diff_angle=getDiffangle(history_targets_[current_index].get_instant_angle() , history_targets_[last_index].get_instant_angle());
            float angle_speed=1000*diff_angle/ static_cast<float>(history_targets_[current_index].get_time_now() - history_targets_[last_index].get_time_now());
            if(angle_speed>0.4) {
                judgement += 1;  //小符权重+1
            } else if(angle_speed<-0.4){
                judgement -= 1;  //小符权重+1
            } else{
                ;
            }

        }
        if(judgement > 3){//如果是大符就判断顺时针或者逆时针
            buff_type_ = true;
            clockwise_= false;
        }else if(judgement < -3){
            buff_type_ = true;
            clockwise_= true;
        } else{//小符
            buff_type_ = false;
        }
    }
}


Point3f EnergyBuffPredictor::PredictTime2Point(const float predict_time,int kr)
{
    Mat rotmatix, p;	//矩阵,旋转矩阵,p(旋转之后点在物体坐标系的坐标)
    Vec3d diffrot, p0;	//向量,旋转向量,p0(物体坐标系中,流水灯装甲板中心坐标)
    float predict_angle;	//预测的角度
    //预测角度计算,因为6000ms一圈,所以预测时间除以6000乘以2pi就是预测角度,后面是旋转方向(顺逆时针)
    predict_angle = 2 * kPI * predict_time / 6000.f * (clockwise_?-1:1);
    predict_point_angle = 2 * kPI + ((history_targets_.back().get_instant_angle() + predict_angle));

    diffrot = {0, 0, predict_angle};	//旋转的roll角度
    Rodrigues(Mat(diffrot), rotmatix);	//旋转向量转换为旋转矩阵
    p0 = {0, G_PointList.kEBr*kr, 14.5f*kr};
    p = rotmatix * Mat(p0);	//物体坐标系中预测点的位置
    Rodrigues(history_targets_.back().get_rvec(), rotmatix);	//旋转向量转换为旋转矩阵
    p = rotmatix * p+history_targets_.back().get_tvec();	//预测点在世界坐标系中的位置

    p.convertTo(p, CV_32F);	//将p转换为32为浮点型向量
    return  *(p.ptr<Point3f>());	//转化为Point3f
}


void EnergyBuffPredictor::predict(ReciveMessage const &reciveMessage)
//通过预测时间计算大概击打位置,推算就出diff_pitch,得到settime(提前静态瞄准时间)

{
    float  predict_time;
    Polor3f a = history_targets_.back().get_planes()[0].get_polor_point();
    const ShootPlatform &platform = reciveMessage.shoot_platform;	//云台接收到的信息


    if(follow_){
        predict_time=(cos(a.pitch) / cos(platform.pitch)) * (a.distance) / platform.bulletspeed+120;

    } else{
        //默认提前静态瞄准时间,为EnergyBuffPredictor默认初始化成员变量basePredictTime
        float set_time = basePredictTime;


        //预测时间默认同set_time,默认值用来估测击打位置
        predict_time = basePredictTime;


        predict_point_ = PredictTime2Point(predict_time * (buff_type_ ? 1 : 0));


        //计算pitch和yaw方向的偏差量
        float diff_pitch = fabs(reciveMessage.shoot_platform.pitch - history_targets_.back().get_planes()[0].get_polor_point().pitch) * 180 / kPI;
        float diff_yaw = fabs(reciveMessage.shoot_platform.yaw - history_targets_.back().get_planes()[0].get_polor_point().yaw) * 180 / kPI;

        //通过pitch偏差量来确定最终的set_time,计算方法暂时定死
        if(diff_pitch < 5)
        {
            set_time = basePredictTime ;
        }
        if(diff_pitch > 10)
        {
            set_time = basePredictTime + 500;
        }


        //如果切换装甲,历史目标位置会清空,第一次检测到装甲板会进行静态预测
        if(history_targets_.size() == 1)
        {
            all_uptime_ = set_time;
            last_uptime_ = get_timenow();
            last_set_time_ = set_time;
        }
        else
        {
            //计算all_uptime(静态瞄准位置(击打位置)的时间提前量),用上次更新all_uptime时它的总时间减去距离上次更新all_uptime距现在流逝的时间
            if(all_uptime_ > 0)
            {
                all_uptime_ = last_set_time_ - (float(get_timenow() - last_uptime_));
            }
        }

        //预测子弹飞行时间,用来计算实际击打位置
        predict_time = (cos(a.pitch) / cos(platform.pitch)) * (a.distance) / platform.bulletspeed;

        //在预测时间基础上,加上时间提前量,得到炮口需要收敛到的真实位置对应的预测时间
        if(all_uptime_>0)
        {
            predict_time += all_uptime_;
        }
        else	//时间提前量消耗完之后,更新时间提前量,选择下一个静态瞄准位置
        {
            //在静态瞄准启用时未击中的话重新寻找静态目标
            all_uptime_ = basePredictTime+200;
            last_uptime_ = get_timenow();
            last_set_time_ =  basePredictTime+200;
        }

    }


    predict_point_ = PredictTime2Point(predict_time * (buff_type_ ? 1 : 0));

#ifdef DEBUG
    if(G_setval.WORK){
        if(*G_setval.tmpval1>90){
            predict_point_ = PredictTime2Point(predict_time * (buff_type_ ? 1 : 0),0);

        }

    }
#endif
}

void EnergyBuffPredictor::calc_Output(ReciveMessage const &reciveMessage, SendMessage &sendMessage)
{
    if(history_targets_.empty()){
        sendMessage.no_object = true;
        sendMessage.pitch=0;
        sendMessage.yaw=0;
        sendMessage.beat= false;
        return;
    }
    bool go_to_circle= false;
    judge_clockwise();
    const ShootPlatform &platform = reciveMessage.shoot_platform;
    this -> predict(reciveMessage);//预测,更新predict_point_
    if(get_timenow()-history_targets_.back().get_time_now()>250){
        if(circle_point_.y==-1){
            sendMessage.no_object = true;
            sendMessage.pitch=0;
            sendMessage.yaw=0;
            sendMessage.beat= false;
            return;
        }
        if(fabs(history_targets_.back().get_instant_angle())<0.5||
           get_timenow()-history_targets_.back().get_time_now()>400){
            predict_point_=circle_point_;
            go_to_circle=true;
        } else{
            sendMessage.no_object = true;
            sendMessage.pitch=0;
            sendMessage.yaw=0;
            sendMessage.beat= false;
            return;
        }

    }
    /////重力补偿/////
    Point3f world_point = predict_point_;
    Polor3f polor_point = to_polor(world_point);//极坐标

    float x1,y1;//平面直角坐标系,抛物线过点(x1,y1)
    y1 = world_point.y;
    x1 = sqrt(world_point.x * world_point.x + world_point.z * world_point.z);//平面坐标系中的x1,y1
    Vec2f phi = parabolasolve(Point2f(x1, y1), platform.bulletspeed);//过点(x1,y1)解抛物线方程,得到两个出射角的解
    sendMessage.pitch = fabs(phi[0] - platform.pitch) < fabs(phi[1] - platform.pitch) ? phi[0] : phi[1];//选择较近的出射角
    sendMessage.yaw = polor_point.yaw;

    /////开火判断/////
    float deta_x, deta_y;
    deta_x = (sin(platform.yaw-sendMessage.yaw)) * x1;//x轴方向偏差
    deta_y = (tan(platform.pitch) - tan(sendMessage.pitch)) * x1;//y轴方向偏差

    //判断目标和炮台相对角速度
    float diff_yaw = 0, diff_pitch = 0, omega_yaw = 0, omega_pitch = 0;
    diff_yaw = sendMessage.yaw - platform.yaw;
    diff_pitch = sendMessage.pitch - platform.pitch;
    omega_yaw = static_cast<float>((diff_yaw - last_diff_yaw) / (get_timenow() - last_calc_time_));
    omega_pitch = static_cast<float>((diff_pitch - last_diff_pitch) / (get_timenow() - last_calc_time_));
    last_diff_yaw = diff_yaw;
    last_diff_pitch = diff_pitch;

    double  deta_r=0;
    double  deta_g=0;
    //当时间提前量在炮口发射的最小稳定时间附近,且子弹落点误差在最大误差允许范围内时,下达开火命令
    if(follow_){
        float angle = predict_point_angle;
        AngleCorrect(angle);
static int  converge_count=0;
        DetaConvert(angle,deta_x,deta_y,deta_r,deta_g);
        if(buff_type_){
            if(fabs(deta_r) < 25.0f && fabs(deta_g) < 40.5f){
                converge_count++;
            } else{
                converge_count=0;
            }
            if(converge_count>0){
                sendMessage.beat=true;
            }else{
                sendMessage.beat=false;
            }

//
        } else{
            if(fabs(deta_r) < 25.0f && fabs(deta_g) < 40.5f){
                converge_count++;
            } else{
                converge_count=0;
            }
            if(converge_count>0){
                sendMessage.beat=true;
            }else{
                sendMessage.beat=false;
            }

//

        }
              //(fabs(omega_yaw) < maxOmegaOnBeat && fabs(omega_pitch) < maxOmegaOnBeat);//大神符装甲板

//        if(get_timenow()-last_beat_time_<630){
//            sendMessage.beat= false;
//        }

    } else{
        sendMessage.beat = (fabs(all_uptime_ - beatStableDelay) < 10) &&
                           (fabs(deta_x) < maxErrorOnBeat && fabs(deta_y) < maxErrorOnBeat) ;
    }


    if(sendMessage.beat){
        last_beat_time_=get_timenow();
    }


    sendMessage.no_object = false;

    sendMessage.yaw -= platform.yaw;
    sendMessage.pitch -= platform.pitch;
    AngleCorrect(sendMessage.yaw);
    AngleCorrect(sendMessage.pitch);
    if( go_to_circle){
        G_buffinformation.tmpparam0=100000;
        sendMessage.beat = false;
        sendMessage.yaw = 0;

    }

#ifdef DEBUG
    if(G_setval.WORK){
        G_buffinformation.predict_point=predict_point_;
        G_buffinformation.diffsize=Size2f(float(deta_r),float(deta_g));
        G_buffinformation.realdiff=Size2f(deta_x,deta_y);
    }
#endif
}