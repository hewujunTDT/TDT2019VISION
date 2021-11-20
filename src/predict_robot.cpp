#include "predict.h"
#include "debug.h"
#include "targetresolver.h"
//////////////////////////Predictor//////////////////////////////
void Vasual(float data1, float data2,int min,int max);

Predictor::Predictor() {;}

//////////////////////////RobotPredictor//////////////////////////////

RobotPredictor::RobotPredictor() :Predictor() {
    for(int i=0;i<5;i++){
        if(i<4){//前四个滤波器是对四个装甲板的速度进行滤波的
            KF_[i]=_1KalmanFilter(1,1,1,100, true);
        } else{//第五个用于对轴的yaw进行滤波
            KF_[i]=_1KalmanFilter(1,1,1,100, false);
        }
        if(i<4){//初始化过程噪声和测量噪声
            KF_[i].set_processNoiseCov(0.0001f);
            KF_[i].set_measurementNoiseCov(0.01f);
        } else{
            KF_[i].set_processNoiseCov(0.01f);
            KF_[i].set_measurementNoiseCov(6);
        }
    }
    lastbeat_j_=-1;
    lastbeat_time_=0;
}

void RobotPredictor::set_new_target(RobotTarget &target) {
    if(history_targets_.empty()){
        history_targets_.push_back(target);
    } else{
        bool same_robot=history_targets_.back().get_robot_type()==target.get_robot_type();
        bool same_time=target.get_time_now()-history_targets_.back().get_time_now()<100;
        if(same_robot&&same_time){//空间时间连续 车种相同
            history_targets_.push_back(target);
        } else{ // 清空之前的数据,开始一个新的预测周期
            vector<RobotTarget>{target}.swap(history_targets_);
        }
        if(history_targets_.size()>100){//只储存若干帧
            lastgoodid=0;
            history_targets_.erase(history_targets_.begin());
        }
    }
#ifdef DEBUG
    if(G_setval.WORK){
        G_armorinformation.target=target;
    }
#endif
}

void RobotPredictor::calc_spinning_speed() {
    /////////计算目标的旋转速度////////
    // todo 现在的数据是离散的,测量精度也大概够了,如果拟合出连续的数据,dd
    // todo 现在是至少转过1/4圈才能反应过来,理论上可以优化到1/8圈d
    G_armorinformation.tmpparam2=0;

    if(history_targets_.size()<15){ // 15帧算不出来转速的
        spinning_speed_=0;
    } else{
        vector<int> idexs;
        vector<float> angles;
        for(int i=0;i<history_targets_.size();i++){
            const vector<FinitePlane> &planes=history_targets_[i].get_planes();
            if(!planes[0].empty()&&planes[1].empty()){ //意思是只有只有一个装甲板,因为正对情况肯定只能看见一个装甲板
                float diffyaw=fabs(history_targets_[i].get_axis_yaw()-planes[0].get_polor_point().yaw);
                // 目标轴  心与当前装甲板的yaw偏差值
                if(fabs(diffyaw*planes[0].get_polor_point().distance)<1.5){ // 偏差很小的情况,认为是这个装甲板是正对着镜头的,并记录下它
                    idexs.push_back(i);
                    if(i==history_targets_.size()-1){
                        G_armorinformation.tmpparam2=1;
                    }
                }
            }
        }
        if(idexs.size()<2){
            spinning_speed_=0;
        } else{ // 找到了至少两个装甲板正对着镜头的情况
            lastgoodid=idexs[idexs.size()-1];
            int sum=0;
            for(int i=idexs[0]+1;i<=lastgoodid;i++){ // 区间是从第一次正对镜头到最后一次正对镜头
                if(history_targets_[i].get_linkid()==1){
                    sum+=1;
                } else if(history_targets_[i].get_linkid()==3){
                    sum+=-1;
                }
            } // sum的含义是这个区间内车体旋转了多少个1history_targets_/4圈
            spinning_speed_=1000*(sum*kPI/2)/float(history_targets_[lastgoodid].get_time_now()-
                    history_targets_[idexs[0]].get_time_now());//旋转圈数除以时间差
        }
    }
}

void RobotPredictor::predict(const ShootPlatform &platform) {
    static  ShootPlatform lastplatform=platform; // 初始化静态变量,上一次云台信息
    static  vector<float> history_sv; // 初始化静态变量,历史云台速度信息
    static  int _j_link=0; // 初始化静态变量,_j_link是一个稳定的指标,KF_,predict_polors_,current_polors_都以它来定序
    // _j_link的定义是当前预测周期内,所有已知target的linkid之和,具体往下看

    vector<RobotTarget>::iterator current_iter,last_iter;//用法相当于指针
    if(history_targets_.size()>1){
        current_iter=history_targets_.end()-1;//vector中最后一个target
        last_iter=history_targets_.end()-2;//vector中倒数第二个target
    } else if(history_targets_.size()==1){//当只有一个target的情况下,伪造一个lasttarget
        current_iter=history_targets_.end()-1;
        last_iter=history_targets_.end()-1;
    } else{return;}
    RobotTarget &current=*(current_iter);
    RobotTarget &last=*(last_iter);



    /////////计算轴的位置////////
    // 计算轴的位置是小陀螺算法中最最重要的部分
    // todo 当前计算轴的yaw并进行kalman滤波,原地小陀螺实测效果不错,但是运动小陀螺效果未知,需要进行相应的调参和优化
    float _pn=0.025,_mn;
    if(current.plane1_is_empty()){//只有一个装甲版
        _mn=0.3;
    } else{//有两个装甲版
        _mn=0.01;
    }
    _pn=_pn*float(current.get_time_now()-last.get_time_now())/10;//过程噪声的表达式
    KF_[4].set_measurementNoiseCov(_mn*_mn);
    KF_[4].set_processNoiseCov(_pn*_pn);
    float axis_yaw=current.get_axis_yaw();//得到轴的yaw值
    int flag;
    if(axis_yaw>0){
        flag=1;
    } else{
        flag=-1;
    }
    float temp=axis_yaw;
    axis_yaw=fabs(axis_yaw);
    if(history_targets_.size()==1){
        _j_link=0;//当前预测周期开始,会清零_j_link_
        KF_[4].correct(axis_yaw,false);//矫正观测值
    } else{
        _j_link+=current.get_linkid();//更新_j_link_,
        KF_[4].correct(axis_yaw);//矫正观测值
    }
    if(KF_[4].get_statePost()>kPI)
    {
        current.set_axis_yaw(-1*flag*(2*kPI-KF_[4].get_statePost()));//应用//tmp_delta = 2*kPI-KF_[4].get_statePost();
    } else
    {
        current.set_axis_yaw(KF_[4].get_statePost()*flag);//应用
    }

   // Vasual(temp,current.get_axis_yaw(),-4,4);
    axis_yaw=current.get_axis_yaw();
    calc_spinning_speed(); // 计算旋转速度,会更新成员spinning_speed_
    // todo 在敌方高速小陀螺情况下,小陀螺模式将会采用定点或定范围的打击方式,而进入小陀螺模式的决定权理应放在操作手那里
    cout<<"spinning_speed_"<<spinning_speed_<<endl;
    float axis_pitch;
    if(current.get_planes()[1].empty()) {
         axis_pitch = current.get_planes()[0].get_polor_point().pitch;
    } else{
         axis_pitch=current.get_planes()[1].get_polor_point().pitch;
    }
    float axis_dist=current.get_planes()[0].get_polor_point().distance;
    axis_polor_={axis_dist,axis_yaw,axis_pitch};

    ////////计算测量噪声和过程噪声////////
    // 降低测量误差和预测误差并不容易,但是只要我们知道当前的测量误差和预测误差大概是多少,kalmanfiliter就能帮我们解决剩下的问题
    // todo 实时更新测量噪声核过程噪声在实践上效果有了显著的改进,但当前的计算方式和算法参数并没有怎么调过,代码上车之后需要进一步调参
    float pn=0.025,mn;
    float _sv=1000*(platform.yaw-lastplatform.yaw)/float(current.get_time_now()-last.get_time_now()+0.00001f);//计算云台角速度

    history_sv.push_back(_sv);//储存
    if(history_sv.size()>5){//history_sv的size控制在5
        history_sv.erase(history_sv.begin());
    }
    float _asv=0;
    for(int j=0;j<history_sv.size()-1;j++){//计算_asv,物理意义为云台速度变化率
        _asv+=pow(history_sv[j+1]-history_sv[j],2);
    }
    mn=0.20f+fabs(_sv*_sv)*0.2f+fabs(_asv)*0.6f;//测量噪声的表达式
    pn=pn*float(current.get_time_now()-last.get_time_now())/10;//过程噪声的表达式
//    cout<<"mn"<<mn<<",pn"<<_pn<<endl;
    lastplatform=platform;//更新lastplatform

    ////////对于四个装甲板矫正速度,并预测yaw////////
    // 对当前车体的四个装甲板都会进行预测(可以预测的),而且现在对于掉帧(漏识别)有一定鲁棒性,至于打哪个会在之后做出决策
    // todo 以下代码在逻辑上已调通但仍然有已知的进步空间,可能会出现未知的报错,需要更多的测试
    for(int i=0;i<4;i++){
        int j=(i+_j_link)%4;//在KF_,current_polors_,predict_polors_容器中,j是i的真实稳定的index
        //在一个预测周期内,一个固定的j(比如0)永远指向一个固定的装甲板

        int i0=i;
        int i1=(i+current.get_linkid())%4;//i1和i0值可能不同,但是指向当前和上次的同一个装甲板
        if(!current.get_planes()[i0].empty()&&!last.get_planes()[i1].empty()){
            Polor3f current_polor=current.get_planes()[i0].get_polor_point();
            Polor3f last_polor=last.get_planes()[i1].get_polor_point();
            float _v=10*getDiffangle(current_polor.yaw,last_polor.yaw)*current_polor.distance
                     /float(current.get_time_now()-last.get_time_now());//线速度

            KF_[j].set_measurementNoiseCov(mn*mn);
            KF_[j].set_processNoiseCov(pn*pn);
            if(true){
                float _diffangle=getDiffangle(current_polor.yaw,axis_polor_.yaw);

                float _radius=sin(_diffangle)*current_polor.distance;

                float _spv=0.3*spinning_speed_;
                G_armorinformation.tmpparam4=_spv;

                if(_diffangle>0){
                        G_armorinformation.tmpparam0=_v;

                        _v-=0.6*_spv*_radius/30;
                    G_armorinformation.tmpparam1=_v;


                } else{
                    G_armorinformation.tmpparam2=_v;
                    _v*=0.8;
                    _v-=1.0*_spv*_radius/30;
                    G_armorinformation.tmpparam3=_v;

                }

            }
            if(history_targets_.size()==1) {

                KF_[j].correct(_v, false); // 矫正观测值
            } else {
                KF_[j].correct(_v); // 矫正观测值

            }
            current_polors_[j]=current_polor;
            float T=current_polor.distance/platform.bulletspeed+10;  // 补偿时间为距离除以速度
            current_polor.yaw+=T*KF_[j].get_statePost()/(10*current_polor.distance);//根据观测的速度对yaw进行预测
            predict_polors_[j]=current_polor;

        } else{//目标丢失,不连续
            if(current.get_time_now()-KF_[j].get_last_time()<50){ // 在50ms之内还可以忍受
                Polor3f current_polor=predict_polors_[j];
                current_polors_[j]=current_polor;
                float T=current_polor.distance/2.7f+10;  // 补偿时间为距离除以速度
                current_polor.yaw+=T*KF_[j].get_statePost()/(10*current_polor.distance);
                predict_polors_[j]=current_polor;

            } else{ //不能忍受
                if(current.get_planes()[i0].empty()){//当前帧没有检测到,将预测点置为-1,当它消失了
                    predict_polors_[j]={-1,-1,-1};
                } else{//当前帧检测到了,将速度清零
                    KF_[j].correct(0, false);//矫正观测值,fasle意为数据不连续了
                    Polor3f current_polor=current.get_planes()[i0].get_polor_point();
                    current_polors_[j]=current_polor;
                    float T=current_polor.distance/platform.bulletspeed+10;  //补偿时间为距离除以速度
                    current_polor.yaw+=T*KF_[j].get_statePost()/(10*current_polor.distance);
                    predict_polors_[j]=current_polor;
                }
            }
        }
    }



#ifdef DEBUG
    if(G_setval.WORK){
        for (int i = 0; i < 5; ++i) {
            G_armorinformation.kf_[i]=_1KalmanFilter(KF_[i]);
        }
        for (int i = 0; i < 4; ++i) {
            G_armorinformation.predict_polors_[i]=predict_polors_[i];
        }
        G_armorinformation.axis_polor_=axis_polor_;

    }
#endif
}

void RobotPredictor::calc_Output(ReciveMessage const &reciveMessage,SendMessage &sendMessage){//重力补偿,返回云台应该转的位置
    spinningTop_form_= false; // 小陀螺模式关闭
    cout<<"开火模式"<<spinningTop_form_<<endl;
    ShootPlatform const &platform=reciveMessage.shoot_platform;
    this->predict(platform);//先预测,更新各成员变量
    if(fabs(spinning_speed_)>=2)spinningTop_form_=true;
    ////////决策要指向的位置////////
    // todo 可以加入更多的考虑因素,不同因素的影响了需要进一步调参
    Polor3f oriented_polor; // 要指向位置的极坐标
    if(spinningTop_form_){ //小陀螺模式,直接对准轴
        oriented_polor=axis_polor_;
        if(spinning_speed_>0&&get_timenow()-lastbeat_time_<110){
            for(int i=0;i<4;i++){
                if(predict_polors_[i].distance==-1) continue; // 短时间内没有检测到此装甲板
                if(getDiffangle(current_polors_[i].yaw,axis_polor_.yaw)<0)continue;
                float  k=float(get_timenow()-lastbeat_time_)/110;
                //oriented_polor.yaw=k*predict_polors_[i].yaw+(1-k)*axis_polor_.yaw;
             }
        }
    } else{ // 跟随模式
        float max_score=0; // 初始化最高分数
        int beat_j=0; // 最终决定的要击打装甲板的index
        for(int i=0;i<4;i++){
            if(predict_polors_[i].distance==-1) continue; // 短时间内没有检测到此装甲板
            float tmp_score=100; // 总分
            // tmp_score-=fabs(tan(platform.yaw-oriented_polors_[i].yaw))*oriented_polors_[i].distance;
            // 云台距离目标的远近,加上这个会抖,可以乘个较小的系数



            if(fabs(spinning_speed_)>1){//如果发现目标在顺时针旋转，优先打右边的装甲板
                if(spinning_speed_>0&&current_polors_[i].yaw>axis_polor_.yaw){
                    tmp_score+=50;
                } else if(spinning_speed_<0&&current_polors_[i].yaw<axis_polor_.yaw){//反之左边的
                    tmp_score+=50;
                }
            }else{
                tmp_score-=fabs(tan(axis_polor_.yaw-current_polors_[i].yaw))*predict_polors_[i].distance*2;
                // 目标点距离轴的远近，和装甲板的大小正相关
                if(i==lastbeat_j_){//与上次目标相同
                    tmp_score+=10;
                }
            }
            if(tmp_score<0) tmp_score=1; // 限值,只要这个目标装甲板在视野内,它的底分就是1
            if(tmp_score>max_score) { // 更新max_score和beat_j
                max_score=tmp_score;
                beat_j=i;
            }
        }
        oriented_polor=predict_polors_[beat_j];
        lastbeat_j_=beat_j;

        if(max_score==0){ // 这种情况应该是一个可击打目标也没有
            sendMessage.no_object = true;
            sendMessage.beat = false;
            sendMessage.pitch = 0;
            sendMessage.yaw = 0;
        }

    }

    if(oriented_polor.distance<500){
        oriented_polor.pitch-=0.015f;
    }
    ////////重力补偿,角度解算////////
    Point3f world_point=to_sprectangular(oriented_polor);
    float x1,y1;//平面直角坐标系,抛物线过点(x1,y1)
    y1=world_point.y;
    x1=sqrt(world_point.x*world_point.x+world_point.z*world_point.z); // 平面坐标系中的x1,y1
    Vec2f phi=parabolasolve(Point2f(x1,y1),reciveMessage.shoot_platform.bulletspeed); // 过点(x1,y1)解抛物线方程,得到两个出射角的解
    sendMessage.pitch=fabs(phi[0]-platform.pitch)<fabs(phi[1]-platform.pitch)?phi[0]:phi[1];
    sendMessage.yaw=oriented_polor.yaw;

    /////开火判断/////
    float deta_x,deta_y;
    deta_x=tan(oriented_polor.yaw-platform.yaw)*x1;//x轴方向偏差
    deta_y=(tan(sendMessage.pitch-platform.pitch))*x1;//y轴方向偏差
    int type= static_cast<int>(history_targets_.back().get_robot_type());
    if(type==1||type==6){//大装甲板
        sendMessage.beat=fabs(deta_x)<21.8/2&&fabs(deta_y)<10.f/2;
    }else{//小装甲板
        sendMessage.beat=fabs(deta_x)<11.8/2&&fabs(deta_y)<10.f/2;
    }
    if(spinningTop_form_){
        sendMessage.beat=sendMessage.beat=fabs(deta_x)<21.8/2&&fabs(deta_y)<10.f/2;
    }
    if(sendMessage.beat&&spinningTop_form_){ // 小陀螺模式,指向轴心了再判断是否在开火区间内
        // 这种模式是通过开火时机来进行预测
        double lasttime=history_targets_[lastgoodid].get_time_now();
        float T=history_targets_.back().get_planes()[0].get_polor_point().distance/reciveMessage.shoot_platform.bulletspeed+140;
        // 补偿时间为距离除以速度加开火延迟，这种模式下必须要考虑到开火延迟

        float kt2=1000*kPI/(2*fabs(spinning_speed_)+0.00001f);
        float limitt=0; //开火区间
        if(type==1||type==6){//大装甲板
            limitt=(21.8f/2)/(fabs(spinning_speed_)*28/1000)*0.8f;
        }else{ // 小装甲板
            limitt=(11.8f/2)/(fabs(spinning_speed_)*28/1000)*0.8f;
        }
         limitt=15; // 开火区间太低了反而开不出火来

         float judge_time=(float(get_timenow() - lasttime) + T - kt2);
        if (judge_time >= -10&&judge_time<20) {
            sendMessage.beat = true;
        } else { // 当前tmppa时间在开火区间内，则开火
            sendMessage.beat = false;
        }
        G_armorinformation.tmpparam0=fabs(float(get_timenow() - lasttime));
        G_armorinformation.tmpparam1=T;
        G_armorinformation.tmpparam3=kt2;
//
    }
//    if(oriented_polor.yaw-platform.yaw>0){
//        sendMessage.beat=1;
//    }
//    if(get_timenow()-lastbeat_time_<200){
//        sendMessage.beat=false;
//    }

    if(sendMessage.beat){
        lastbeat_time_=get_timenow();
    }

//G_armorinformation.tmpparam3=get_timenow()-lastbeat_time_;
    sendMessage.no_object = false;
    sendMessage.yaw-=platform.yaw; // 求相对的yaw
    sendMessage.pitch-=platform.pitch; // 求相对的pitch
    AngleCorrect(sendMessage.yaw); // 限幅
    AngleCorrect(sendMessage.pitch); // 限幅

#ifdef DEBUG
    if(G_setval.WORK){
        G_armorinformation.oriented_polor_=oriented_polor;
        if(type==1||type==6){
            G_armorinformation.diffsize=Size2f(21.8f/2,10.f/2);
        }else{
            G_armorinformation.diffsize=Size2f(11.8f/2,10.f/2);
        }
        G_armorinformation.realdiff=Size2f(deta_x,deta_y);
    }
#endif
}
void Vasual(float data1, float data2,int min,int max){
    static int x=0;
    static Mat show(500,2000,CV_8UC3,Scalar(255,255,255));
    x=x+5;
    if(x>1999){
        show=(500,2000,CV_8UC3,Scalar(255,255,255));
    }
    x=x%2000;
    int y1=500*(data1-min)/(max-min);
    int y2=500*(data2-min)/(max-min);
    Point point1(x,y1);
    Point point2(x,y2);
    circle(show,point1,5,Scalar(0,255,255),1,8,0);
    circle(show,point2,5,Scalar(0,0,255),1,8,0);
    imshow("Visual",show);
    waitKey(1);
}
