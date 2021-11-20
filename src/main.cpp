#include <thread>
#include <sys/ioctl.h>
#include "targetresolver.h"
#include "energybuff.h"
#include "readparameter.h"
#include "camera.h"
#include "armordetect.h"
#include "usart.h"
#include "predict.h"
#include "debug.h"
#include "lob.h"
#include "log.h"

#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

using namespace std;
using namespace cv;

int *child_time;//子进程传给父进程的时间，用来防止子进程阻塞
#define WATCH_DOG//看门狗模式，平时测试建议用看门狗模式，顺便测试看门狗有没有bug。如果程序停止则说明有bug.普通模式直接把这句话注释
#define RECORDER//录像
static struct termio term_orig;
static int kbdflgs;
/**********检测是否有键盘敲击**************/
int kbhit(void)
{
    struct timeval tv;
    struct termios old_termios, new_termios;
    int error;
    int count = 0;
    tcgetattr(0, &old_termios);
    new_termios = old_termios;
    new_termios.c_lflag &= ~ICANON;
    new_termios.c_lflag &= ~ECHO;
    new_termios.c_cc[VMIN] = 1;
    new_termios.c_cc[VTIME] = 0;
    error = tcsetattr(0, TCSANOW, &new_termios);
    tv.tv_sec = 0;
    tv.tv_usec = 100;
    select(1, NULL, NULL, NULL, &tv);
    error += ioctl(0, FIONREAD, &count);
    error += tcsetattr(0, TCSANOW, &old_termios);
    return error == 0 ? count : -1;
}

static Mat src;
static mutex  src_mutex;//互斥锁
void Thread(Camera* camera,Usart usart);//主线程
void Thread_Getmat(Camera* camera);//取帧
void Thread_recorder();
void son_process();

int main(){
    initLogger();////日志系统////////
    time_start=get_timenow();
    ReadParameter read_parameter;

#ifdef WATCH_DOG
    while(1) {
        OERROR("看门狗杀死程序，重启");
        pid_t pid_flag;
        child_time = (int *) mmap(NULL, sizeof(int), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        pid_flag = fork();
        if (pid_flag == 0) {
            son_process();
            break;
        } else if (pid_flag > 0) {
            father_process(pid_flag,child_time);
        } else {
            cout << "创建进程失败" << endl;
        }
    }
#endif
#ifndef WATCH_DOG
    son_process();
#endif
    return 0;
}

void son_process()
{
//    VideoDebug camera("/home/tdt/桌面/国赛derta8.5/7.25RONGHE/T-DT2019Vision/videos/09-07-23:49:33.avi");
    HikvisionCam camera(0,0);
    Usart usart;

    thread t1(Thread_Getmat,&camera);     //取帧线程
    thread t2(Thread,&camera,usart);
#ifdef RECORDER
    thread t3(Thread_recorder);
#endif
    t1.join();
    t2.join();
#ifdef RECORDER
    t3.join();
#endif

}
void Thread(Camera* camera,Usart usart) {
    ArmorDetector armor_detector(*camera);
    EnergyBuffDetector energy_buff_detector(*camera);
    BridgeLob bridge_lob(*camera);
    TargetResolver target_resolver(camera);
    RobotPredictor robot_predictor;
    EnergyBuffPredictor energybuff_predictor;
    Debugger debugger(camera->get_matrix(), camera->get_dist_coeffs(), Parameter);
    ReciveMessage recive_message;
    SendMessage send_message;
    BulletSpeedSolve bulletSpeedSolve;
    Mat tmpsrc;
    while (true) {

//        camera->get_mat(src);
        if (src.empty()) {
            continue;
        }
        Scalar jud = sum(src);
        Scalar jud1 = sum(tmpsrc);
        if (jud[0] == jud1[0] && jud[1] == jud1[1])continue;

        tmpsrc = src.clone();

        usart.UsartSend(send_message);
#ifdef WATCH_DOG
        feed_dog();
#endif

        double time_now = get_timenow();
        double time_start=getTickCount();
        static Mode last_mode = ArmorMode;


#ifdef DEBUG
        if (G_setval.WORK) {
            if (last_mode != G_setval.detectmode) {
                if (G_setval.detectmode == EnergyBuffMode) {
                    camera->set_exposure(Parameter.cam_parameter[1].exposure);
                    camera->set_gain(Parameter.cam_parameter[1].gain);
                    camera->set_gamma(Parameter.cam_parameter[1].gamma);
                    camera->set_black_level(Parameter.cam_parameter[1].black_level);
                }
                if (G_setval.detectmode == ArmorMode) {
                    camera->set_exposure(Parameter.cam_parameter[0].exposure);
                    camera->set_gain(Parameter.cam_parameter[0].gain);
                    camera->set_gamma(Parameter.cam_parameter[0].gamma);
                    camera->set_black_level(Parameter.cam_parameter[0].black_level);
                }
            }
            last_mode = G_setval.detectmode;
        }
#endif


        volatile int data[7];
        //send_message.visiononline++;
        if (last_mode == ArmorMode) {//装甲识别
            cout<<last_mode<<endl;
            RobotTarget target;
            vector<Armor> armors;
            unsigned char find_object=get_find_object();
            armors = armor_detector.Get(tmpsrc, recive_message);
            usart.UsartRecv(data);
            recive_message.shoot_platform.yaw = -(data[0] * 1.f / 1000 / (180 / kPI));
            recive_message.shoot_platform.pitch = (data[1] * 1.f / 100 / (180 / kPI));
            recive_message.mode = (Mode) data[2];
            recive_message.bulletspeed = data[3] / 8.5f * 0.1f;
            recive_message.shoot_platform.bulletspeed = bulletSpeedSolve.solve_speed(recive_message.bulletspeed,
                                                                                     time_now);
            recive_message.firemode = (bool) data[4];
            recive_message.lock_command = bool(data[5]);
            recive_message.enemy_color = data[6] * 2;
            Parameter.enemy_colour = recive_message.enemy_color;
            if (!G_setval.WORK) {
                if (recive_message.mode != last_mode) {
                    if (recive_message.mode == EnergyBuffMode) {
                        camera->set_exposure(Parameter.cam_parameter[1].exposure);
                        camera->set_gain(Parameter.cam_parameter[1].gain);
                        camera->set_gamma(Parameter.cam_parameter[1].gamma);
                        camera->set_black_level(Parameter.cam_parameter[1].black_level);
                        last_mode = EnergyBuffMode;
                        continue;
                    } else {
                        last_mode = recive_message.mode;
                        continue;
                    }
                }
            }

            if (armors.empty()) {
                send_message.no_object = true;
                send_message.beat = false;
                send_message.pitch = 0;
                send_message.yaw = 0;
                if(find_object==49)
                    send_message.find_object_left=1;
                if(find_object==50)
                    send_message.find_object_right=1;
                if(find_object==51) {
                    //cout<<"aaaaaaaaa"<<endl;
                    send_message.find_object_left = 0;
                }
                if(find_object==52) {
                    //cout<<"dddddddddd"<<endl;
                    send_message.find_object_right = 0;
                }
                cout << "no armor" << endl;
            } else {
                target = target_resolver.RobotResolver(armors, recive_message);
                target.set_time_now(time_now);
                robot_predictor.set_new_target(target);
                robot_predictor.calc_Output(recive_message, send_message);
                if(find_object==49)
                    send_message.find_object_left=1;
                if(find_object==50)
                    send_message.find_object_right=1;
                if(find_object==51)
                    send_message.find_object_left=0;
                if(find_object==52)
                    send_message.find_object_right=0;
            }
        } else if (last_mode == EnergyBuffMode) {  //能量机关
            cout << "buff" << endl;
            BuffTarget buff_target;
            vector<EnergyBuffArmor> buff_armors;
            buff_armors = energy_buff_detector.Get(tmpsrc);

            usart.UsartRecv(data);
            recive_message.shoot_platform.yaw = (data[0] * 1.f / 1000 / (180 / kPI))*0;
            recive_message.shoot_platform.pitch = (data[1] * 1.f / 100 / (180 / kPI));
            recive_message.mode = (Mode) data[2];
            recive_message.bulletspeed = data[3] / 8.5f * 0.1f;
            recive_message.shoot_platform.bulletspeed = bulletSpeedSolve.solve_speed(recive_message.bulletspeed,
                                                                                     time_now);
            recive_message.shoot_platform.bulletspeed=2.75;
            if (recive_message.shoot_platform.bulletspeed < 2.5f) {
                recive_message.shoot_platform.bulletspeed = 2.8f;
            }
            recive_message.firemode = (bool) data[4];
            recive_message.lock_command = bool(data[5]);
            recive_message.enemy_color = (data[6]) * 2;
            Parameter.enemy_colour = recive_message.enemy_color;
            AngleCorrect(recive_message.shoot_platform.yaw);
            AngleCorrect(recive_message.shoot_platform.pitch);

            if (!G_setval.WORK) {
                if (last_mode != recive_message.mode) {
                    if (recive_message.mode != EnergyBuffMode) {
                        camera->set_exposure(Parameter.cam_parameter[0].exposure);
                        camera->set_gain(Parameter.cam_parameter[0].gain);
                        camera->set_gamma(Parameter.cam_parameter[0].gamma);
                        camera->set_black_level(Parameter.cam_parameter[0].black_level);
                        last_mode = recive_message.mode;
                        continue;
                    }
                }
            }

                if (buff_armors[0].empty()) {
                    energybuff_predictor.calc_Output(recive_message, send_message);

                } else {
                    buff_target = target_resolver.EnergyBuffResolver(buff_armors, recive_message);
                    buff_target.set_time_now(time_now);
                    energybuff_predictor.set_new_target(buff_target);
                    energybuff_predictor.calc_Output(recive_message, send_message);
                }
            } else if (last_mode == LobMode) {
            cout<<last_mode<<endl;
            usart.UsartRecv(data);
                recive_message.shoot_platform.yaw = (data[0] * 1.f / 1000 / (180 / kPI));
                recive_message.shoot_platform.pitch = (data[1] * 1.f / 100 / (180 / kPI));
                recive_message.mode = (Mode) data[2];
                recive_message.bulletspeed = data[3] / 8.5f * 0.1f;
                recive_message.shoot_platform.bulletspeed = bulletSpeedSolve.solve_speed(recive_message.bulletspeed,
                                                                                         time_now);
                cout<<"speed        "<<recive_message.shoot_platform.bulletspeed<<endl;

                if (recive_message.shoot_platform.bulletspeed < 2.0f) {
                    recive_message.shoot_platform.bulletspeed = 2.8f;
                }
                recive_message.firemode = (bool) data[4];
                recive_message.lock_command = bool(data[5]);
                Parameter.enemy_colour = recive_message.enemy_color;
                AngleCorrect(recive_message.shoot_platform.yaw);
                AngleCorrect(recive_message.shoot_platform.pitch);
                if (!G_setval.WORK) {
                    if (last_mode != recive_message.mode) {
                        if (recive_message.mode == EnergyBuffMode) {
                            camera->set_exposure(Parameter.cam_parameter[1].exposure);
                            camera->set_gain(Parameter.cam_parameter[1].gain);
                            camera->set_gamma(Parameter.cam_parameter[1].gamma);
                            camera->set_black_level(Parameter.cam_parameter[1].black_level);
                            last_mode = EnergyBuffMode;
                            continue;
                        } else {
                            last_mode = recive_message.mode;
                            continue;
                        }

                    }
                }

                    bridge_lob.Get(src, recive_message, send_message);

            }
#ifdef DEBUG
            if (G_setval.WORK) {
                if (debugger.get_setval().detectmode == ArmorMode) {
                    G_armorinformation.reciveMessage = recive_message;
                    G_armorinformation.sendMessage = send_message;
                    G_armorinformation.timenow = time_now;
                    debugger.set_armor_infomation(G_armorinformation);
                } else {
                    G_buffinformation.reciveMessage = recive_message;
                    G_buffinformation.sendMessage = send_message;
                    G_buffinformation.timenow = time_now;
                    debugger.set_buff_infomation(G_buffinformation);
                }

                debugger.show();
                G_setval = debugger.get_setval();
                debugger.CorrectAll(G_setval, camera, Parameter);
                G_armorinformation = ArmorDebugInformation();
                G_buffinformation = BuffDebugInformation();
            }
            if (G_setval.QUIT)break;
#endif
        double time_end=((getTickCount()-time_start)/getTickFrequency())*1000;
        if(time_end>40) OWARNING("帧率过低!!!FPS:"+to_string(1000/time_end));
        }

    }
void Thread_Getmat(Camera *camera){
    int flag=0;
    while(true){

        src_mutex.lock();
        if(!camera->get_mat(src)){
            flag++;
            src_mutex.unlock();
            if(flag>=5){
                flag=0;
                camera->RestartCam();
            }
            continue;
        }
        src_mutex.unlock();
#ifdef  CALIBRATE
        static int CalibrateFlag=1;
        if (CalibrateFlag==1)
        {

            if (!camera->CameraCalibrate(src))//循环直到相机标定完成
                continue;
            else
                CalibrateFlag=0;
        }
#endif
        if(G_setval.QUIT)break;

    }
}
void Thread_recorder(){
    Videocorder videocorder("../videos/");
    while(getDiskfreespace()>10){
        if(G_setval.QUIT)break;
        Mat frame=Mat::zeros(Size(1440,1080),CV_8UC3);
        if(src.empty())
            continue;
        Scalar jud=sum(src.col(500));
        Scalar jud1=sum(frame.col(500));
        if(jud[0]==jud1[0])continue;//等待直到src更新
        src.copyTo(frame);
        struct timeval tv;
        gettimeofday(&tv, NULL);
        videocorder.Recorder(frame);

    }
}