/*****************************************************************************
 * @brief        从 .yaml 文件读取参数
 * @version      1.0.0.1
 *
 * @author       黎容熙
 * @qq           919935013
 *
 *----------------------------------------------------------------------------
 * Change History :
 * <Date>     | <Version> | <Author> | <Description>
 *----------------------------------------------------------------------------
 * 2019/02/16 | 1.0.0.1   | 黎容熙    | 代码规范
 *----------------------------------------------------------------------------
 *
*****************************************************************************/

#include "readparameter.h"
#include <sys/file.h>
#include <iostream>


using namespace std;
const std::string kParameterAddress = "../config/config.yaml";

Struct_Parameter Parameter;

ReadParameter::ReadParameter() {
    if ((flock(1, LOCK_EX | LOCK_NB)) < 0) {
        std::cout << "Log::The log have been locked" << std::endl;
    } else {
        time_t timep;
        time(&timep);
        strftime(Parameter.date, sizeof(Parameter.date), "%m-%d-%H:%M:%S", localtime(&timep));
        string file_address = kParameterAddress;
        cv::FileStorage fs(file_address, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            cout << "the address of config.yaml is error!(file)\n" << endl;
        } else {
            fs["enemy_colour"] >> Parameter.enemy_colour;
            fs["firing_rate"] >> Parameter.firing_rate;
            fs["gun_height"] >> Parameter.gun_height;
            fs["pitch_k"] >> Parameter.pitch_k;
            fs["ArmorDetect_lightbarthre"] >>Parameter.ArmorDetect_lightbarthre;
            fs["EnergyBuffDetect_gray_thre"] >> Parameter.EnergyBuffDetect_gray_thre;
            fs["EnergyBuffDetect_split_thre"] >> Parameter.EnergyBuffDetect_split_thre;
            fs["EnergyBuffDetect_dialesize"] >> Parameter.EnergyBuffDetect_dialesize;
            fs["EnergyBuffDetect_erodesize"] >> Parameter.EnergyBuffDetect_erodesize;
            fs["DebugWork"] >> Parameter.DebugWork;
            fs["lob_yaw_offset"] >> Parameter.lob_yaw_offset;
            fs["lob_pitch_offset"] >> Parameter.lob_pitch_offset;
            fs["VideoRecordWork"] >> Parameter.VideoRecordWork;
            cv::FileNode camera = fs["camera"];
            for (int i = 0; i < 2; i++) {
                CamParameter temp_parameter;
                camera[i]["fps"] >> temp_parameter.fps;
                camera[i]["width"] >> temp_parameter.width;
                camera[i]["height"] >> temp_parameter.height;
                camera[i]["exposure_red"] >> temp_parameter.red_exposure;
                camera[i]["exposure_blue"] >> temp_parameter.blue_exposure;
                if (Parameter.enemy_colour == 2) {
                    temp_parameter.exposure= temp_parameter.red_exposure;
                } else {
                    temp_parameter.exposure= temp_parameter.blue_exposure;
                }
                camera[i]["gain"] >> temp_parameter.gain;
                camera[i]["brightness"] >> temp_parameter.brightness;
                camera[i]["saturation"] >> temp_parameter.saturation;
                camera[i]["contrast"] >> temp_parameter.contrast;
                camera[i]["gamma"] >> temp_parameter.gamma;
                camera[i]["sharpness"] >> temp_parameter.sharpness;
                camera[i]["black_level"] >> temp_parameter.black_level;

                camera[i]["balance_value"] >> temp_parameter.balance_value;
                camera[i]["balance_red"] >> temp_parameter.balance_ratio_red;
                camera[i]["balance_green"] >> temp_parameter.balance_ratio_green;
                camera[i]["balance_blue"] >> temp_parameter.balance_ratio_blue;

                camera[i]["matrix"]>>temp_parameter.camera_matrix;
                camera[i]["dist_coeffs"]>>temp_parameter.dist_coeffs;
                camera[i]["save_res"]>>temp_parameter.save_res;
                camera[i]["save_runlog"]>>temp_parameter.save_runlog;
                camera[i]["video_fps"]>>temp_parameter.video_fps;
                camera[i]["video_path"]>>temp_parameter.video_path;
                Parameter.cam_parameter.push_back(temp_parameter);
            }
            fs.release();
        }
        if((flock(1, LOCK_UN ))<0)
        {
            std::cout<<"Log::unlock the logfile error"<<std::endl;
        }
    }
}
void ReadParameter::WriteParameter() {
    if ((flock(1, LOCK_EX | LOCK_NB)) < 0) {
        std::cout << "Log::The log have been locked" << std::endl;
    } else {
        time_t timep;
        time(&timep);
        strftime(Parameter.date, sizeof(Parameter.date), "%m-%d-%H:%M:%S", localtime(&timep));
        string file_address = kParameterAddress;
        cv::FileStorage fs(file_address, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            cout << "the address of config.yaml is error!(file)\n" << endl;
        } else {
            fs<<"enemy_colour"<< Parameter.enemy_colour;
            fs<<"firing_rate"<< Parameter.firing_rate;
            fs<<"gun_height"<< Parameter.gun_height;
            fs<<"pitch_k"<< Parameter.pitch_k;
            fs<< "ArmorDetect_lightbarthre"<<Parameter.ArmorDetect_lightbarthre;
            fs<< "EnergyBuffDetect_gray_thre"<< Parameter.EnergyBuffDetect_gray_thre;
            fs<< "EnergyBuffDetect_split_thre"<< Parameter.EnergyBuffDetect_split_thre;
            fs<< "EnergyBuffDetect_dialesize"<< Parameter.EnergyBuffDetect_dialesize;
            fs<< "EnergyBuffDetect_erodesize"<< Parameter.EnergyBuffDetect_erodesize;
            fs<< "DebugWork"<< Parameter.DebugWork;
            fs<<"lob_yaw_offset"<<Parameter.lob_yaw_offset;
            fs<<"lob_pitch_offset"<<Parameter.lob_pitch_offset;
            fs<< "VideoRecordWork"<< Parameter.VideoRecordWork;
            // Mappings write
            fs << "camera"<< "[";
            for (int i = 0; i < Parameter.cam_parameter.size(); i++) {
                CamParameter &temp_parameter=Parameter.cam_parameter[i];
                fs <<"{"<< "id" << i
                <<"fps"<< temp_parameter.fps
                <<"width"<< temp_parameter.width
                <<"height"<< temp_parameter.height;
                fs<<"exposure_red"<< temp_parameter.red_exposure;
                fs<<"exposure_blue"<< temp_parameter.blue_exposure;
                fs<<"gain"<< temp_parameter.gain;
                fs<<"brightness"<< temp_parameter.brightness;
                fs<<"saturation"<< temp_parameter.saturation;
                fs<<"contrast"<< temp_parameter.contrast;
                fs<<"gamma"<< temp_parameter.gamma;
                fs<<"sharpness"<< temp_parameter.sharpness;
                fs<<"black_level"<< temp_parameter.black_level;

                fs<<"balance_value"<< temp_parameter.balance_value;
                fs<<"balance_red"<< temp_parameter.balance_ratio_red;
                fs<<"balance_green"<< temp_parameter.balance_ratio_green;
                fs<<"balance_blue"<< temp_parameter.balance_ratio_blue;

                fs<<"matrix"<<temp_parameter.camera_matrix;
                fs<<"dist_coeffs"<<temp_parameter.dist_coeffs;
                fs<<"save_res"<<temp_parameter.save_res;
                fs<<"save_runlog"<<temp_parameter.save_runlog;
                fs<<"video_fps"<<temp_parameter.video_fps;
                fs<<"video_path"<<temp_parameter.video_path;
                fs<<"}";
            }
            fs<<"]";
            fs.release();
        }
        if((flock(1, LOCK_UN ))<0)
        {
            std::cout<<"Log::unlock the logfile error"<<std::endl;
        }
    }
}