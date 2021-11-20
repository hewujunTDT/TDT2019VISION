#include "log.h"
#include <cstdlib>
#include <ctime>
#include <zconf.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define random(x) (rand()%x)

using namespace std;

std::ofstream Logger::log_file_;


void initLogger(){
    srand((int)time(0));
    Logger::log_file_.open("../log/event.log",ios::app);
    Logger::log_file_<<"//////////////////////////////////////////////////"<<
                     Logger::GetTime()<<"Process Initialization!rand="<<random(9999999)<<endl<<
                     "Program Compilation Date:\t"<<__DATE__<<" "<<__TIME__<<endl<<endl<<flush;
}


char* Logger::GetTime() {
    time_t tm;
    time(&tm);
    static char time_string[128];
    ctime_r(&tm,time_string);
    return time_string;
}


void Logger::taken(Log_Rank log_rank,string massage,const int line, const std::string &file) {
    string rank;
    switch (log_rank){
        case 0:
            rank="[INFO] ";
            break;
        case 1:
            rank="[WARNING] ";
            break;
        case 2:
            rank="[ERROR] ";
            break;
        case 3:
            rank="[FATAL] ";
            break;
    }
    if(log_file_.is_open()){
        log_file_<<GetTime()<<
                 rank<<massage<<" ("<<file<<":"<<line<<")"<<endl<<endl<<flush;
    }
    if(log_rank==INFO){
        cout<<rank<<massage<<" ("<<file<<":"<<line<<")"<<endl<<flush;
    }else{
        cerr<<rank<<massage<<" ("<<file<<":"<<line<<")"<<endl<<flush;
    }

}

Logger::~Logger() {
    if (FATAL == log_rank_) {
        log_file_.close();
        abort();
    }
}