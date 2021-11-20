//
// Created by li on 19-6-16.
//

#ifndef T_DT2019VISION_LOG_H
#define T_DT2019VISION_LOG_H

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <cstdlib>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/kd.h>
#include <sys/ioctl.h>

#include "log.h"


///
/// \brief 日志文件的类型
///
enum Log_Rank {
    INFO,
    WARNING,
    ERROR,
    FATAL
};

///
/// \brief 初始化日志文件
void initLogger();



/// \brief 日志系统类
///
class Logger {
    friend void initLogger();

public:
    //构造函数
    Logger(Log_Rank log_rank) : log_rank_(log_rank) {};

    ~Logger();
    ///
    /// \brief 写入日志信息之前先写入的源代码文件名, 行号, 函数名
    /// \param log_rank 日志的等级
    /// \param line 日志发生的行号
    /// \param function 日志发生的函数
    static void taken(Log_Rank log_rank,
                      std::string massage,
                      const int line,
                      const std::string& function);

private:
    static char* GetTime();

    static std::ofstream log_file_;                   ///< 信息日子的输出流
    Log_Rank log_rank_;                             ///< 日志的信息的等级
};    /// \brief 写入日志信息之前先写入的源代码文件名, 行号, 函数名



///
/// \brief 根据不同等级进行用不同的输出流进行读写
///
#define OINFO(massage)   \
Logger(INFO).taken(INFO,massage, __LINE__,__FILE__)
#define OWARNING(massage)   \
Logger(WARNING).taken(WARNING,massage, __LINE__,__FILE__)
#define OERROR(massage)   \
Logger(ERROR).taken(ERROR,massage, __LINE__,__FILE__)
#define OFATAL(massage)   \
Logger(FATAL).taken(FATAL,massage, __LINE__,__FILE__)


#endif //T_DT2019VISION_LOG_H
