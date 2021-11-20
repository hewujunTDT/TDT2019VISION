#cmake_minimum_required(VERSION 3.12)
#project(T_DT2019Vision)
#include_directories(./include)
#include_directories("./HikVision/ImageProcess")
#LINK_DIRECTORIES("./HikVision/lib64")
#include_directories("/opt/libjpeg-turbo/include")
#LINK_DIRECTORIES("/opt/libjpeg-turbo/lib64")
#set(CMAKE_CXX_STANDARD 14)
#find_package( OpenCV REQUIRED )
#add_executable(T_DT2019Vision src/readparameter.cpp src/main.cpp  src/angleresolver.cpp   src/predict.cpp src/firecommand.cpp src/usart.cpp src/armordetect.cpp  src/macro.cpp src/targetresolver.cpp src/camera.cpp src/energybuff.cpp)
#target_link_libraries(T_DT2019Vision -pthread ${OpenCV_LIBS} libMvCameraControl.so libMVGigEVisionSDK.so libMVRender.so  libMvUsb3vTL.so libturbojpeg.so)


cmake_minimum_required(VERSION 3.5.1)
project(T_DT2019Vision)
set(CMAKE_CXX_STANDARD 14)
find_package( OpenCV 3.4.2 REQUIRED )
include_directories(./include)


#大恒
include_directories("./sdk/genicam/library/CPP/include")
include_directories("./sdk/include")
link_directories("./sdk/lib")
link_directories("./sdk/genicam/bin/Linux64_x64")
#海康
include_directories("./HikVision/ImageProcess")
include_directories("/opt/libjpeg-turbo/include")
link_directories("./HikVision/lib64")
link_directories("/opt/libjpeg-turbo/lib64")
#MXNET 的cmake配置  /home/tdt ... tdt为自己配置mxnet的目录每个人需要自己更改
include_directories( ~/mxnet/include
        ~/mxnet/3rdparty/dmlc-core/include
        ~/mxnet/dmlc-core/include
        ~/mxnet/3rdparty/tvm/nnvm/include
        ~/mxnet/3rdparty/tvm/include
        ~/mxnet/cpp-package/include )
link_directories(~/mxnet/lib)
##################################

add_executable(T_DT2019Vision src/readparameter.cpp src/main.cpp  src/predict_energybuff.cpp src/predict_robot.cpp src/usart.cpp src/armordetect.cpp src/macro.cpp src/macro_tools.cpp src/targetresolver.cpp src/camera.cpp src/energybuff.cpp include/energybuff.h src/tnet.cpp src/debug.cpp include/debug.h include/kalman.h src/lob.cpp include/lob.h src/log.cpp include/log.h)
target_link_libraries(T_DT2019Vision -pthread mxnet  ${OpenCV_LIBS} libX11.so
        libMvCameraControl.so libMVGigEVisionSDK.so libMVRender.so  libMvUsb3vTL.so libturbojpeg.so #海康
        )