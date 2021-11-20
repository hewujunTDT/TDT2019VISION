# TDT2019VISION
MXNET基本依赖安装
sudo apt-get install upgrade
sudo apt-get install -y build-essential git libatlas-base-dev libopencv-dev
sudo apt-get install libatlas-base-dev



命令行输入：
    git clone --no-checkout https://github.com/apache/mxnet
    cd mxnet
    git checkout 1.4.0 -b 1.4.0
    git submodule update --init
把make文件夹下的config.mk 复制到mxnet下 

将'/home/###自己的目录###/mxnet/config.mk'文件进行如下修改：
    USE_CPP_PACKAGE = 1
    USE_MKLDNN =1

命令行输入：
    make -j4
