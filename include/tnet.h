
#ifndef UNTITLED3_TDTNET_H
#define UNTITLED3_TDTNET_H
//#define USESVM
#define USEMXNET
#ifdef USEMXNET
#include <mxnet/c_predict_api.h>
#include <mxnet-cpp/MxNetCpp.h>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

using namespace std;
using namespace mxnet::cpp;
using namespace cv;

#include <opencv2/opencv.hpp>
class BufferFile {
public :
    string file_path_;
    int length_;
    char* buffer_;

    explicit BufferFile(std::string file_path)
            :file_path_(file_path) {

        std::ifstream ifs(file_path.c_str(), std::ios::in | std::ios::binary);
        if (!ifs) {
            std::cerr << "Can't open the file. Please check " << file_path << ". \n";
            length_ = 0;
            buffer_ = NULL;
            return;
        }

        ifs.seekg(0, std::ios::end);
        length_ = ifs.tellg();
        ifs.seekg(0, std::ios::beg);
        std::cout << file_path.c_str() << " ... "<< length_ << " bytes\n";

        buffer_ = new char[sizeof(char) * length_];
        ifs.read(buffer_, length_);
        ifs.close();
    }

    int GetLength() {
        return length_;
    }
    char* GetBuffer() {
        return buffer_;
    }

    ~BufferFile() {
        if (buffer_) {
            delete[] buffer_;
            buffer_ = NULL;
        }
    }
};

class Tnet {

public:
    Tnet(int idex);
    ~Tnet(){
        MXPredFree(pred_hnd);
    }
    void predict(vector<Mat> &samples,vector<int> &flags);
private:
    int width=28;
    int height=28;
    int channels=1;
    int image_size = width * height * channels;
    PredictorHandle pred_hnd = 0;

};

#endif //USEMXNET

#endif //UNTITLED3_TDTNET_H
