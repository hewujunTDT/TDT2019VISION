// Created by mark2 on 19-3-31.
//

#include "tnet.h"
#ifdef USEMXNET

Tnet::Tnet(int idex) {
    std::string json_file;
    std::string param_file;
    // Models path for your model, you have to modify it
    if(idex==0){
        json_file = "../model/TDT_Armor-4.0.json";
        param_file ="../model/TDT_Armor-4.0.params";
    } else{
        json_file = "../model/Tnet33.json";
        param_file ="../model/Tnet33.params";
    }

    BufferFile json_data(json_file);
    BufferFile param_data(param_file);

    // Parameters
    int dev_type = 1;  // 1: cpu, 2: gpu
    int dev_id = 0;  // arbitrary.
    mx_uint num_input_nodes = 1;  // 1 for feedforward
    const char* input_key[1] = {"data"};
    const char** input_keys = input_key;

    // Image size and channels

    const mx_uint input_shape_indptr[2] = { 0, 4 };
    const mx_uint input_shape_data[4] = { 4,
                                          static_cast<mx_uint>(channels),
                                          static_cast<mx_uint>(height),
                                          static_cast<mx_uint>(width)};

    if (json_data.GetLength() == 0 ||
        param_data.GetLength() == 0) {;
    }

    // Create Predictor
    assert(0==MXPredCreate((const char*)json_data.GetBuffer(),
                           (const char*)param_data.GetBuffer(),
                           static_cast<size_t>(param_data.GetLength()),
                           dev_type,
                           dev_id,
                           num_input_nodes,
                           input_keys,
                           input_shape_indptr,
                           input_shape_data,
                           &pred_hnd
    ));
    assert(pred_hnd);


}

void Tnet::predict(vector<Mat> &samples, vector<int> &flags) {
    flags.clear();
    static Mat tmpzero =Mat::zeros(Size(28,28),CV_8U);
    std::vector<float> alldata;

    for(size_t i=0;i<samples.size();i++){
        if(i%4==0){
            vector<mx_float > image_data;
            for(size_t j=0;j<4;j++){
                vector<mx_float > tmp;
                if(i+j>=samples.size()) {
                    tmp = (vector<mx_float>) (tmpzero.reshape(1, 1));

                } else{
                    tmp = (vector<mx_float>) (samples[i+j].reshape(1, 1));

                }
                image_data.insert(image_data.end(),tmp.begin(),tmp.end());
                tmp.clear();
            }


            // Set Input Image
            MXPredSetInput(pred_hnd, "data", image_data.data(), image_size*4);
            image_data.clear();
            // Do Predict Forward
            MXPredForward(pred_hnd);
            mx_uint output_index = 0;

            mx_uint *shape = 0;
            size_t size = 1;
            mx_uint shape_len;
            // Get Output Result
            MXPredGetOutputShape(pred_hnd, output_index, &shape, &shape_len);
            for (mx_uint i = 0; i < shape_len; ++i) size *= shape[i];

            std::vector<float> data(size);
            MXPredGetOutput(pred_hnd, 0, &(data[0]), size);
            alldata.insert(alldata.end(),data.begin(),data.end());

        }
    }
    for(size_t i=0;i<samples.size()*9;i++){
        if(i%9==0){
            int result=0;
            float best_accuracy = 0.0;

            for(int j=0;j<9;j++){

                if ( alldata[i+j] > best_accuracy ) {
                    best_accuracy = alldata[i+j];
                    result = j;
                }
            }

            flags.push_back(result);
        }
    }
    alldata.clear();
}
#endif //USEMXNET
