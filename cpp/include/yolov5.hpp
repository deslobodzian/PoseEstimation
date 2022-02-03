//
// Created by DSlobodzian on 1/2/2022.
//
#pragma once

#include <iostream>
#include <chrono>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "utils.hpp"
#include "calibrator.h"
#include <mutex>

#include "monocular_camera.hpp"
#include <sl/Camera.hpp>

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

class Yolov5 {

private:
    ICudaEngine* engine_;
    IRuntime* runtime_;
    IExecutionContext* context_;
    cudaStream_t stream_;
    void* buffers_[2];
    int inputIndex_;
    int outputIndex_;
    int batch_ = 0;
    std::vector<sl::CustomBoxObjectData> objects_in_;
    std::vector<tracked_object> monocular_objects_in_;


public:
    static const int INPUT_H = Yolo::INPUT_H;
    static const int INPUT_W = Yolo::INPUT_W;
    static const int CLASS_NUM = Yolo::CLASS_NUM; 
    // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
    static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;    

    float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    float prob[BATCH_SIZE * OUTPUT_SIZE];

    const char *INPUT_BLOB_NAME = "data";
    const char *OUTPUT_BLOB_NAME = "prob";

    Yolov5() = default;
    ~Yolov5();
    
    Logger gLogger;

    int get_width(int x, float gw, int divisor = 8);

    int get_depth(int x, float gd);

    void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *input, float *output, int batchSize);

    std::vector<sl::uint2> cvt(const cv::Rect &bbox_in);
    void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix);
    bool parse_args(int argc, char** argv, std::string& wts, std::string& engine, bool& is_p6, float& gd, float& gw);
    bool initialize_engine(std::string& engine_engine);
    bool prepare_inference(sl::Mat img_sl, cv::Mat& img_cv_rgb);
    bool prepare_inference(cv::Mat& img_cv_rgb);
    void run_inference_and_convert_to_zed(cv::Mat& img_cv_rgb);
    void run_inference(cv::Mat& img_cv_rgb);
    template <typename T>
    void convert_for_zed_sdk(T& res, cv::Mat& img_cv_rgb);
    std::vector<sl::CustomBoxObjectData> get_custom_obj_data();
    std::vector<tracked_object> get_monocular_obj_data();

    void kill();

};

