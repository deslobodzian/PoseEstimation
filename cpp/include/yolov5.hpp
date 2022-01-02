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
#include "calibrator.h"

#include <sl/Camera.hpp>

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

class Yolov5:

private:


public:
    static const int INPUT_H = Yolo::INPUT_H;
    static const int INPUT_W = Yolo::INPUT_W;
    static const int CLASS_NUM = Yolo::CLASS_NUM;
    static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof (Yolo::Detection) / sizeof (float) + 1; // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
    const char* INPUT_BLOB_NAME = "data";
    const char* OUTPUT_BLOB_NAME = "prob";
    static Logger gLogger;

    static int get_width(int x, float gw, int divisor = 8);
    static int get_depth(int x, float gd)
    CudaEngine* build_engine(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, float& gd, float& gw, std::string& wts_name);
    ICudaEngine* build_engine_p6(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt, float& gd, float& gw, std::string& wts_name);
    void APIToModel(unsigned int maxBatchSize, IHostMemory** modelStream, bool& is_p6, float& gd, float& gw, std::string& wts_name);
    void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize);
    std::vector<sl::uint2> cvt(const cv::Rect &bbox_in);
