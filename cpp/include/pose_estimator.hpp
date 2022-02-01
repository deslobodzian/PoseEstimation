//
// Created by DSlobodzian on 1/27/2022.
//

#pragma once

#include <iostream>
#include <thread>
#include "yolov5.hpp"
#include "particle_filter.hpp"
#include "Zed.hpp"
#include "monocular_camera.hpp"

class PoseEstimator {
private:
    int num_cameras_;
    std::vector<std::thread> inference_treads_;
    std::vector<MonocularCamera> monocular_cameras_;
    Yolov5 yoloRT_;
public:
    PoseEstimator() = default;
    PoseEstimator(int usb_cameras);
    ~PoseEstimator();

    void run_inference(MonocularCamera& camera);
    void init();

    void print_measurements(int camera_id);
};
