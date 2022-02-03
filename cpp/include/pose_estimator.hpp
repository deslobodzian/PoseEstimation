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
    int num_monocular_cameras_;
    int num_zed_cameras_;
    std::vector<std::thread> inference_treads_;
    std::vector<MonocularCamera> monocular_cameras_;
    Zed zed_;
    std::vector<Yolov5> inference_engines_;
public:
    PoseEstimator() = default;
    PoseEstimator(int num_monocular_cameras);
    PoseEstimator(int num_monocular_cameras, int num_zed_cameras);
    ~PoseEstimator();

    void run_inference(MonocularCamera& camera);
    void run_inference_zed(Zed& camera);
    void init();

    void print_measurements(int camera_id);
    void display_frame(int camera_id);
};