//
// Created by deslobodzian on 5/2/22.
//

#ifndef POSE_INFERENCE_MANAGER_HPP
#define POSE_INFERENCE_MANAGER_HPP

#include <vector>
#include <thread>
#include "yolov5.hpp"
#include "vision/Zed.hpp"
#include "vision/monocular_camera.hpp"

class InferenceManger {
private:
    std::vector<std::thread> threads_;
    std::string engine_name_;
public:
    InferenceManger(std::string custom_engine);
    ~InferenceManger();

    void add_inference_thread(Zed& camera);
    void add_inference_thread(MonocularCamera& camera);

    void run_inference_zed(Zed& camera);
    void run_inference(MonocularCamera& camera);
};

#endif //POSE_INFERENCE_MANAGER_HPP
