//
// Created by DSlobodzian on 1/2/2022.
//
#include <chrono>
#include "yolov5.hpp"
#include "utils.hpp"
#include "Zed.hpp"
#include "pose_estimator.hpp"
#include "udp_server.hpp"
#include "particle_filter.hpp"


int main() {
    // if c++20 can replace with " Landmark{.x = 3, .y = -2, .id = 3} "
    Map map;
    // estimator(monocular cam, zed cam, landmarks)
    PoseEstimator estimator(1, 1, map.get_landmarks());
    std::vector<Eigen::Vector3d> z;
    estimator.init();

    while (true) {
        // wait until the estimator has started all threads before feeding data to the filter.
        if (estimator.threads_started()) {
            //estimator.print_zed_measurements(2);
            estimator.send_message();
//            auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        }
    }
}

