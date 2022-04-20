//
// Created by DSlobodzian on 1/2/2022.
//
#include <chrono>
#include "inference/yolov5.hpp"
#include "utils.hpp"
#include "Zed.hpp"
#include "pose_estimator.hpp"
#include "networking/zmq_server.hpp"


int main() {
    // if c++20 can replace with " Landmark{.x = 3, .y = -2, .id = 3} "
//    Map map;
    // estimator(monocular cam, zed cam, landmarks)
//    PoseEstimator estimator(0, 1, map.get_landmarks());
//    std::vector<Eigen::Vector3d> z;
//    estimator.init();
    info("test");
    ZMQServer server;
    info("test1");
    while (true) {
        debug("receiving");
//        server.receive_message();
        // wait until the estimator has started all threads before feeding data to the filter.
//        if (estimator.threads_started()) {
	    //auto start = std::chrono::high_resolution_clock::now();
            //estimator.print_zed_measurements(0);
//            estimator.send_message();
	    //auto stop = std::chrono::high_resolution_clock::now();
	    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
	    //info("Duration: " + std::to_string(duration.count()));
//            z.clear();
//            estimator.add_measurements(z);

//            auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
//        }
    }
}

