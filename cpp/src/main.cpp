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
    std::vector<Landmark> map;
    Landmark l1(16.459252 - 13.2334, 8.22960 - 0.3683, blue_plate);
    Landmark l2(16.459252 - 13.2334, 8.22960 - 2.85, red_plate);
    Landmark l3(16.45925 / 2.0, 8.22960 / 2.0, blue_plate);
    Landmark l4(13.2334, 0.3683, red_plate);
    Landmark l5(13.2334, 2.85, goal);
    // if c++20 can replace with " Landmark{.x = 3, .y = -2, .id = 3} "
    map.emplace_back(l1);
    map.emplace_back(l2);
    map.emplace_back(l3);
    map.emplace_back(l4);
    map.emplace_back(l5);
    Server server("10.56.87.59", 27002, "10.56.87.2", 27001);
    PoseEstimator estimator(0, 1, map);
//    std::vector<Eigen::Vector3d> z;
    server.start_thread();
    bool exit_init = false;
    while (!exit_init) {
        debug("Looking for init_pose");
        server.receive_frame();
        if (server.received_init_pose()) {
            Eigen::Vector3d init{
                    server.get_init_pose_frame().init_pose[0],
                    server.get_init_pose_frame().init_pose[1],
                    server.get_init_pose_frame().init_pose[2],
            };
            estimator.init(init);
            exit_init = true;
        }
    }
    while (true) {
        if (estimator.threads_started()) {
//            z.clear();
//            estimator.add_measurements(z);
            auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();

	    std::cout << "Object distance is: " << estimator.get_zed().get_distance_to_object_label(0) << "\n";
//            std::cout << "[INFO] Number of measurements is {" << z.size() << "}\n";
  //          output_frame frame(time, 0, 0, 0, 0, estimator.get_zed().get_distance_to_object_label(0), 0);
   //         if (server.send(frame) < 0 ) { std::cout << "[ERROR] Couldn't send frame!"; }
        }
        //std::cout << "Sending data\n";
    }
}

