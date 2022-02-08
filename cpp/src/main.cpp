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
    PoseEstimator estimator(2, 1, map);
    estimator.init();
//    std::vector<Eigen::Vector3d> z;
    server.start_thread();
    while (true) {
        if (estimator.threads_started()) {
//            z.clear();
//            estimator.add_measurements(z);
            auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
//            std::cout << "[INFO] Number of measurements is {" << z.size() << "}\n";
//            output_frame frame(time, 0, 0, 0, 0, estimator.get_zed().get_distance_to_object_label(0), 0);
//            if (server.send(frame) < 0 ) { std::cout << "[ERROR] Couldn't send frame!"; }
        }
        //std::cout << "Sending data\n";
    }
//	img_sl = zed.get_left_image();
//	yoloRT.prepare_inference(img_sl, img_cv);
//
//	yoloRT.run_inference_and_convert_to_zed(img_cv);
//	zed.input_custom_objects(yoloRT.get_custom_obj_data());
//	picture = zed.get_object_from_id(0);
//	std::cout << zed.center_cam_distance_from_object(picture) << std::endl;
//	i++;
//    }
//
//    // destroy objects after use in program
//    yoloRT.kill();
//    zed.close();
//
  
//    std::cout << "Cam open and closed" << std::endl;
}

