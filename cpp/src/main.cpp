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
    Landmark l1{};
    l1.x = 16.459252 - 13.2334, l1.y = 8.22960 - 0.3683, l1.id = 0;
    Landmark l2{};
    l2.x = 16.459252 - 13.2334, l2.y = 8.22960 - 2.85, l2.id = 1;
    Landmark l3{};
    l3.x = 16.45925 / 2.0, l3.y = 8.22960 / 2.0, l3.id = 2;
    Landmark l4{};
    l4.x = 13.2334, l4.y = 0.3683, l4.id = 3;
    Landmark l5{};
    l5.x = 13.2334, l5.y = 2.85, l5.id = 4;
    // if c++20 can replace with " Landmark{.x = 3, .y = -2, .id = 3} "
    map.emplace_back(l1);
    map.emplace_back(l2);
    map.emplace_back(l3);
    map.emplace_back(l4);
    map.emplace_back(l5);
    Server server("10.56.87.59", 27002, "10.56.87.2", 27001);
    PoseEstimator estimator(0, 1, map);
    estimator.init();
    server.start_thread();
    while (true) {
        if (estimator.threads_started()) {
            auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            output_frame frame(time, 0, 0, 0, 0, estimator.get_zed().get_distance_to_object(0), 0);
            if (server.send(frame) < 0 ) { std::cout << "[ERROR] Couldn't send frame!"; }
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

