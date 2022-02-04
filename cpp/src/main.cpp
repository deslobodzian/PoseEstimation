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
    Server server("10.56.87.59", 27002, "10.56.87.2", 27001);
    PoseEstimator estimator(0, 1);
    //estimator.init();
//    yoloRT.initialize_engine(engine_name);
//

    while (true) {
        auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        frame frame(time, 0, 0, 0, 0, estimator.get_zed().get_distance_to_object(0), 0);
        server.send(frame);
        server.receive();
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

