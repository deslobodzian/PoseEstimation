//
// Created by DSlobodzian on 1/2/2022.
//
#include "yolov5.hpp"
#include "utils.hpp"
#include "Zed.hpp"
#include "pose_estimator.hpp"
#include "particle_filter.hpp"

int main() {
    Zed zed;
//    Yolov5 yoloRT;
    PoseEstimator estimator(2);

    sl::Mat img_sl;
    cv::Mat img_cv;
    sl::ObjectData picture;
	    
    if (!zed.open_camera()) {
        return EXIT_FAILURE;
    }
    if (zed.enable_tracking()) {}
    if (zed.enable_object_detection()) {}
    estimator.init();
    int i = 0;
    while (i < 100) {
        estimator.print_measurements(0);
        i++;
    }

//    std::string engine_name = "yolov5s.engine";

//    yoloRT.initialize_engine(engine_name);

//    while(i <= 100) {
//	img_sl = zed.get_left_image();
//	yoloRT.prepare_inference(img_sl, img_cv);
//
//	yoloRT.run_inference_and_convert_to_zed(img_cv);
//	zed.input_custom_objects(yoloRT.get_custom_obj_data());
//	picture = zed.get_object_from_id(0);
//	std::cout << zed.center_cam_distance_from_object(picture) << std::endl;
//	i++;
//    }

    // destroy objects after use in program
//    yoloRT.kill();
    zed.close();

  
    std::cout << "Cam open and closed" << std::endl;
}

