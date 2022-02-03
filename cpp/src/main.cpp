//
// Created by DSlobodzian on 1/2/2022.
//
#include "yolov5.hpp"
#include "utils.hpp"
#include "Zed.hpp"
#include "pose_estimator.hpp"
#include "particle_filter.hpp"

int main() {
    PoseEstimator estimator(0, 1);

    sl::Mat img_sl;
    cv::Mat img_cv;
    sl::ObjectData picture;
	
    estimator.init();
    int i = 0;

//    yoloRT.initialize_engine(engine_name);
//
    while(i <= 1000) {
        i++;
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

