//
// Created by DSlobodzian on 1/28/2022.
//

#include "pose_estimator.hpp"


PoseEstimator::PoseEstimator(int num_monocular_cameras) {
    num_monocular_cameras_ = num_monocular_cameras;
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        camera_config config = camera_config(fov(62.2, 48.8), resolution(640, 480), 30);
        monocular_cameras_.emplace_back(MonocularCamera(i, config));
        monocular_cameras_.at(i).open_camera();
    }
}

PoseEstimator::PoseEstimator(int num_monocular_cameras, int num_zed_cameras) {
    num_monocular_cameras_ = num_monocular_cameras;
    num_zed_cameras_ = num_zed_cameras;
    for (int i = 0; i < num_monocular_cameras_; ++i) { camera_config config = camera_config(fov(62.2, 48.8), resolution(640, 480), 30);
        monocular_cameras_.emplace_back(MonocularCamera(i, config));
        monocular_cameras_.at(i).open_camera();
    }
}


PoseEstimator::~PoseEstimator(){
    for (auto& i : inference_threads_) {
        i.join();
    }
}

void PoseEstimator::run_zed() {
    zed_.open_camera();
    zed_.enable_tracking();
    zed_.enable_object_detection();
}

void PoseEstimator::run_inference(MonocularCamera& camera) {
    Yolov5 yoloRT;
    yoloRT.initialize_engine(engine_name_);
    while (true) {
   	    camera.read_frame();
    	cv::Mat image = camera.get_frame();
    	yoloRT.prepare_inference(image);
    	yoloRT.run_inference(image);
    	camera.add_tracked_objects(yoloRT.get_monocular_obj_data());
	//std::cout << 
	//	"Size[" << camera.get_id() << "] is "  <<
	//       	yoloRT_.get_monocular_obj_data(camera.get_id()).size()
	//       	<< "\n";
	//       	}
    }
}

void PoseEstimator::run_inference_zed(Zed& camera) {
    run_zed();
    Yolov5 yoloRT;
    yoloRT.initialize_engine(engine_name_);
    while (true) {
    	sl::Mat image = camera.get_left_image();
    	cv::Mat temp;
    	yoloRT.prepare_inference(temp);
    	yoloRT.run_inference_and_convert_to_zed(temp);
        camera.input_custom_objects(yoloRT.get_custom_obj_data());
        print_zed_measurements(0);
    }
}


void PoseEstimator::init() {
    std::cout << "[INFO] Starting ZED Camera thread.\n";
    inference_threads_.push_back(
            std::thread(
                &PoseEstimator::run_inference_zed,
                this,
                std::ref(zed_)
            ));
    std::cout << "[INFO] Starting Monocular Camera threads.\n";
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        inference_threads_.push_back(
			std::thread(
				&PoseEstimator::run_inference,
			       	this,
			       	std::ref(monocular_cameras_.at(i))
			));
    }
    std::cout << "[INFO] Threads Started!\n";
}

void PoseEstimator::print_measurements(int camera_id) {
    tracked_object obj = monocular_cameras_.at(camera_id).get_object(0);
    double measurement = monocular_cameras_.at(camera_id).yaw_angle_to_object(obj);
    std::cout << "Camera[" << camera_id << "] has found object: " <<
    obj.class_id << " with angle {" << measurement << "}\n";
}

void PoseEstimator::print_zed_measurements(int object_id) {
    sl::ObjectData data = zed_.get_object_from_id(object_id);
    std::cout << "[Debug] ZED camera measurement { Object [" <<
        object_id << "], Distance [" <<
        zed_.center_cam_distance_from_object(data) <<
        "] meters.\n";
}

void PoseEstimator::display_frame(int camera_id) {
	monocular_cameras_.at(camera_id).draw_tracked_objects();
	cv::Mat frame = monocular_cameras_.at(camera_id).get_frame();
	std::string id = "Camera " + std::to_string(camera_id);
	if (!frame.empty()) {
		cv::imshow(id, monocular_cameras_.at(camera_id).get_frame());
	} else {
		std::cout << "[ERROR] Frame empty in camera [" << camera_id << "]\n";
	}
}




