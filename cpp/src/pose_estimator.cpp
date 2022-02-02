//
// Created by DSlobodzian on 1/28/2022.
//

#include "pose_estimator.hpp"


PoseEstimator::PoseEstimator(int usb_cameras) {
    num_cameras_ = usb_cameras;
    std::string engine_name = "../yolov5s.engine";
    yoloRT_.initialize_engine(engine_name);
    std::cout << "Constructor starting\n";
    for (int i = 0; i < num_cameras_; ++i) {
	std::cout << "i value is: " << i << "\n";
        camera_config config = camera_config(fov(62.2, 48.8), resolution(640, 480), 30);
        monocular_cameras_.emplace_back(MonocularCamera(i, config));
        monocular_cameras_.at(i).open_camera();
    }
    std::cout << "Constructor complete!\n";
}

PoseEstimator::~PoseEstimator(){
    for (auto& i : inference_treads_) {
        i.join();
    }
}

void PoseEstimator::run_inference(MonocularCamera& camera) {
    while (true) {
        camera.read_frame();
        cv::Mat image = camera.get_frame();
        yoloRT_.prepare_inference(image);
        yoloRT_.run_inference(image, camera.get_id());
        camera.add_tracked_objects(yoloRT_.get_monocular_obj_data(camera.get_id()));
	//std::cout << 
	//	"Size[" << camera.get_id() << "] is "  <<
	//       	yoloRT_.get_monocular_obj_data(camera.get_id()).size()
	//       	<< "\n";
    }
}


void PoseEstimator::init() {
    std::cout << "starting init\n";
    for (int i = 0; i < num_cameras_; ++i) {
        inference_treads_.push_back(
			std::thread(
				&PoseEstimator::run_inference,
			       	this,
			       	std::ref(monocular_cameras_.at(i))
			));
    }
    std::cout << "ending init\n";
}

void PoseEstimator::print_measurements(int camera_id) {
    tracked_object obj = monocular_cameras_.at(camera_id).get_object(0);
    double measurement = monocular_cameras_.at(camera_id).yaw_angle_to_object(obj);
    std::cout << "Camera[" << camera_id << "] has found object: " <<
    obj.class_id << " with angle {" << measurement << "}\n";
}

void PoseEstimator::display_frame(int camera_id) {
	monocular_cameras_.at(camera_id).draw_tracked_objects();
	cv::Mat frame = monocular_cameras_.at(camera_id).get_frame();
	std::string id = "Camera " + std::to_string(camera_id);
	if (!frame.empty()) {
		cv::imshow(id, monocular_cameras_.at(camera_id).get_frame());
	} else {
		std::cout << "frame empty\n";
	}
}




