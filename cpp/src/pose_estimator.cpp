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
        Yolov5 temp;
        inference_engines_.emplace_back(temp);
    }
    std::string engine_name = "../yolov5s.engine";
    for (auto &engine : inference_engines_) {
        engine.initialize_engine(engine_name);
    }
}

PoseEstimator::PoseEstimator(int num_monocular_cameras, int num_zed_cameras) {
    num_monocular_cameras_ = num_monocular_cameras;
    num_zed_cameras_ = num_zed_cameras;
    std::cout << "Starting monocular cameras.\n";
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        camera_config config = camera_config(fov(62.2, 48.8), resolution(640, 480), 30);
        monocular_cameras_.emplace_back(MonocularCamera(i, config));
        monocular_cameras_.at(i).open_camera();
        Yolov5 temp;
        inference_engines_.emplace_back(temp);
    }
    std::cout << "Finished creating " << monocular_cameras_.size() << " monocular cameras.\n";
    std::cout << "Starting zed cameras.\n";
    for (int i = 0; i < num_zed_cameras_; ++i) {
        zed_cameras_.emplace_back(Zed(i));
        zed_cameras_.at(i).open_camera();
        zed_cameras_.at(i).enable_tracking();
        zed_cameras_.at(i).enable_object_detection();
        Yolov5 temp;
        inference_engines_.emplace_back(temp);
    }
    std::cout << "Finished creating " << zed_cameras_.size() << " zed cameras.\n";

    std::cout << "Starting inference engines.\n";
    std::string engine_name = "../yolov5s.engine";
    for (auto &engine : inference_engines_) {
        engine.initialize_engine(engine_name);
    }
    std::cout << "Inference engines started!\n";
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
        inference_engines_.at(camera.get_id()).prepare_inference(image);
        inference_engines_.at(camera.get_id()).run_inference(image);
        camera.add_tracked_objects(inference_engines_.at(camera.get_id()).get_monocular_obj_data());
	//std::cout << 
	//	"Size[" << camera.get_id() << "] is "  <<
	//       	yoloRT_.get_monocular_obj_data(camera.get_id()).size()
	//       	<< "\n";
    }
}
void PoseEstimator::run_inference_zed(Zed& camera) {
    while (true) {
        sl::Mat image = camera.get_left_image();
        cv::Mat temp;
        inference_engines_.at(0).prepare_inference(image, temp);
        inference_engines_.at(0).run_inference_and_convert_to_zed(temp);
    }
}


void PoseEstimator::init() {
    std::cout << "starting init\n";
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        inference_treads_.push_back(
			std::thread(
				&PoseEstimator::run_inference,
			       	this,
			       	std::ref(monocular_cameras_.at(i))
			));
    }
    for (int i = 0; i < num_zed_cameras_; ++i) {
        std::thread(
                &PoseEstimator::run_inference_zed,
                this,
                std::ref(zed_cameras_.at(i))
        );
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




