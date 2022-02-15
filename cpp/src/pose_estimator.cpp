//
// Created by DSlobodzian on 1/28/2022.
//

#include "pose_estimator.hpp"


PoseEstimator::PoseEstimator(int num_monocular_cameras) {
    num_monocular_cameras_ = num_monocular_cameras;
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        camera_config config = camera_config(78, resolution(1920, 1080), 30);
        monocular_cameras_.emplace_back(MonocularCamera(i, config));
        monocular_cameras_.at(i).open_camera();
    }
}

PoseEstimator::PoseEstimator(int num_monocular_cameras, int num_zed_cameras, std::vector<Landmark> landmarks) {
    filter_ = ParticleFilter(landmarks);
    num_monocular_cameras_ = num_monocular_cameras;
    num_zed_cameras_ = num_zed_cameras;
    for (int i = 0; i < num_zed_cameras + num_monocular_cameras; ++i) {
        threads_started_.push_back(false);
    }
    for (int i = 0; i < num_monocular_cameras_; ++i) { camera_config config = camera_config(78, resolution(1920, 1080), 30);
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
    threads_started_.at(camera.get_id()) = true;
    while (true) {
   	    camera.read_frame();
    	cv::Mat image = camera.get_frame();
    	yoloRT.prepare_inference(image);
    	yoloRT.run_inference(image);
    	camera.add_tracked_objects(yoloRT.get_monocular_obj_data());
    }
}

void PoseEstimator::run_inference_zed(Zed& camera) {
    run_zed();
    Yolov5 yoloRT;
    yoloRT.initialize_engine(engine_name_);
    threads_started_.back() = true;
    while (true) {
    	sl::Mat image = camera.get_left_image();
    	cv::Mat temp;
    	yoloRT.prepare_inference(image, temp);
    	yoloRT.run_inference_and_convert_to_zed(temp);
        camera.input_custom_objects(yoloRT.get_custom_obj_data());
    }
}

void PoseEstimator::add_measurements(std::vector<Eigen::Vector3d> &z) {
    for (auto &camera : monocular_cameras_) {
        camera.add_measurements(z);
    }
    zed_.add_measurements(z, 0);
}

void PoseEstimator::estimate_pose(double *u, std::vector<Eigen::Vector3d> z) {
    while (true) {
        auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
        filter_.monte_carlo_localization(u, z);
        std::this_thread::sleep_until(x);
    }
}


void PoseEstimator::init(Eigen::Vector3d init_pose) {
    double x = init_pose(0);
    double y = init_pose(1);
    double theta = init_pose(2);
    info("Initializing filter with pose: [" +
            std::to_string(x) + ", " +
            std::to_string(y) + ", " +
            std::to_string(theta) + "]");
    filter_.init_particle_filter(init_pose);
    info("Starting ZED camera thread.");
    inference_threads_.push_back(
            std::thread(
                &PoseEstimator::run_inference_zed,
                this,
                std::ref(zed_)
            ));
    info("Starting monocular camera threads.");
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        inference_threads_.push_back(
			std::thread(
				&PoseEstimator::run_inference,
			       	this,
			       	std::ref(monocular_cameras_.at(i))
			));
    }
    info("Threads started!");
}

void PoseEstimator::print_measurements(int camera_id) {
    tracked_object obj = monocular_cameras_.at(camera_id).get_object(0);
    double measurement = monocular_cameras_.at(camera_id).yaw_angle_to_object(obj);
    std::string message = "Camera[" + std::to_string(camera_id) + "] has found object: " +
    std::to_string(obj.class_id) + " with angle {" + std::to_string(measurement);
    info(message);
}

void PoseEstimator::print_zed_measurements(int label) {
    std::string message =
            "ZED camera measurement { Object [" + std::to_string(label) +
            "]: Distance [" + std::to_string(zed_.get_distance_to_object_label(label)) +
            "] meters, Angle [" + std::to_string(zed_.get_angle_to_object_label(label)) +
            "] radians}";
    debug(message);
}

Zed& PoseEstimator::get_zed() {
    return zed_;
}

void PoseEstimator::display_frame(int camera_id) {
	monocular_cameras_.at(camera_id).draw_tracked_objects();
	cv::Mat frame = monocular_cameras_.at(camera_id).get_frame();
	std::string id = "Camera " + std::to_string(camera_id);
	if (!frame.empty()) {
		cv::imshow(id, monocular_cameras_.at(camera_id).get_frame());
	} else {
		error("Frame empty in camera [" + std::to_string(camera_id));
	}
}

bool PoseEstimator::threads_started(){
    for (bool i : threads_started_) {
        if (!i) {
            return false;
        }
    }
    return true;
}

void PoseEstimator::kill() {
    for (auto& i : inference_threads_) {
        i.join();
    }
    zed_.close();
}




