//
// Created by DSlobodzian on 1/28/2022.
//

#include "pose_estimator.hpp"


PoseEstimator::PoseEstimator(int num_monocular_cameras) {
    num_monocular_cameras_ = num_monocular_cameras;
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        // Logitech C920s
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
    // Logitech C920s
    for (int i = 0; i < num_monocular_cameras_; ++i) {
        camera_config config = camera_config(78, resolution(1920, 1080), 30);
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
    info("Monocular Inference started");
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
    info("Zed Inference started");
    while (true) {
    	sl::Mat image = camera.get_left_image();
    	cv::Mat temp;
    	yoloRT.prepare_inference(image, temp);
    	yoloRT.run_inference_and_convert_to_zed(temp);
        camera.input_custom_objects(yoloRT.get_custom_obj_data());
    }
}

void PoseEstimator::update_measurements() {
    for (auto &camera : monocular_cameras_) {
        camera.add_measurements(z_);
    }
    zed_.add_measurements(z_, blue_plate);
    zed_.add_measurements(z_, red_plate);
    zed_.add_measurements(z_, goal);
}

void PoseEstimator::estimate_pose() {
    while (true) {
        auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
        update_measurements();
//        filter_.monte_carlo_localization(server_.get_latest_frame().u, z_);
        std::this_thread::sleep_until(x);
    }
}


void PoseEstimator::init() {
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
    info("Starting UDP Server thread");
    server_.start_thread();
//    info("Waiting for initial pose");
//    bool exit_init = false;
//    while (!exit_init) {
//        // dummy frame so RoboRIO can grab the IP of the Jetson.
//        output_frame frame(0, 0, 0, 0, 0, 0, 0);
//        server_.send(frame);
//        // Wait till server has received the initial pose from the RoboRio.
//        if (server_.received_init_pose()) {
//            init_pose_ = Eigen::Vector3d{
//                    server_.get_init_pose_frame().init_pose[0],
//                    server_.get_init_pose_frame().init_pose[1],
//                    server_.get_init_pose_frame().init_pose[2],
//            };
//            // initialize our estimator with the initial pose.
//            exit_init = true;
//            info("Set initial pose");
//        }
//    }
//    info("Initializing filter with pose: [" +
//         std::to_string(init_pose_(0)) + ", " +
//         std::to_string(init_pose_(1)) + ", " +
//         std::to_string(init_pose_(2)) + "]");
//    filter_.init_particle_filter(init_pose_);
//    bool start_estimator = false;
//    info("Waiting for odometry data");
//    while (!start_estimator) {
//        if (server_.real_data_started()) {
//            // initialize our estimator with the initial pose.
//            pose_estimation_thread_ = std::thread(&PoseEstimator::estimate_pose, this);
//            start_estimator = true;
//            info("Starting estimator");
//        }
//    }
    info("Threads started!");
}

void PoseEstimator::print_measurements(int camera_id) {
    tracked_object obj = monocular_cameras_.at(camera_id).get_object_at_index(0, 0);
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
void PoseEstimator::send_message() {
    zed_.update_objects();
    // Camera ID 0, will be the camera facing the intake.
    double blue_ball_yaw = -99;
    double red_ball_yaw = -99;
    if (num_monocular_cameras_ > 0) {
	info("Number of cameras: " + std::to_string(num_monocular_cameras_));
        tracked_object b_ball = monocular_cameras_.at(0).closest_object_to_camera(blue_ball);
        tracked_object r_ball = monocular_cameras_.at(0).closest_object_to_camera(red_ball);
        // id of 99 is a dummy id, no current object has this object id.
        if (b_ball.class_id != 99) {
            blue_ball_yaw = monocular_cameras_.at(0).yaw_angle_to_object(b_ball);
        }
        if (r_ball.class_id != 99) {
            red_ball_yaw = monocular_cameras_.at(0).yaw_angle_to_object(r_ball);
        }
    }
    info("Closest red ball yaw" + std::to_string(red_ball_yaw));
    info("Closest blue ball yaw" + std::to_string(blue_ball_yaw));
    auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    output_frame frame(
            time,
            0,
            0,
            0,
            zed_.has_objects(1),
            zed_.get_distance_to_object_label(1),
            zed_.get_angle_to_object_label(1),
            zed_.object_x_from_catapult(1),
            zed_.object_y_from_catapult(1),
            zed_.object_z_from_catapult(1),
            zed_.object_vx(1),
            zed_.object_vy(1),
            zed_.object_vz(1),
            blue_ball_yaw,
            red_ball_yaw
    );
    server_.set_data_frame(frame);
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
    pose_estimation_thread_.join();
    zed_.close();
}




