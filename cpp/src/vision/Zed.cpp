//
// Created by DSlobodzian on 4/21/2022.
//
#include "vision/Zed.hpp"

Zed::Zed() {
    // Initial Parameters
    init_params_.camera_resolution = RESOLUTION::HD720;
    init_params_.camera_fps = 60;
    init_params_.depth_mode = DEPTH_MODE::ULTRA;
    init_params_.sdk_verbose = true;
    init_params_.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_params_.coordinate_units = UNIT::METER;
    init_params_.depth_maximum_distance = 20;

    runtime_params_.measure3D_reference_frame = REFERENCE_FRAME::CAMERA;
    // Object Detection Parameters
    detection_params_.enable_tracking = true;
    detection_params_.enable_mask_output = false;
    detection_params_.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;

    cam_to_robot_.setIdentity();
    cam_to_robot_.tx = CAM_TO_ROBOT_X;
    cam_to_robot_.ty = CAM_TO_ROBOT_Y;
}

Zed::~Zed() {}

bool Zed::successful_grab() {
    return (zed_.grab(runtime_params_) == ERROR_CODE::SUCCESS);
}

bool Zed::open_camera() {
    auto return_state = zed_.open(init_params_);
    calibration_params_ = zed_.getCameraInformation().camera_configuration.calibration_parameters;
    return (return_state == ERROR_CODE::SUCCESS);
}

bool Zed::enable_tracking() {
    PositionalTrackingParameters tracking_params;
    if (!zed_.isOpened()) {
        error("Opening vision failed");
        return false;
    }
    tracking_params.enable_area_memory = true;
    sl::Transform initial_position;
    zed_.enablePositionalTracking(tracking_params);
    return true;
}

bool Zed::enable_tracking(Eigen::Vector3d init_pose) {
    PositionalTrackingParameters tracking_params;
    if (!zed_.isOpened()) {
        error("Opening vision failed");
        return false;
    }
    tracking_params.enable_area_memory = true;
    sl::Transform initial_position;
    initial_position.setTranslation(sl::Translation(init_pose(0), init_pose(1), 0));
    tracking_params.initial_world_transform = initial_position;
    zed_.enablePositionalTracking(tracking_params);
    return true;
}

bool Zed::enable_object_detection() {
    if (zed_.enableObjectDetection(detection_params_) != sl::ERROR_CODE::SUCCESS) {
        error("Object Detection Failed");
        return false;
    }
    return true;
}

void Zed::input_custom_objects(std::vector<sl::CustomBoxObjectData> objects_in) {
    zed_.ingestCustomBoxObjects(objects_in);
}

void Zed::update_objects() {
    zed_.retrieveObjects(objects_, objectTracker_params_rt_);
}

ObjectData Zed::get_object_from_id(int id) {
    ObjectData tmp;
    objects_.getObjectDataFromId(tmp, id);
    return tmp;
}

std::vector<ObjectData> Zed::get_objects_from_label(int label) {
    std::vector<ObjectData> tmp;
    for (auto object : objects_.object_list) {
        if (object.raw_label == label) {
            tmp.push_back(object);
        }
    }
    return tmp;
}


void Zed::close() {
    zed_.close();
}