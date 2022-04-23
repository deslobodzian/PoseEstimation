//
// Created by DSlobodzian on 11/5/2021.
//
#pragma once

#include <sl/Camera.hpp>
#include <thread>
#include "utils.hpp"
#include <chrono>
#include <Eigen/Dense>
#include "map.hpp"
#include "localization/particle_filter.hpp"

#define CAM_TO_ROBOT_X -0.361696
#define CAM_TO_ROBOT_Y -0.00889
#define CAM_TO_CATAPULT_Y -0.1651

using namespace sl;

class Zed {
private:
     Camera zed_;
     Pose camera_pose_;
     sl::Mat image_;

     InitParameters init_params_;
     RuntimeParameters runtime_params_;
     ObjectDetectionParameters detection_params_;
     ObjectDetectionRuntimeParameters objectTracker_params_rt_;

     Objects objects_;
     SensorsData sensors_data_;
     SensorsData::IMUData imu_data_;
     CalibrationParameters calibration_params_;
     Transform cam_to_robot_;

     float left_offset_to_center_;

     bool successful_grab();

public:
    Zed();
    ~Zed();

    bool open_camera();
    bool enable_tracking();
    bool enable_tracking(Eigen::Vector3d init_pose);
    bool enable_object_detection();

    void input_custom_objects(std::vector<sl::CustomBoxObjectData> objects_in);
    void update_objects();

    ObjectData get_object_from_id(int id);
    std::vector<ObjectData> get_objects_from_label(int label);
    std::vector<ObjectData> get_objects_from_element(game_elements element);

    sl::Transform get_calibration_stereo_transform();

    sl::Mat get_left_image();
    sl::Mat get_right_image();

    Pose get_camera_pose();
    void close();


    void print_pose(Pose& pose) {
        printf("Translation: x: %.3f, y: %.3f, z: %.3f, timestamp: %llu\r",
               pose.getTranslation().tx, pose.getTranslation().ty, pose.getTranslation().tz, pose.timestamp.getMilliseconds());
    }

};
