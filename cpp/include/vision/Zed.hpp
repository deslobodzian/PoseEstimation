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

    void close();


    float catapult_phi_angle_to_object(ObjectData& object) {
        float ty = calibration_params_.stereo_transform.ty * 0.5f;
        Eigen::Vector2d a(object.position.x, object.position.y - ty - CAM_TO_CATAPULT_Y);
        Eigen::Vector2d b(1, 0);
	    float angle = 0;
	    if (object.position.y < 0) {
             	angle = -acos(a.dot(b)/(a.norm() * b.norm()));
	    } else {
            	angle = acos(a.dot(b)/(a.norm() * b.norm()));
	    }
        return angle;
    }


    bool has_objects(int label) {
         return !get_objects_from_label(label).empty();
    }


    double get_angle_to_object(int id) {
        ObjectData obj = get_object_from_id(id);
        return catapult_phi_angle_to_object(obj);
    }

    double get_angle_to_object_label(int label) {
         std::vector<ObjectData> tmp = get_objects_from_label(label);
         if (tmp.empty()) {
             return 333;
         } else {
             return get_angle_to_object(tmp.at(0).id);
         }
     }

    void print_object(int label) {
         for (auto object : get_objects_from_label(label)) {
		        std::cout << "Object label {" << object.raw_label << "} with id {" << object.id << "}\n";
         }
    }

    void add_measurements(std::vector<Measurement> &z, game_elements element) {
         for (auto &object : get_objects_from_element(element)) {
             z.push_back(Measurement(get_distance_to_object(object.id), get_angle_to_object(object.id), element));
         }
    }

    Pose get_camera_pose() {
         zed_.getPosition(camera_pose_, sl::REFERENCE_FRAME::CAMERA);
         if (successful_grab()) {
             return camera_pose_;
         }
         return Pose();
    }


    void transform_pose(Transform &pose, float tx, float ty, float tz) {
        Transform tmpTransform;
        tmpTransform.setIdentity();
        tmpTransform.tx = tx;
        tmpTransform.ty = ty;
        tmpTransform.tz = tz;
        pose = Transform::inverse(tmpTransform) * pose * tmpTransform;
    }

    void transform_pose(Transform &pose, Transform transform) {
         pose = Transform::inverse(transform) * pose * transform;
    }

    float get_linear_velocity_x() {
        if (successful_grab()) {
            zed_.getSensorsData(sensors_data_, TIME_REFERENCE::IMAGE);
            imu_data_ = sensors_data_.imu;
            return imu_data_.linear_acceleration.tx;
        }
        return 0;
    }

    float get_linear_velocity_y() {
        if (successful_grab()) {
            zed_.getSensorsData(sensors_data_, TIME_REFERENCE::IMAGE);
            imu_data_ = sensors_data_.imu;
            return imu_data_.linear_acceleration.ty;
        }
        return 0;
    }



    void print_pose(Pose& pose) {
        printf("Translation: x: %.3f, y: %.3f, z: %.3f, timestamp: %llu\r",
               pose.getTranslation().tx, pose.getTranslation().ty, pose.getTranslation().tz, pose.timestamp.getMilliseconds());
    }

};

//
