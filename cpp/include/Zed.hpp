//
// Created by DSlobodzian on 11/5/2021.
//
#pragma once

#include <sl/Camera.hpp>
#include <thread>
#include <mutex>
#include <chrono>
#include <Eigen/Dense>
#include "map.hpp"

#define CAM_TO_ROBOT_X -0.361696
#define CAM_TO_ROBOT_Y -0.00889
#define CAM_TO_CATAPULT_Y -0.1651

using namespace sl;

class Zed {
private:
     Camera zed_;
     Pose camera_pose_;
     Pose robot_pose_;
     sl::Mat image_;
     InitParameters init_params_;
     RuntimeParameters runtime_params_;
     ObjectDetectionParameters detection_params_;
     ObjectDetectionRuntimeParameters objectTracker_params_rt_;
     Objects objects_;
     bool has_area_map_ = false;
     SensorsData sensors_data_;
     SensorsData::IMUData imu_data_;
     CalibrationParameters calibration_params_;
     Transform cam_to_robot_;

     float left_offset_to_center_;

     bool successful_grab() {
         return (zed_.grab(runtime_params_) == ERROR_CODE::SUCCESS);
     }

public:
    Zed() {
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
    ~Zed(){}

    // Open the zed camera with initial parameters.
    // init_params_ defined in Zed contructor 
    bool open_camera() {
         auto return_state = zed_.open(init_params_);
         calibration_params_ = zed_.getCameraInformation().calibration_parameters;
         return (return_state == ERROR_CODE::SUCCESS);
    }

    bool enable_tracking(Eigen::Vector3d init_pose) {
        PositionalTrackingParameters tracking_params;
        if (!zed_.isOpened()) {
            error("Opening camera failed");
            return false;
        }
        if (has_area_map_) {
           tracking_params.area_file_path = "map.area";
        }
        tracking_params.enable_area_memory = true;
        sl::Transform initial_position;
        initial_position.setTranslation(sl::Translation(init_pose(0), init_pose(1), 0));
        tracking_params.initial_world_transform = initial_position;
        zed_.enablePositionalTracking(tracking_params);
        return true;
    }

    bool enable_object_detection() {
	    if (zed_.enableObjectDetection(detection_params_) != sl::ERROR_CODE::SUCCESS) {
            error("Object Detection Failed");
            return false;
	    }
        return true;
    }


    // Basic euclidean distance equation.
    float robot_distance_to_object(ObjectData& object) {
        float ty = calibration_params_.stereo_transform.ty * 0.5f;
        Transform tmp;
        tmp.setIdentity();
        tmp.ty = ty;
        float x = pow(object.position.x, 2);
        float y = pow(object.position.y - tmp.ty, 2);
        float z = pow(object.position.z, 2);
        return sqrt(x + y + z);
    }

    float object_x_from_catapult(int id) {
        ObjectData obj = get_object_from_id(id);
        return obj.position.x;
    }

    float object_y_from_catapult(int id) {
        ObjectData obj = get_object_from_id(id);
        float ty = calibration_params_.stereo_transform.ty * 0.5f;
        return obj.position.y - ty - CAM_TO_CATAPULT_Y;
    }

    float object_z_from_catapult(int id) {
        ObjectData obj = get_object_from_id(id);
        return obj.position.z;
    }

    float object_vx(int id) {
        ObjectData obj = get_object_from_id(id);
        return obj.velocity.x;
    }
    float object_vy(int id) {
        ObjectData obj = get_object_from_id(id);
        return obj.velocity.y;
    }
    float object_vz(int id) {
        ObjectData obj = get_object_from_id(id);
        return obj.velocity.z;
    }

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

    void input_custom_objects(std::vector<sl::CustomBoxObjectData> objects_in) {
	    zed_.ingestCustomBoxObjects(objects_in);
    }

    void update_objects() {
         zed_.retrieveObjects(objects_, objectTracker_params_rt_);
    }

    ObjectData get_object_from_id(int id) {
         ObjectData tmp;
         objects_.getObjectDataFromId(tmp, id);
         return tmp;
    }

    std::vector<ObjectData> get_objects_from_label(int label) {
         std::vector<ObjectData> tmp;
         for (auto object : objects_.object_list) {
         	if (object.raw_label == label) {
               		tmp.push_back(object);
             	}
         }
         return tmp;
    }

    std::vector<ObjectData> get_objects_from_element(game_elements element) {
         return get_objects_from_label(element);
    }

    bool has_objects(int label) {
         return !get_objects_from_label(label).empty();
    }

    double get_distance_to_object(int id) {
        ObjectData obj = get_object_from_id(id);
        return robot_distance_to_object(obj);
    }

    double get_distance_to_object_label(int label) {
        std::vector<ObjectData> tmp = get_objects_from_label(label);
        if (tmp.empty()) {
            return -1;
        } else {
            return get_distance_to_object(tmp.at(0).id);
        }
    }

    double get_distance_to_object(game_elements element) {
        return get_distance_to_object_label(element);
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

    void add_measurements(std::vector<Eigen::Vector3d> &z, game_elements element) {
         for (auto &object : get_objects_from_element(element)) {
             z.push_back(Eigen::Vector3d{get_distance_to_object(object.id), get_angle_to_object(object.id), element});
         }
    }

    Pose get_camera_pose() {
         zed_.getPosition(camera_pose_, sl::REFERENCE_FRAME::CAMERA);
         if (successful_grab()) {
             return camera_pose_;
         }
         return Pose();
    }

    Pose get_robot_pose() {
         zed_.getPosition(robot_pose_, sl::REFERENCE_FRAME::WORLD);
         transform_pose(robot_pose_.pose_data, cam_to_robot_);
         if (successful_grab()) {
             return robot_pose_;
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

    sl::Mat get_left_image() {
        if (successful_grab()) {
	    sl::Mat im;
            zed_.retrieveImage(im, VIEW::LEFT);
	    return im;
        }
        return sl::Mat();
    }

    sl::Mat get_right_image() {
        if (successful_grab()) {
            sl::Mat im;
            zed_.retrieveImage(im, VIEW::RIGHT, MEM::GPU);
            return im;
        }
        return sl::Mat();
    }

    //default left camera.
//    const sl::CameraParameters get_camera_parameters(std::string side) const {
//        if (side == "right") {
//            return camera_infos_.camera_configuration.calibration_parameters.right_cam;
//        }
//        return camera_infos_.camera_configuration.calibration_parameters.left_cam;
//    }

    void print_pose(Pose& pose) {
        printf("Translation: x: %.3f, y: %.3f, z: %.3f, timestamp: %llu\r",
               pose.getTranslation().tx, pose.getTranslation().ty, pose.getTranslation().tz, pose.timestamp.getMilliseconds());
    }

    void close() {
        zed_.close();
    }
};

//
