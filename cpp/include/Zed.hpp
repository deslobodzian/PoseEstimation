//
// Created by DSlobodzian on 11/5/2021.
//
#pragma once

#include <sl/Camera.hpp>
#include <Eigen/Dense>
#include "map.hpp"

using namespace sl;

class Zed {
private:
     Camera zed_;
     Pose pose_;
     sl::Mat image_;
     InitParameters init_params_;
     RuntimeParameters runtime_params_;
     ObjectDetectionParameters detection_params_;
     ObjectDetectionRuntimeParameters objectTracker_params_rt_;
     Objects objects_;
     bool has_area_map_ = false;
     SensorsData sensors_data_;
     SensorsData::IMUData imu_data_;
     CameraInformation camera_infos_;

     float left_offset_to_center_;

     bool successful_grab() {
         return (zed_.grab(runtime_params_) == ERROR_CODE::SUCCESS);
     }

public:
    Zed() {
	    // Initial Parameters
        init_params_.camera_resolution = RESOLUTION::HD1080;
	    init_params_.sdk_verbose = true;
        init_params_.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
        init_params_.coordinate_units = UNIT::METER;

        runtime_params_.measure3D_reference_frame = REFERENCE_FRAME::CAMERA;
	    // Object Detection Parameters
	    detection_params_.enable_tracking = true;
	    detection_params_.enable_mask_output = false;
	    detection_params_.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    }
    ~Zed(){}

    // Open the zed camera with initial parameters.
    // init_params_ defined in Zed contructor 
    bool open_camera() {
         return (zed_.open(init_params_) == ERROR_CODE::SUCCESS);
    }

    bool enable_tracking() {
        PositionalTrackingParameters tracking_params;
        if (!zed_.isOpened()) {
            std::cout << "ERROR: opening camera failed\n";
            return false;
        }
        camera_infos_ = zed_.getCameraInformation();
        if (has_area_map_) {
           tracking_params.area_file_path = "map.area";
        }
        tracking_params.enable_area_memory = true;
        zed_.enablePositionalTracking(tracking_params);
        return true;
    }

    bool enable_object_detection() {
	    if (zed_.enableObjectDetection(detection_params_) != sl::ERROR_CODE::SUCCESS) {
		    std::cout << "ERROR: enabling OD failed\n";
            return false;
	    }
        return true;
    }

    // Basic euclidean distance equation.
    float center_cam_distance_from_object(ObjectData& object) {
        float x_pose = pose_.getTranslation().tx;
        float y_pose = pose_.getTranslation().ty;
        float z_pose = pose_.getTranslation().tz;
        float x = pow(object.position.x - x_pose, 2);
        float y = pow(object.position.y - y_pose, 2);
        float z = pow(object.position.z - z_pose, 2);
        return sqrt(x + y + z);
    }

    float center_cam_phi_angle_to_object(ObjectData& object) {
        float x_pose = pose_.getTranslation().tx;
        float y_pose = pose_.getTranslation().ty;
        return atan2(object.position.y - y_pose, object.position.x - x_pose);
    }

    void input_custom_objects(std::vector<sl::CustomBoxObjectData> objects_in) {
	    zed_.ingestCustomBoxObjects(objects_in);
    }

    ObjectData get_object_from_id(int id) {
	   ObjectData tmp;
	   zed_.retrieveObjects(objects_, objectTracker_params_rt_);
	   objects_.getObjectDataFromId(tmp, id);
	   return tmp;
    }

    std::vector<ObjectData> get_objects_from_label(int label) {
         std::vector<ObjectData> tmp;
         zed_.retrieveObjects(objects_, objectTracker_params_rt_);
         for (auto object : objects_.object_list) {
             if (object.raw_label == label) {
                 tmp.push_back(object);
             }
         }
         return tmp;
    }
    std::vector<ObjectData> get_objects_from_element(game_elements element) {
         return get_object_from_id(element);
    }

    double get_distance_to_object(int id) {
        ObjectData obj = get_object_from_id(id);
        return center_cam_distance_from_object(obj);
    }

    double get_distance_to_object_label(int label) {
        std::vector<ObjectData> tmp = get_objects_from_label(label);
        if (tmp.empty()) {
            return -1;
        } else {
            return get_distance_to_object(tmp.at(0).id);
        }
    }

    double get_distance_from_object(game_elements element) {
        return get_distance_to_object_label(element);
    }

    double get_angle_to_object(int id) {
        ObjectData obj = get_object_from_id(id);
        return center_cam_phi_angle_to_object(obj);
    }

    void print_object(int label) {
         for (auto object : get_objects_from_label(label)) {
		        std::cout << "Object label {" << object.raw_label << "} with id {" << object.id << "}\n";
         }
    }

    void add_measurements(std::vector<Eigen::Vector3d> &z, int label) {
         for (auto &object : get_objects_from_label(label)) {
             z.push_back(Eigen::Vector3d{get_distance_to_object(object.id), get_angle_to_object(object.id), label});
         }
    }


    Pose get_pose() {
        if (successful_grab()) {
            return pose_;
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
    const sl::CameraParameters get_camera_parameters(std::string side) const {
        if (side == "right") {
            return camera_infos_.camera_configuration.calibration_parameters.right_cam;
        }
        return camera_infos_.camera_configuration.calibration_parameters.left_cam;
    }

    void print_pose(Pose& pose) {
        printf("Translation: x: %.3f, y: %.3f, z: %.3f, timestamp: %llu\r",
               pose.getTranslation().tx, pose.getTranslation().ty, pose.getTranslation().tz, pose.timestamp.getMilliseconds());
    }


    void close() {
        zed_.close();
    }
};

