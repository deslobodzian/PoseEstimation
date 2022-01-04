//
// Created by DSlobodzian on 11/5/2021.
//
#pragma once

#include <sl/Camera.hpp>

using namespace sl;

class Zed {
private:
     Camera zed_;
     Pose pose_;
     Mat image_;
     InitParameters init_params_;
     ObjectDetectionParameters detection_params_;
     ObjectDetectionRuntimeParameters objectTracker_params_rt_;
     Objects objects_;
     bool has_area_map_ = false;
     SensorsData sensors_data_;
     SensorsData::IMUData imu_data_;
     CameraInformation camera_infos_;

     bool successfulGrab() {
         return (zed_.grab() == ERROR_CODE::SUCCESS);
     }

public:
    Zed() {
	// Initial Parameters
        init_params_.camera_resolution = RESOLUTION::HD1080;
	init_params_.sdk_verbose = true;
        init_params_.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;
        init_params_.coordinate_units = UNIT::METER;
	// Obeject Detection Parameteres 
	detection_params_.enable_tracking = true;
	detection_params_.enable_mask_output = false;
	detection_params_.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
    }
    ~Zed(){}

    // Opend the zed camera with initial parameters.
    // init_params_ defined in Zed contructor 
    bool openCamera() {
         return (zed_.open(init_params_) == ERROR_CODE::SUCCESS);
    }

    bool enableTracking() {
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
    bool enableObjectDetection() {
	    if (zed_.enableObjectDetection(detection_params_) != sl::ERROR_CODE::SUCCESS) {
		    std::cout << "ERROR: enabling OD failed\n";
	    }
    }

    void inputCustomObjects(std::vector<sl::CustomBoxObjectData> objects_in) {
	    zed_.ingestCustomBoxObjects(objects_in);
    }

    ObjectData getObjectFromId(int id) {
	    ObjectData tmp;
	    zed_.retrieveObjects(objects_, objectTracker_params_rt_);
	    objects_.getObjectDataFromId(tmp, id);
	    return tmp;
    }

    Pose getPose() {
        if (successfulGrab()) {
            auto state = zed_.getPosition(pose_, REFERENCE_FRAME::WORLD);
            return pose_;
        }
        return Pose();
    }

    float getLinearVelocityX() {
        if (successfulGrab()) {
            zed_.getSensorsData(sensors_data_, TIME_REFERENCE::IMAGE);
            imu_data_ = sensors_data_.imu;
            return imu_data_.linear_acceleration.tx;
        }
        return 0;
    }

    float getLinearVelocityY() {
        if (successfulGrab()) {
            zed_.getSensorsData(sensors_data_, TIME_REFERENCE::IMAGE);
            imu_data_ = sensors_data_.imu;
            return imu_data_.linear_acceleration.ty;
        }
        return 0;
    }

    sl::Mat getLeftImage() {
        if (successfulGrab()) {
	    sl::Mat im;
            zed_.retrieveImage(im, VIEW::LEFT);
	    return im;
        }
        return sl::Mat();
    }

    sl::Mat getRightImage() {
        if (successfulGrab()) {
            sl::Mat im;
            zed_.retrieveImage(im, VIEW::RIGHT, MEM::GPU);
            return im;
        }
        return sl::Mat();
    }

    //default left camera.
    const sl::CameraParameters getCameraParameters(std::string side) const {
        if (side == "right") {
            return camera_infos_.camera_configuration.calibration_parameters.right_cam;
        }
        return camera_infos_.camera_configuration.calibration_parameters.left_cam;
    }

    void printPose(Pose pose) {
        printf("Translation: x: %.3f, y: %.3f, z: %.3f, timestamp: %llu\r",
               pose.getTranslation().tx, pose.getTranslation().ty, pose.getTranslation().tz, pose.timestamp.getMilliseconds());
    }

    void close() {
        zed_.close();
    }
};

