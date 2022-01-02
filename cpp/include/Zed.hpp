//
// Created by DSlobodzian on 11/5/2021.
//
#pragma once

#include <sl/Camera.hpp>;

using namespace sl;

class Zed {
private:
     Camera zed_;
     Pose pose_;
     Mat image_;
     InitParameters init_params_;
     bool has_area_map_ = false;
     SensorsData sensors_data_;
     SensorsData::IMUData imu_data_;
     CameraInformation camera_infos_;

     bool successfulGrab() {
         return (zed_.grab() == ERROR_CODE::SUCCESS);
     }

public:
    Zed() {
        init_params_.camera_resolution = RESOLUTION::HD720;
        init_params_.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
        init_params_.coordinate_units = UNIT::METER;
    }
    ~Zed(){}

    void enable() {
        PositionalTrackingParameters tracking_params;
        zed_.open(init_params_);
        if (!zed_.isOpened()) {
            std::cout << "ERROR: opening camera failed" << std::endl;
        }
        camera_infos_ = zed_.getCameraInformation();
        if (true) {
            if (has_area_map_) {
                tracking_params.area_file_path = "map.area";
            }
            tracking_params.enable_area_memory = true;
            zed_.enablePositionalTracking(tracking_params);
        }
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
            zed_.retrieveImage(image_, VIEW::LEFT, MEM::GPU);
        }
        return image_;
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

    void printPose(Pose &pose) {
        printf("Translation: x: %.3f, y: %.3f, z: %.3f, timestamp: %llu\r",
               pose.getTranslation().tx, pose.getTranslation().ty, pose.getTranslation().tz, pose.timestamp.getMilliseconds());
    }

    void close() {
        zed_.close();
    }
};

