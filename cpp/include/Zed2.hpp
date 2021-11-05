//
// Created by DSlobodzian on 11/5/2021.
//
#pragma once

#include <sl/Camera.hpp>;

using namespace sl;

class Zed2 {
private:
     Camera zed_;
     Pose pose_;
     bool has_area_map_ = false;

public:
    Zed2() {
        InitParameters init_params;
        init_params.camera_resolution = RESOLUTION::HD720;
        init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
        init_params.coordinate_units = UNIT::METER;

        PositionalTrackingParameters tracking_params;

        zed_.open(init_params);
        if (has_area_map_) {
            tracking_params.area_file_path = "map.area";
        }
        tracking_params.enable_area_memory = true;
        zed_.enablePositionalTracking(tracking_params);
    }

    Pose getPose() {
        if (zed_.grab() == SUCCESS) {
            auto state = zed_.getPosition(pose_, REFERENCE_FRAME::WORLD);
            return pose_;
        }
        return Pose();
    }

    void printPose(Pose &pose) {
        printf("Translation: x: %.3f, y: %.3f, z: %.3f, timestamp: %llu\r",
               pose.getTranslation().tx, pose.getTranslation().ty, pose.getTranslation().tz, pose.timestamp);
    }
};

