//
// Created by DSlobodzian on 4/21/2022.
//
#include "vision/target_info.hpp"

TargetInfo::TargetInfo() {}

TargetInfo::TargetInfo(sl::ObjectData data) {
    element_ = (game_elements) data.label;
    x_ = data.position.x;
    y_ = data.position.y;
    z_ = data.position.z;

    vx_ = data.velocity.x;
    vy_ = data.velocity.y;
    vz_ = data.velocity.z;
}

const double TargetInfo::get_x() {
    return x_;
}

const double TargetInfo::get_y() {
    return y_;
}

const double TargetInfo::get_z() {
    return z_;
}

const double TargetInfo::get_vx() {
    return vx_;
}

const double TargetInfo::get_vy() {
    return vy_;
}

const double TargetInfo::get_vz() {
    return vz_;
}




