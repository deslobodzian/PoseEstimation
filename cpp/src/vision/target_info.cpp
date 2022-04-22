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

double TargetInfo::get_distance(double x_offset, double y_offset, double z_offset) {
    double x = pow(get_x() - x_offset, 2);
    double y = pow(get_y() - y_offset, 2);
    double z = pow(get_z() - z_offset, 2);
    return sqrt(x + y + z);
}

double TargetInfo::get_distance(sl::Transform offset) {
    return get_distance(offset.tx, offset.ty, offset.tz);
}

double TargetInfo::get_yaw_angle() {
    return atan((sqrt((get_x() * get_x()) + (get_y() + get_y())) / get_z()));
}

double TargetInfo::get_pitch_angle() {
    return atan((get_y() / get_z()));
}




