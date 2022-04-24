//
// Created by DSlobodzian on 4/21/2022.
//
#include "vision/tracked_object_info.hpp"

TrackedObjectInfo::TrackedObjectInfo() {}

TrackedObjectInfo::TrackedObjectInfo(sl::ObjectData data) {
    element_ = (game_elements) data.label;
    x_ = data.position.x;
    y_ = data.position.y;
    z_ = data.position.z;

    vx_ = data.velocity.x;
    vy_ = data.velocity.y;
    vz_ = data.velocity.z;
}

const double TrackedObjectInfo::get_x() {
    return x_;
}

const double TrackedObjectInfo::get_y() {
    return y_;
}

const double TrackedObjectInfo::get_z() {
    return z_;
}

const double TrackedObjectInfo::get_vx() {
    return vx_;
}

const double TrackedObjectInfo::get_vy() {
    return vy_;
}

const double TrackedObjectInfo::get_vz() {
    return vz_;
}

double TrackedObjectInfo::get_distance(double x_offset, double y_offset, double z_offset) {
    double x = pow(get_x() - x_offset, 2);
    double y = pow(get_y() - y_offset, 2);
    double z = pow(get_z() - z_offset, 2);
    return sqrt(x + y + z);
}

double TrackedObjectInfo::get_distance(sl::Transform offset) {
    return get_distance(offset.tx, offset.ty, offset.tz);
}

double TrackedObjectInfo::get_yaw_angle() {
    return atan((sqrt((get_x() * get_x()) + (get_y() + get_y())) / get_z()));
}

double TrackedObjectInfo::get_pitch_angle() {
    return atan((get_y() / get_z()));
}

std::string TrackedObjectInfo::to_packet() {
    return std::to_string((int) element_) + ";" +
            std::to_string(x_) + ";" +
            std::to_string(y_) + ";" +
            std::to_string(z_) + ";" +
            std::to_string(vx_) + ";" +
            std::to_string(vy_) + ";" +
            std::to_string(vz_) + ";";
}




