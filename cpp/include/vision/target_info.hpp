//
// Created by DSlobodzian on 4/21/2022.
//

#ifndef POSE_ESTIMATION_TARGET_INFO_HPP
#define POSE_ESTIMATION_TARGET_INFO_HPP

#include <sl/Camera.hpp>
#include <Eigen/Dense>
#include "map.hpp"

class TargetInfo {
private:

    game_elements element_;
    double x_;
    double y_;
    double z_;

    double vx_;
    double vy_;
    double vz_;

public:
    TargetInfo();
    TargetInfo(sl::ObjectData object);
    ~TargetInfo();

    const double get_x();
    const double get_y();
    const double get_z();

    const double get_vx();
    const double get_vy();
    const double get_vz();
};

#endif //POSE_ESTIMATION_TARGET_INFO_HPP
