//
// Created by deslobodzian on 4/26/22.
//
#include "vision/c920s.hpp"

C920s::C920s(int device_id, Resolution c920_resolution, int fps) {
    unsigned int res_height;
    unsigned int res_width;
    GetCaptureSize(c920_resolution, res_width, res_height);
    resolution res = resolution(res_width, res_height);
    config_ = CameraConfig("", diagonal_fov_, res, fps);
    open_camera();
}

C920s::~C920s() {}

