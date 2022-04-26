//
// Created by deslobodzian on 4/26/22.
//

#ifndef POSE_C920S_HPP
#define POSE_C920S_HPP

#include "monocular_camera.hpp"

static int RESOLUTION_WIDTHS[] = { 160, 160, 176, 320, 320, 352, 432, 640, 640, 800, 800, 864, 960, 1024, 1280, 1600, 1920 };
static int RESOLUTION_HEIGHTS[] = { 90, 120, 144, 180, 240, 288, 240, 360, 480, 448, 600, 480, 720, 576, 720, 896, 1080 };

enum Resolution {
    RESOLUTION_160x90,
    RESOLUTION_160x120,
    RESOLUTION_172x144,
    RESOLUTION_320x180,
    RESOLUTION_320x240,
    RESOLUTION_352x288,
    RESOLUTION_432x240,
    RESOLUTION_620x360,
    RESOLUTION_640x480,
    RESOLUTION_800x448,
    RESOLUTION_800x600,
    RESOLUTION_864x480,
    RESOLUTION_960x720,
    RESOLUTION_1024x576,
    RESOLUTION_1280x720,
    RESOLUTION_1600x896,
    RESOLUTION_1920x1080
};

class C920s : public MonocularCamera {
private:
    int device_id_;
    CameraConfig config_;
    std::string pipeline_;
    int diagonal_fov_ = 72;

    static void GetCaptureSize(enum Resolution res, unsigned int &width, unsigned int &height) {
        width = RESOLUTION_WIDTHS[res];
        height = RESOLUTION_HEIGHTS[res];
    }
public:
    C920s(int device_id, Resolution c920_resolution, int fps);
    ~C920s();
};

#endif //POSE_C920S_HPP
