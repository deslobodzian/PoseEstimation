//
// Created by deslobodzian on 4/25/22.
//

#ifndef POSE_CAMERA_SERVER_HPP
#define POSE_CAMERA_SERVER_HPP

#include <cameraserver/CameraServer.h>
#include "vision/monocular_camera.hpp"
#include <opencv2/opencv.hpp>

class CameraServer {

private:
    cs::MjpegServer server_;
    cs::CvSource cv_source_;
    MonocularCamera camera_;

public:
    CameraServer() = default;
    CameraServer(MonocularCamera camera);
    ~CameraServer() = default;

    void sendFrame(cv::Mat image);
};
#endif //POSE_CAMERA_SERVER_HPP
