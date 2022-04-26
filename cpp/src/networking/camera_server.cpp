//
// Created by deslobodzian on 4/25/22.
//
#include "networking/camera_server.hpp"

CameraServer::CameraServer(MonocularCamera camera) {
    cv_source_ = frc::CameraServer::PutVideo("Inference Image", 640, 480);
    frc::CameraServer::AddServer("Why");
    camera_ = camera;
}

void CameraServer::sendFrame(cv::Mat image) {
    cv_source_.PutFrame(image);
}

