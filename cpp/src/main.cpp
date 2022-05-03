//
// Created by DSlobodzian on 1/2/2022.
//
#include "utils.hpp"
#include "networking/udp_server.hpp"
#include "networking/camera_server.hpp"
#include "inference/inference_manager.hpp"


int main() {

    // Instantiate Systems
    UDPServer server;

    // generate camera configs
    resolution res(1920, 1080);
    CameraConfig config("/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_EEDAD4AF-video-index0", 72, res, 30);

    // monocular cameras
    MonocularCamera c920_(config);

    // zed camera;
    Zed zed_;

    // opening zed camera. Do this before zed inference thread.
    zed_.open_camera();
    zed_.enable_tracking();
    zed_.enable_object_detection();

    // inference manager;
    InferenceManager inference_manager("custom.engine");

    // add the threads to the manager.
    inference_manager.add_inference_thread(zed_);
    inference_manager.add_inference_thread(c920_);

    while (true) {

    }
}

