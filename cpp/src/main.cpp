//
// Created by DSlobodzian on 1/2/2022.
//
#include "utils.hpp"
#include "networking/udp_server.hpp"
#include "vision/c920s.hpp"
#include "networking/camera_server.hpp"


int main() {

    // Instantiate Systems:
    UDPServer server;
    resolution res(1920, 1080);
    CameraConfig config("/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_EEDAD4AF-video-index0", 72, res, 30);
    MonocularCamera camera(config);
    CameraServer cameraServer(camera);

    // if c++20 can replace with " Landmark{.x = 3, .y = -2, .id = 3} "
//    Map map;
    // estimator(monocular cam, zed cam, landmarks)
//    PoseEstimator estimator(0, 1, map.get_landmarks());
//    std::vector<Eigen::Vector3d> z;
//    estimator.init();

    debug("Starting loop");
    camera.open_camera();
    cv::Mat image;
    while (true) {
            if (!camera.read_frame()) {
                image = camera.get_frame();
                cameraServer.sendFrame(image);
            }
//        server.receive_message();
        // wait until the estimator has started all threads before feeding data to the filter.
//        if (estimator.threads_started()) {
	    //auto start = std::chrono::high_resolution_clock::now();
            //estimator.print_zed_measurements(0);
//            estimator.send_message();
	    //auto stop = std::chrono::high_resolution_clock::now();
	    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
	    //info("Duration: " + std::to_string(duration.count()));
//            z.clear();
//            estimator.add_measurements(z);

//            auto time = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
//        }
    }
}

