// ZED includes

//#include "Zed.hpp"
#include <sl/Camera.hpp>
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

int main(int argc, char** argv) {
    Camera zed;
    // Setup configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL coordinates system

    ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != ERROR_CODE::SUCCESS) {
        std::cout << "ERROR opening" << std::endl;
        return EXIT_FAILURE;
    }
    auto camera_infos = zed.getCameraInformation();


    GLViewer viewer;
    bool error_viewer = viewer.init(argc, argv, camera_infos.camera_configuration.calibration_parameters.left_cam);
    if (error_viewer) {
        viewer.exit();
        zed.close();
        return EXIT_FAILURE;
    }

    Mat image;
    while (viewer.isAvailable()) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, VIEW::LEFT, MEM::GPU);
            viewer.updateImage(image);
        }
    }
    image.free();
    zed.close();

    return 0;
}

