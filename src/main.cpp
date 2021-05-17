#include <sl/Camera.hpp>

#include "GLViewer.hpp"

using namespace std;
using namespace sl;


int main(int argc, char **argv) {

	Camera zed;

	GLViewer viewer;
	viewer.init(argc, argv);

	ERROR_CODE returned_state = zed.open();

	if (returned_state != ERROR_CODE::SUCCESS) {
		cout << "Error " << returned_state << ", exit program.\n";
		return EXIT_FAILURE;
	}

	auto camera_infos = zed.getCameraInformation();
	printf("Hello! This is my serial number: %d\n", camera_infos.serial_number);
	zed.close();
	return EXIT_SUCCESS; 
}
