#include <sl/Camera.hpp>

#include "GLViewer.hpp"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace sl;

void parse_args(int argc, char **argv, InitParameters& param);

void print(std::string msg_prefix, sl::ERROR_CODE err_code == sl::ERROR_CODE::SUCCESS, std::string_msg_suffix = "");

int main() {
	Camera zed;

	InitParameters init_parameters;
	init_parameters.depth_mode = DEPTH_MODE::ULTRA;
	init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
	parse_args(argc, argv, init_parameters);

	auto returned_state = zed.open(init_parameters);

	if (returned_state != ERROR_CODE::SUCCESS) { // Quit if an error occurred 
		print("Open Camera", returned_state, "\nExit program.");
		zed.close();
		return EXIT_FAILURE;
	}

	auto camera_infos = zed.getCameraInformation();

	// Point cloud viewer.
	GLViewer viewer;

	// Initialize point cloud viewer
	FusedPointCloud map;
	GLenum errgl = viewer.init(argc, argv, camera_infos.camera_configuration.calibaration_parameters.left_cam, &map, camera_infors.camera_model);
	if (errgl != GLEW_OK) {
		print("Error OpenGL: " + std::string((char*)glewGetErrorString(errgl)));
	}

	// Setup and start positional tracking
	Pose pose;
	POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;
	PositionalTrackingParameters positional_tracking_parameters;
	// will want to change later for field callibration.
	positional_tracking_parameters.enable_area_memory = false;
	returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("Enabling positional tracking failed: ", returned_state);
		zed.close();
		return EXIT_FAILURE;
	}

	// Set spatial mapping parametes
	SpatialMappingParameters = spatial_mapping_parameters;
	// Request a Point Cloud.
	spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
	// Set mapping range, it will set the resolution accordingly (a higher range, a lower resolution)
	spatial_mapping_parameters.set(SpatialMappingParameters::MAPPING_RANGE::LONG);
	// Request partial updates only (only the lastest updated chunks need to be re-draw)
	spatial_mapping_parameters.use_chunk_only = true;
	// Start the spatial mapping
	zed.enableSpatialMapping(spatial_mapping_parameters);

	// Timestamp of the last fused point cloud requested
	chrono::high_resolution_clock::time_point ts_last;

	// Setup runtime parameters
	RuntimeParameters runtime_parameters;
	// Use low depth confidence avoid introuducing noise in the contruction model
	runtime_parameters.confidence_threshold = 50;

	auto resolution = camera_infos.camera_configuration.resolution;

	// Define display resolution and check that it fit at least the image resolution
	Resolution display_resolution(min((int)resolution.width, 720), min((int)resolution.height, 404));

	// Create a Mat to contain the left image and its opencv ref
	Mat image_zed(display_resolution, MAT_TYPE::U8_C4);
	cv::Mat image_zed_ocv(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(MEM::CPU));

	// Start the main loop
	while (viewer.isAvailable()) {
		// Grab a new image
		if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
			// Retreive the left image
			zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, display_resolution);
			// Retreive teh camera pose data.
			tracking_state = zed.getPosition(pose);
			viewer.updatePose(pose, tracking_state);

			if (tracking_state == POSITIONAL_TRACKING_STATE::OK) {
				auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();

				// Ask for the fused point cloud update if 500ms have elapsed since last request.
				if ((duration > 500) && viewer.chunksUpdated()) {
					// Ask for a point cloud refresh
					zed.requestSpatialMapAsync();
					ts_last = chrono::high_resolution_clock::now();
				}

				// If the point cloud is ready to be retrieved
				if (zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS) {
					zed.retreiveSpatialMapAsync(map);
					viewer.updateChunks();
				}
			}
		}

		cv::imshow("ZED View", image_zed_ocv);
		cv::waitKey(15);
	}
	// Save gernerated point cloud
	//map.save("MyFusedPointCloud");
	
	// Free allocated memory before closing the camera
	image_zed.free();
	// Close the ZED
	zed.close();
}

void parse_args(int argc, char** argv, InitParameters& param)
{
	if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
		// SVO input mode
		param.input.setFromSVOFile(argv[1]);
		param.svo_real_time_mode = true;

		cout << "[Sample] Using SVO File input: " << argv[1] << endl;
	}
	else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
		string arg = string(argv[1]);
		unsigned int a, b, c, d, port;
		if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
			// Stream input mode - IP + port
			string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
			param.input.setFromStream(String(ip_adress.c_str()), port);
			cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
		}
		else  if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
			// Stream input mode - IP only
			param.input.setFromStream(String(argv[1]));
			cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
		}
		else if (arg.find("HD2K") != string::npos) {
			param.camera_resolution = RESOLUTION::HD2K;
			cout << "[Sample] Using Camera in resolution HD2K" << endl;
		}
		else if (arg.find("HD1080") != string::npos) {
			param.camera_resolution = RESOLUTION::HD1080;
			cout << "[Sample] Using Camera in resolution HD1080" << endl;
		}
		else if (arg.find("HD720") != string::npos) {
			param.camera_resolution = RESOLUTION::HD720;
			cout << "[Sample] Using Camera in resolution HD720" << endl;
		}
		else if (arg.find("VGA") != string::npos) {
			param.camera_resolution = RESOLUTION::VGA;
			cout << "[Sample] Using Camera in resolution VGA" << endl;
		}
	}
	else {
		// Default
	}
}

void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
	cout << "[Sample]";
	if (err_code != sl::ERROR_CODE::SUCCESS)
		cout << "[Error] ";
	else
		cout << " ";
	cout << msg_prefix << " ";
	if (err_code != sl::ERROR_CODE::SUCCESS) {
		cout << " | " << toString(err_code) << " : ";
		cout << toVerbose(err_code);
	}
	if (!msg_suffix.empty())
		cout << " " << msg_suffix;
	cout << endl;
}
