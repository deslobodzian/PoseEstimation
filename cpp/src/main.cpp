//
// Created by DSlobodzian on 1/2/2022.
//
#include "yolov5.hpp"
#include "utils.hpp"
#include "Zed.hpp"

int main() {


    std::string engine_name = "yolov5s.engine";
    Zed zed;
    Yolov5 yoloRT;
    std::ifstream file(engine_name,  std::ios::binary);
    if (!file.good()) {
    std::cout << "Error reading engine name!\n";
    return false;
    }
    char *trtModelStream = nullptr;
	size_t size = 0;
	file.seekg(0, file.end);
	size = file.tellg();
	file.seekg(0, file.beg);
	trtModelStream = new char[size];
	assert(trtModelStream);
	file.read(trtModelStream, size);
	file.close();
	std::cout << trtModelStream << std::endl;

    IRuntime *runtime = createInferRuntime(yoloRT.gLogger);
    assert(runtime != nullptr);
    ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    IExecutionContext *context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    void *buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(yoloRT.INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(yoloRT.OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * yoloRT.INPUT_H * yoloRT.INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * yoloRT.OUTPUT_SIZE * sizeof(float)));
    // Create stream
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

//    if (yoloRT.initialize_engine(engine_name)) {
//	    std::cout << "engine init passed!\n";
//    }
    sl::Mat img_sl;
    cv::Mat img_cv;
    sl::ObjectData picture;
	    
    if (!zed.openCamera()) {
        return EXIT_FAILURE;
    }
    if (zed.enableTracking()) {
    }

    if (zed.enableObjectDetection()) {
    }

    int i = 0;

    while(i <= 100) {
	img_sl = zed.getLeftImage();
    	//zed.printPose(zed.getPose());
	yoloRT.prepare_inference(img_sl, img_cv);

	yoloRT.run_inference_and_convert_to_zed(*context, stream, buffers, 0.0f, 1.0f, img_cv);
	zed.inputCustomObjects(yoloRT.get_custom_obj_data());
	picture = zed.getObjectFromId(0);
	std::cout << picture.position << std::endl;
	i++;
    }

    yoloRT.kill();
    zed.close();

  
    std::cout << "Cam open and closed" << std::endl;
}

//int main(int argc, char** argv) {
//
//    Yolov5 yoloRT;
//    sl::ObjectData object;
//    std::string engine_name = "yolov5s.engine";
//    bool is_p6 = false;
//    float gd = 0.0f, gw = 0.0f;
//
//    /// Opening the ZED camera before the model deserialization to avoid cuda context issue
//    sl::Camera zed;
//    sl::InitParameters init_parameters;
//    init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
//    init_parameters.sdk_verbose = true;
//    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
//    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
//
//    // Open the camera
//    auto returned_state = zed.open(init_parameters);
//    if (returned_state != sl::ERROR_CODE::SUCCESS) {
//        //yoloRT.print("Camera Open", returned_state, "Exit program.");
//        return EXIT_FAILURE;
//    }
//    zed.enablePositionalTracking();
//    // Custom OD
//    sl::ObjectDetectionParameters detection_parameters;
//    detection_parameters.enable_tracking = true;
//    detection_parameters.enable_mask_output = false; // designed to give person pixel mask
//    detection_parameters.detection_model = sl::DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
//    returned_state = zed.enableObjectDetection(detection_parameters);
//    if (returned_state != sl::ERROR_CODE::SUCCESS) {
//        //yoloRT.print("enableObjectDetection", returned_state, "\nExit program.");
//        zed.close();
//        return EXIT_FAILURE;
//    }
//    auto camera_config = zed.getCameraInformation().camera_configuration;
//    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720),
//                                 std::min((int) camera_config.resolution.height, 404));
//    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;
//
//    // deserialize the .engine and run inference
//    std::ifstream file(engine_name, std::ios::binary);
//    if (!file.good()) {
//        std::cerr << "read " << engine_name << " error!" << std::endl;
//        return -1;
//    }
//    char *trtModelStream = nullptr;
//    size_t size = 0;
//    file.seekg(0, file.end);
//    size = file.tellg();
//    file.seekg(0, file.beg);
//    trtModelStream = new char[size];
//    assert(trtModelStream);
//    file.read(trtModelStream, size);
//    file.close();
//
//    // prepare input data ---------------------------
//    static float data[BATCH_SIZE * 3 * yoloRT.INPUT_H * yoloRT.INPUT_W];
//    static float prob[BATCH_SIZE * yoloRT.OUTPUT_SIZE];
//    IRuntime *runtime = createInferRuntime(yoloRT.gLogger);
//    assert(runtime != nullptr);
//    ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
//    assert(engine != nullptr);
//    IExecutionContext *context = engine->createExecutionContext();
//    assert(context != nullptr);
//    delete[] trtModelStream;
//    assert(engine->getNbBindings() == 2);
//    void *buffers[2];
//    // In order to bind the buffers, we need to know the names of the input and output tensors.
//    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
//    const int inputIndex = engine->getBindingIndex(yoloRT.INPUT_BLOB_NAME);
//    const int outputIndex = engine->getBindingIndex(yoloRT.OUTPUT_BLOB_NAME);
//    assert(inputIndex == 0);
//    assert(outputIndex == 1);
//    // Create GPU buffers on device
//    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * yoloRT.INPUT_H * yoloRT.INPUT_W * sizeof(float)));
//    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * yoloRT.OUTPUT_SIZE * sizeof(float)));
//    // Create stream
//    cudaStream_t stream;
//    CUDA_CHECK(cudaStreamCreate(&stream));
//
//    assert(BATCH_SIZE == 1); // This sample only support batch 1 for now
//
//    sl::Mat left_sl, point_cloud;
//    cv::Mat left_cv_rgb;
//    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
//    sl::Objects objects;
//    sl::Pose cam_w_pose;
//    cam_w_pose.pose_data.setIdentity();
//
//    while ((char)cv::waitKey(1) != 27) {
//        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
//
//            zed.retrieveImage(left_sl, sl::VIEW::LEFT);
//
//            // Preparing inference
//            cv::Mat left_cv_rgba = slMat2cvMat(left_sl);
//            cv::cvtColor(left_cv_rgba, left_cv_rgb, cv::COLOR_BGRA2BGR);
//            if (left_cv_rgb.empty()) continue;
//            cv::Mat pr_img = preprocess_img(left_cv_rgb, yoloRT.INPUT_W, yoloRT.INPUT_H); // letterbox BGR to RGB
//            int i = 0;
//            int batch = 0;
//            for (int row = 0; row < yoloRT.INPUT_H; ++row) {
//                uchar *uc_pixel = pr_img.data + row * pr_img.step;
//                for (int col = 0; col < yoloRT.INPUT_W; ++col) {
//                    data[batch * 3 * yoloRT.INPUT_H * yoloRT.INPUT_W + i] = (float) uc_pixel[2] / 255.0;
//                    data[batch * 3 * yoloRT.INPUT_H * yoloRT.INPUT_W + i + yoloRT.INPUT_H * yoloRT.INPUT_W] = (float) uc_pixel[1] / 255.0;
//                    data[batch * 3 * yoloRT.INPUT_H * yoloRT.INPUT_W + i + 2 * yoloRT.INPUT_H * yoloRT.INPUT_W] = (float) uc_pixel[0] / 255.0;
//                    uc_pixel += 3;
//                    ++i;
//                }
//            }
//
//            // Running inference
//            yoloRT.doInference(*context, stream, buffers, data, prob, BATCH_SIZE);
//            std::vector<std::vector<Yolo::Detection >> batch_res(BATCH_SIZE);
//            auto &res = batch_res[batch];
//	    int len = sizeof(res)/sizeof(res[0]);
//	    std::cout << "lenght is " << len << std::endl;
//            nms(res, &prob[batch * yoloRT.OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
//
//            // Preparing for ZED SDK ingesting
//            std::vector<sl::CustomBoxObjectData> objects_in;
//            for (auto &it: res) {
//                sl::CustomBoxObjectData tmp;
//                cv::Rect r = get_rect(left_cv_rgb, it.bbox);
//                // Fill the detections into the correct format
//                tmp.unique_object_id = sl::generate_unique_id();
//                tmp.probability = it.conf;
//                tmp.label = (int) it.class_id;
//                tmp.bounding_box_2d = yoloRT.cvt(r);
//                tmp.is_grounded = ((int) it.class_id ==
//                                   0); // Only the first class (person) is grounded, that is moving on the floor plane
//                // others are tracked in full 3D space
//                objects_in.push_back(tmp);
//            }
//            // Send the custom detected boxes to the ZED
//            zed.ingestCustomBoxObjects(objects_in);
//
//
//            // Displaying 'raw' objects
//            for (size_t j = 0; j < res.size(); j++) {
//                cv::Rect r = get_rect(left_cv_rgb, res[j].bbox);
//                cv::rectangle(left_cv_rgb, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
//                cv::putText(left_cv_rgb, std::to_string((int) res[j].class_id), cv::Point(r.x, r.y - 1),
//                            cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
//            }
//            cv::imshow("Objects", left_cv_rgb);
//            cv::waitKey(10);
//
//            // Retrieve the tracked objects, with 2D and 3D attributes
//            zed.retrieveObjects(objects, objectTracker_parameters_rt);
//	    objects.getObjectDataFromId(object, 0);
//	    if (object.tracking_state == sl::OBJECT_TRACKING_STATE::OK) {
//		    std::cout << object.position << std::endl;
//	    }
//        }
//    }
//
//    // Release stream and buffers
//    cudaStreamDestroy(stream);
//    CUDA_CHECK(cudaFree(buffers[inputIndex]));
//    CUDA_CHECK(cudaFree(buffers[outputIndex]));
//    // Destroy the engine
//    context->destroy();
//    engine->destroy();
//    runtime->destroy();
//
//    return 0;
//}
