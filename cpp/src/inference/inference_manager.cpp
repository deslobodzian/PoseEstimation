//
// Created by deslobodzian on 5/2/22.
//

#include "inference/inference_manager.hpp"

InferenceManger::InferenceManger(std::string custom_engine) {
    engine_name_ = custom_engine;
}

void InferenceManger::add_inference_thread(Zed& camera) {
    threads_.emplace_back(&InferenceManger::run_inference_zed, this, std::ref(camera));
}

void InferenceManger::add_inference_thread(MonocularCamera& camera) {
    threads_.emplace_back(&InferenceManger::run_inference, this, std::ref(camera));
}

void InferenceManger::run_inference_zed(Zed &camera) {
    Yolov5 yoloRT;
    yoloRT.initialize_engine(engine_name_);
    sl::Mat zedImage;
    cv::Mat temp;
    info("Zed Inference started");
    while (true) {
        camera.get_left_image(zedImage);
        yoloRT.prepare_inference(zedImage, temp);
        yoloRT.run_inference_and_convert_to_zed(temp);
        camera.input_custom_objects(yoloRT.get_custom_obj_data());
    }
}

void InferenceManger::run_inference(MonocularCamera& camera) {
    Yolov5 yoloRT;
    yoloRT.initialize_engine(engine_name_);
    cv::Mat image;
    info("Monocular Inference started");
    while (true) {
        camera.get_frame(image);
        yoloRT.prepare_inference(image);
        yoloRT.run_inference(image);
        camera.add_tracked_objects(yoloRT.get_monocular_obj_data());
    }
}


