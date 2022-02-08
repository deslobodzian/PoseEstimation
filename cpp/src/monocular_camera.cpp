//
// Created by DSlobodzian on 1/27/2022.
//
#include "monocular_camera.hpp"

MonocularCamera::MonocularCamera(int device_id, camera_config config) {
    device_id_ = device_id;
    config_ = config;
}
MonocularCamera::~MonocularCamera() {
    cap_.release();
}

bool MonocularCamera::open_camera() {
    //std::string c = 
//	    "v4l2src device=/dev/video" + std::to_string(device_id_) +
//	    " ! video/x-raw, width=" + std::to_string(config_.camera_resolution.width) +
//	    ", height=" + std::to_string(config_.camera_resolution.height) +
//	    ", framerate="+ std::to_string(config_.frames_per_second) + 
//	    "/1 ! videoconvert ! appsink";
    cap_.open(device_id_);
    cap_.set(CAP_PROP_FRAME_WIDTH, config_.camera_resolution.width);
    cap_.set(CAP_PROP_FRAME_HEIGHT, config_.camera_resolution.height);
    cap_.set(CAP_PROP_FPS, config_.frames_per_second);
    return cap_.isOpened();
}

bool MonocularCamera::read_frame() {
    cap_.read(frame_);
    return frame_.empty();
}

Mat MonocularCamera::get_frame() {
    return frame_;
}

double MonocularCamera::yaw_angle_to_object(tracked_object obj) {
    Point object_center = (obj.object.br() + obj.object.tl()) / 2.0;
    Point center(config_.camera_resolution.width / 2.0, config_.camera_resolution.width / 2.0);
    double focal_length = config_.camera_resolution.width / (2.0 * tan(config_.field_of_view.horizontal / 2.0));
    return atan((center - object_center).x / focal_length);
}

double MonocularCamera::pitch_angle_to_object(tracked_object obj) {
    Point object_center = (obj.object.br() + obj.object.tl()) / 2.0;
    Point center(config_.camera_resolution.height / 2.0, config_.camera_resolution.height / 2.0);
    double focal_length = config_.camera_resolution.height / (2.0 * tan(config_.field_of_view.vertical / 2.0));
    return atan((center - object_center).x / focal_length);
}

Eigen::Vector3d MonocularCamera::get_measurement(tracked_object obj) {
    return Eigen::Vector3d{-1, yaw_angle_to_object(obj), obj.class_id};
}

void MonocularCamera::draw_rect(Rect rect) {
    rectangle(frame_, rect, Scalar(0, 255, 255), 1);
}
void MonocularCamera::draw_crosshair(Rect rect) {
    Point object_center = (rect.br() + rect.tl()) / 2.0;
    Point top = object_center + Point(0, 10);
    Point bot = object_center - Point(0, 10);
    Point left = object_center - Point(10, 0);
    Point right = object_center + Point(10, 0);

    line(frame_, top, bot, Scalar(0, 255, 0), 1);
    line(frame_, left, right, Scalar(0, 255, 0), 1);
}

void MonocularCamera::add_tracked_objects(std::vector<tracked_object> objs) {
    objects_ = objs;
}

void MonocularCamera::draw_crosshair(tracked_object obj) {
    draw_crosshair(obj.object);
}

void MonocularCamera::draw_tracked_objects() {
	for (auto& i : objects_) {
		draw_rect(i.object);
		draw_crosshair(i.object);
	}
}

tracked_object MonocularCamera::get_object(int id) {
    if (objects_.empty()) {
	    cv::Rect r(Point(0,0), Point(1,1));
	    tracked_object x(r, 5);
	    return x;
    }

    return objects_.at(id);
}

int MonocularCamera::get_id() {
	return device_id_;
}



