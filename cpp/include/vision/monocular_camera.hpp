//
// Created by DSlobodzian on 1/27/2022.
//

#ifndef PARTICLE_FILTER_MONOCULARCAMERA_HPP
#define PARTICLE_FILTER_MONOCULARCAMERA_HPP
#define USE_MATH_DEFINES_


#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "localization/particle_filter.hpp"
#include "map.hpp"
#include "utils.hpp"

using namespace cv;

struct fov {
    double horizontal;
    double vertical;
    double diagonal;
    fov() = default;
    fov(double h, double v) {
        horizontal = h * M_PI / 180.0;
        vertical = v * M_PI / 180.0;
    }
    fov(double h, double v, bool rad) {
        horizontal = h;
        vertical = v;
    }
};

struct resolution {
    unsigned int height;
    unsigned int width;
    resolution() = default;
    resolution(unsigned int w, unsigned int h) {
        height = h;
        width = w;
    }
};

class CameraConfig {
private:
    std::string device_id_;
    fov field_of_view_;
    int frames_per_second_;
    resolution camera_resolution_;
    std::string pipeline_;
public:
    CameraConfig() = default;
    CameraConfig(std::string device, double diagonal_fov, resolution res, int fps) {
        device_id_ = device;
        double d_fov = diagonal_fov * M_PI / 180.0;
        double aspect = hypot(res.width, res.height);
        double h_fov = atan(tan(d_fov / 2.0) * (res.width / aspect)) * 2;
        double v_fov = atan(tan(d_fov / 2.0) * (res.height / aspect)) * 2;
        field_of_view_ = fov(h_fov, v_fov, false);
        camera_resolution_ = res;
        frames_per_second_ = fps;
    }
    CameraConfig(std::string device, fov fov, resolution res, int fps) {
        device_id_ = device;
        field_of_view_ = fov;
        camera_resolution_ = res;
        frames_per_second_ = fps;
    }

    std::string get_device_id() {
        return device_id_;
    }

    fov get_fov() {
        return field_of_view_;
    }

    resolution get_camera_resolution() {
        return camera_resolution_;
    }

    int get_fps() {
        return frames_per_second_;
    }
    std::string get_pipeline() {
        pipeline_ = "v4l2src device=" + device_id_ + " ! video/x-raw(memory::NVMM), format=BGR, width=" + std::to_string(camera_resolution_.width) +
                ", height=" + std::to_string(camera_resolution_.height) + ", framerate=" + std::to_string(frames_per_second_) +
                "/1 ! appsink";
        return pipeline_;
    }
};

struct tracked_object {
    Rect object;
    int class_id;
    tracked_object(const Rect& obj, int id) {
        object = obj;
        class_id = id;
    }
};


class MonocularCamera {
private:
    VideoCapture cap_;
    Mat frame_;
    int device_id_;
    CameraConfig config_;
    std::vector<tracked_object> objects_;
    std::vector<tracked_object> latest_objects_;

public:
    MonocularCamera() = default;
    MonocularCamera(CameraConfig config);
    ~MonocularCamera();

    bool open_camera();
    bool read_frame();
    int get_id();

    Mat get_frame();
    void get_frame(cv::Mat& image);

    void draw_rect(Rect rect);
    void draw_crosshair(Rect rect);
    void draw_crosshair(tracked_object obj);
    void draw_tracked_objects();

    void add_tracked_objects(std::vector<tracked_object> objs);
    double yaw_angle_to_object(tracked_object &obj);
    double pitch_angle_to_object(tracked_object &obj);
    void add_measurements(std::vector<Measurement> &z);
    void update_objects();
    bool is_object_in_box(tracked_object &obj, Rect &rect);
    std::vector<tracked_object> get_objects(int class_id);
    tracked_object closest_object_to_camera(int class_id);
    tracked_object closest_object_to_camera(game_elements game_element);
    tracked_object get_object_at_index(int class_id, int index);
};

#endif //PARTICLE_FILTER_MONOCULARCAMERA_HPP
