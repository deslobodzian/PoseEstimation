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
#include <Eigen/Dense>
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
    int height;
    int width;
    resolution() = default;
    resolution(int w, int h) {
        height = h;
        width = w;
    }
};

struct camera_config {
    fov field_of_view;
    int frames_per_second;
    resolution camera_resolution;
    camera_config() = default;
    camera_config(double df, resolution res, int fps) {
        double dFov = df * M_PI / 180.0;
        double aspect = hypot(res.width, res.height);
        double hFov = atan(tan(dFov / 2.0) * (res.width / aspect)) * 2;
        double vFov = atan(tan(dFov / 2.0) * (res.height / aspect)) * 2;
        field_of_view = fov(hFov, vFov, false);
        camera_resolution = res;
        frames_per_second = fps;
    }
    camera_config(fov f, resolution res, int fps) {
        field_of_view = f;
        camera_resolution = res;
        frames_per_second = fps;
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
    camera_config config_;
    std::vector<tracked_object> objects_;
    std::mutex *obj_mutex_;

public:
    MonocularCamera() = default;
    MonocularCamera(int device_id, camera_config config);
    ~MonocularCamera();

    bool open_camera();
    bool read_frame();
    int get_id();

    Mat get_frame();
    int coordinate_change(Point p);
    void draw_rect(Rect rect);
    void draw_crosshair(Rect rect);
    void draw_crosshair(tracked_object obj);
    void draw_tracked_objects();

    void add_tracked_objects(std::vector<tracked_object> objs);
    double yaw_angle_to_object(tracked_object &obj);
    double pitch_angle_to_object(tracked_object &obj);
    void add_measurements(std::vector<Eigen::Vector3d> &z);
    bool is_object_in_box(tracked_object &obj, Rect &rect);
    std::vector<tracked_object> get_objects(int class_id);
    tracked_object closest_object_to_camera(int class_id);
    tracked_object closest_object_to_camera(game_elements game_element);
    tracked_object get_object_at_index(int class_id, int index);
};

#endif //PARTICLE_FILTER_MONOCULARCAMERA_HPP
