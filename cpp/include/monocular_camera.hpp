//
// Created by DSlobodzian on 1/27/2022.
//

#ifndef PARTICLE_FILTER_MONOCULARCAMERA_HPP
#define PARTICLE_FILTER_MONOCULARCAMERA_HPP

#include <opencv2/opencv.hpp>

using namespace cv;

struct fov {
    double horizontal;
    double vertical;
    fov() = default;
    fov(double h, double v) {
        horizontal = h * M_PI / 180.0;
        vertical = v * M_PI / 180.0;
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

public:
    MonocularCamera() = default;
    MonocularCamera(int device_id, camera_config config);
    ~MonocularCamera();

    bool open_camera();
    bool read_frame();

    Mat get_frame();
    int coordinate_change(Point p);
    void draw_rect(Rect rect);
    void draw_crosshair(Rect rect);
    void draw_crosshair(tracked_object obj);
    void add_tracked_objects(std::vector<tracked_object> objs);
    double yaw_angle_to_object(tracked_object obj);
    double pitch_angle_to_object(tracked_object obj);
    tracked_object get_object(int id);
};

#endif //PARTICLE_FILTER_MONOCULARCAMERA_HPP
