//
// Created by DSlobodzian on 1/4/2022.
//
#pragma once

#define DT 0.05
#define ALPHA_ROTATION 0.0002
#define ALPHA_TRANSLATION 0.5
#define NUM_PARTICLES 100
#define RESAMPLE_PARTICLES 50
#define _USE_MATH_DEFINES


#include <vector>
#include <iostream>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include "map.hpp"
#include "utils.hpp"

struct Particle {
    Eigen::Vector3d x;
    double weight;
};

struct ControlInput {
    double dx;
    double dy;
    double d_theta;
};

class Measurement {
private:
    double range_;
    double bearing_;
    game_elements element_;
public:
    Measurement(double range, double bearing, game_elements element) {
        range_ = range;
        bearing_ = bearing;
        element_ = element;
    }

    double get_range() const{
        return range_;
    }

    double get_bearing() const {
        return bearing_;
    }

    double get_element() {
        return element_;
    }

    void print_measurement() {
        debug(
                "Measurement {Range: " + std::to_string(range_) +
                ", Bearing: " + std::to_string(bearing_) +
                ", Element: " + std::to_string(element_));
    }
};

class ParticleFilter {

private:
    std::vector<Particle> X_; // particle set for filter
    std::vector<Landmark> map_;
    Eigen::Vector3d x_est_;
    static double random(double min, double max);
    static double sample_triangle_distribution(double b);
    static double zero_mean_gaussian(double x, double sigma);

public:
    ParticleFilter() = default;
    ParticleFilter(std::vector<Landmark> map);
    ~ParticleFilter();
    void init_particle_filter(Eigen::Vector3d init_pose);
    Eigen::Vector3d sample_motion_model(ControlInput u, Eigen::Vector3d x);
    std::vector<Eigen::Vector3d> measurement_model(Eigen::Vector3d x);
    double sample_measurement_model(Measurement measurement, Eigen::Vector3d x, Landmark landmark);
    double calculate_weight(std::vector<Measurement> z, Eigen::Vector3d x, double weight, std::vector<Landmark> map);
    std::vector<Particle> monte_carlo_localization(ControlInput u, std::vector<Measurement> &z);
    std::vector<Particle> low_variance_sampler(std::vector<Particle> X);
    std::vector<Particle> get_particle_set();
    Eigen::Vector3d get_estimated_pose();
};
