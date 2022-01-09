//
// Created by DSlobodzian on 1/4/2022.
//
#pragma once

#define DT 0.05
#define ALPHA_ROTATION 0.002
#define ALPHA_TRANSLATION 0.05
#define NUM_PARTICLES 1000
#define RESAMPLE_PARTICLES 500
#define _USE_MATH_DEFINES


#include <vector>
#include <iostream>
#include <random>
#include <cmath>
#include <Eigen/Dense>


struct Particle {
    Eigen::Vector3d x;
    double weight;
};

class ParticleFilter {

private:
    std::vector<Particle> X; // particle set for filter

    static Eigen::Vector2d map;
    static double random(double min, double max);
    static double sample_triangle_distribution(double b);
    static double zero_mean_gaussian(double x, double sigma);

public:
    ParticleFilter() = default;
    ParticleFilter(Eigen::Vector3d initial_position);
    ~ParticleFilter();
    Eigen::Vector3d sample_motion_model(std::vector<Eigen::Vector3d> u, Eigen::Vector3d x);
    Eigen::Vector2d measurement_model(Eigen::Vector3d x, Eigen::Vector2d landmark_location);
    double sample_measurement_model(Eigen::Vector2d feature, Eigen::Vector3d x, Eigen::Vector2d landmark);
    std::vector<Particle> monte_carlo_localization(std::vector<Particle> prev_X, std::vector<Eigen::Vector3d> u, Eigen::Vector2d z);
    std::vector<Particle> low_variance_sampler(std::vector<Particle> X);
};
