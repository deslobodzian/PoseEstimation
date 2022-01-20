//
// Created by DSlobodzian on 1/4/2022.
//
#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#define DT 0.05
#define ALPHA_ROTATION 0.0002
#define ALPHA_TRANSLATION 0.05
#define NUM_PARTICLES 500
#define RESAMPLE_PARTICLES 250
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

struct Landmark {
    double x;
    double y;
    int id;
};

class ParticleFilter {

private:
    std::vector<Particle> X_; // particle set for filter
    std::vector<Landmark> map_;
    static double random(double min, double max);
    static double sample_triangle_distribution(double b);
    static double zero_mean_gaussian(double x, double sigma);

public:
    ParticleFilter() = default;
    ParticleFilter(std::vector<Landmark> map);
    ~ParticleFilter();
    Eigen::Vector3d sample_motion_model(std::vector<Eigen::Vector3d> u, Eigen::Vector3d x);
    std::vector<Eigen::Vector3d> measurement_model(Eigen::Vector3d x);
    double sample_measurement_model(Eigen::Vector3d feature, Eigen::Vector3d x, Landmark landmark);
    double calculate_weight(std::vector<Eigen::Vector3d> z, Eigen::Vector3d x, double weight, std::vector<Landmark> map);
    std::vector<Particle> monte_carlo_localization(std::vector<Eigen::Vector3d> u, std::vector<Eigen::Vector3d> z);
    std::vector<Particle> low_variance_sampler(std::vector<Particle> X);
    std::vector<Particle> get_particle_set();
    Eigen::Vector3d get_estimated_pose();
};

#endif PARTICLE_FILTER_HPP