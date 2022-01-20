//
// Created by DSlobodzian on 1/6/2022.
//
#include "ParticleFilter.hpp"


ParticleFilter::ParticleFilter(std::vector<Landmark> map) {
    Particle zero;
    Eigen::Vector3d pose;
    zero.x = pose;
    zero.weight = 1.0/NUM_PARTICLES;
    X_.assign(NUM_PARTICLES, zero);
//    pose << 10, 0, 0;
//    zero.x = pose;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        pose << random(9, 11), random(-1, 1), 0;
        X_.at(i).x = pose;
    }

//    zero.weight = 1.0/NUM_PARTICLES;
//    X_.assign(NUM_PARTICLES, zero);
    map_ = map;
}

ParticleFilter::~ParticleFilter() {}

double ParticleFilter::random(double min, double max) {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(min, max);
    return dist(mt);
}

double ParticleFilter::sample_triangle_distribution(double b) {
    return (sqrt(6.0)/2.0) * (random(-b, b) + random(-b, b));
}

double ParticleFilter::zero_mean_gaussian(double x, double sigma) {
    double probability = (1.0 / sqrt(2.0 * M_PI * pow(sigma, 2))) *
                         exp(-pow(x, 2) / (2.0 * pow(sigma, 2)));
    return probability;
}

// for now assume feature is [range, bearing]
double ParticleFilter::sample_measurement_model(Eigen::Vector3d feature, Eigen::Vector3d x, Landmark landmark) {
//    std::cout << "Feature: " << feature << "\n";
//    std::cout << "x: " << x << "\n";
//    std::cout << "Landmark Sample: " << landmark.x << " " << landmark.y << " " << landmark.id << "\n";
    double q = 0;

    if (feature(2, 0) == landmark.id) {
        double range = hypot(landmark.x - x(0,0), landmark.y - x(1, 0));
        double bearing = atan2(
                landmark.y - x(1, 0),
                landmark.x - x(0, 0)
        ) - x(2, 0);
//        std::cout << "Feature is: " << feature(1,0) << "\n";
//        std::cout << "Range is: " << range << "\n";
//        std::cout << "Bearing is: " << bearing << "\n";
//        std::cout << "Guass is: " << zero_mean_gaussian(feature(0,0) - range, 0.01) << "\n";
//        std::cout << "Range Error is: " << feature(0,0) - range << "\n";
//        std::cout << "Bearing Error is: " << feature(1,0)- bearing << "\n";
        q = zero_mean_gaussian(feature(0, 0) - range, 0.1) *
            zero_mean_gaussian(feature(1, 0) - bearing,0.1);
//        std::cout << "Probability is: " << q << "\n";
    }
    return q;
}

Eigen::Vector3d ParticleFilter::sample_motion_model(std::vector<Eigen::Vector3d> u, Eigen::Vector3d x) {
//    std::cout << "Current Pose: " << u.at(0) << "\n";
//    std::cout << "Prev Pose: " << u.at(1) << "\n";

    double dx = u.at(0)(0,0) - u.at(1)(0,0);
//    std::cout << "dx: " << dx << "\n";
    double dy = u.at(0)(1,0) - u.at(1)(1,0);
//    std::cout << "dy: " << dy << "\n";
    double dTheta = u.at(1)(2, 0) - u.at(0)(2,0);
//    std::cout << "dTheta: " << dTheta << "\n";
    double noise_dx = dx + sample_triangle_distribution(abs(dx * ALPHA_TRANSLATION));
    double noise_dy = dy + sample_triangle_distribution(abs(dy * ALPHA_TRANSLATION));
    double noise_dTheta = dTheta + sample_triangle_distribution(abs(dTheta * ALPHA_TRANSLATION));

//    std::cout << noise_del_translation <<"\n";
    double x_prime = x(0,0) + noise_dx;
    double y_prime = x(1,0) + noise_dy;
    double theta_prime = x(2, 0) + noise_dTheta;
    Eigen::Vector3d result;
    result << x_prime, y_prime, theta_prime;
    return result;
}

std::vector<Eigen::Vector3d> ParticleFilter::measurement_model(Eigen::Vector3d x) {
    std::vector<Eigen::Vector3d> z_vec;
//    std::cout << "Real Pose: " << x << "\n";
    for (Landmark landmark : map_) {
        double range = hypot(
                landmark.x - x(0, 0),
                landmark.y - x(1, 0));
        double bearing = atan2(
                landmark.y - x(1, 0),
                landmark.x - x(0, 0)) - x(2, 0);
        Eigen::Vector3d z;
        z << range, bearing, landmark.id;
        z_vec.emplace_back(z);
    }
    return z_vec;
}
double ParticleFilter::calculate_weight(std::vector<Eigen::Vector3d> z, Eigen::Vector3d x, double weight, std::vector<Landmark> map) {
    for (Landmark landmark : map) {
//        std::cout << "Reading pose: " << x.transpose() << "\n";
//        std::cout << "Reading landmark {x: " << landmark.x << ", y: " << landmark.y << ", id: " << landmark.id << "\n";
        for (auto & i : z) {
//            std::cout << "Reading measurement {range: " << i(0,0) << ", bearing: " << i(1, 0) << ", id: " << i(2,0) << "\n";
            if (i(2,0) == landmark.id) {
//                std::cout << "Weight of landmark " << landmark.x << ", " << landmark.y << ", " << landmark.id
//                << ": " << sample_measurement_model(i, x, landmark) << "\n";
                weight = weight * sample_measurement_model(i, x, landmark);
                break;
            }
        }
    }
//    std::cout << "Weight of particle is: " << weight << "\n";
    return weight;
}

std::vector<Particle> ParticleFilter::low_variance_sampler(std::vector<Particle> X) {
    std::vector<Particle> X_bar;
    double r = random(0, 1.0 / NUM_PARTICLES);
    double c = X.at(0).weight;
    int i = 0;
    for (int particle = 0; particle < NUM_PARTICLES; ++particle) {
        double u = r + (double)(particle - 1) * (1.0 / NUM_PARTICLES);
        while (u > c) {
            i = i + 1;
            c = c + X.at(i).weight;
        }
        X_bar.emplace_back(X.at(i));
    }
    return X_bar;
}


std::vector<Particle>
ParticleFilter::monte_carlo_localization(std::vector<Eigen::Vector3d> u,
                                         std::vector<Eigen::Vector3d> z) {
    std::vector<Particle> X_bar;
    double sum = 0;
    for (int particle = 0; particle < NUM_PARTICLES; ++particle) {
        Eigen::Vector3d x = sample_motion_model(u, X_.at(particle).x);
        double weight = calculate_weight(z, x, X_.at(particle).weight, map_);
        sum += weight;
        Particle p;
        p.x = x;
        p.weight = weight;
        X_bar.emplace_back(p);
    }
    Eigen::VectorXd weights(NUM_PARTICLES);
    //Normalize the weights
    for (int i = 0; i < X_.size(); ++i) {
//        std::cout << "original weight is" << X_bar.at(i).weight << "\n";
//        std::cout << "normalized weight is" << X_bar.at(i).weight/sum << "\n";
        X_bar.at(i).weight = X_bar.at(i).weight / sum;
        weights(i,0) = X_bar.at(i).weight;
    }

    double effective_particles = 1.0 / (weights.transpose() * weights)(0,0);
    if (effective_particles < RESAMPLE_PARTICLES) {
        X_ = low_variance_sampler(X_bar);
    } else {
        X_ = X_bar;
    }

    return X_;
}
std::vector<Particle> ParticleFilter::get_particle_set() {
    return X_;
}

Eigen::Vector3d ParticleFilter::get_estimated_pose() {
    Eigen::MatrixXd x_set(3, X_.size());
    Eigen::VectorXd weights(X_.size());
    for (int i = 0; i < X_.size(); ++i) {
//        std::cout << "X is: " << X_.at(i).x.transpose() << "\n";
        x_set.col(i) = X_.at(i).x;
//        std::cout << "Weight is: " << X_.at(i).weight << "\n";
        weights(i) = X_.at(i).weight;
    }
//    std::cout << "Shape of x is: " << x_set.rows() << ", " << x_set.cols() << "\n";
//    std::cout << "Shape of weights is: " << weights.rows() << ", " << weights.cols() << "\n";
    return x_set * weights;
}