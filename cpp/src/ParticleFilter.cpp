#include "ParticleFilter.hpp"

ParticleFilter::ParticleFilter(Eigen::Vector3d initial_position) {}

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
double ParticleFilter::sample_measurement_model(Eigen::Vector2d feature, Eigen::Vector3d x, Eigen::Vector2d landmark) {
    double range = sqrt(
            pow(landmark(0,0) - x(0,0), 2) +
            pow(landmark(1,0) - x(1,0), 2));
    double bearing = atan2(
            landmark(1,0) - x(1,0),
            landmark(0,0) - x(0,0)
    );
    double q = zero_mean_gaussian(feature(1,0) - range, 0.01) *
               zero_mean_gaussian(feature(2,0) - bearing, 0.01);
    return q;
}

Eigen::Vector3d ParticleFilter::sample_motion_model(std::vector<Eigen::Vector3d> u, Eigen::Vector3d x) {
    double del_rotation_1 = atan2(
            u.at(1)(1, 0) - u.at(0)(1, 0),
            u.at(1)(0, 0) - u.at(0)(0, 0))
                            - u.at(0)(2,0);
    double del_translation = sqrt(
            pow((u.at(1)(0,0) - u.at(0)(0,0)), 2) +
            pow((u.at(1)(1,0) - u.at(0)(1,0)), 2));
    double del_rotation_2 = u.at(1)(2, 0) - u.at(0)(2, 0) - del_rotation_1;

    double noise_del_rotation_1 = del_rotation_1 + sample_triangle_distribution(
            (ALPHA_ROTATION * abs(del_rotation_1)) +
            (ALPHA_TRANSLATION * abs(del_translation))
    );
    double noise_del_translation = del_translation + sample_triangle_distribution(
            (ALPHA_TRANSLATION * abs(del_translation)) +
            (ALPHA_ROTATION * (abs(del_rotation_1) + abs(del_rotation_2)))
    );
    double noise_del_rotation_2 = del_rotation_1 + sample_triangle_distribution(
            (ALPHA_ROTATION * abs(del_rotation_2)) +
            (ALPHA_TRANSLATION * abs(del_translation))
    );
    double x_prime = x(0,0) + noise_del_translation * cos(x(2,0) + noise_del_rotation_1);
    double y_prime = x(1,0) + noise_del_translation * sin(x(2,0) + noise_del_rotation_1);
    double theta_prime = x(2, 0) + noise_del_rotation_1 + noise_del_rotation_2;
    Eigen::Vector3d result;
    result << x_prime, y_prime, theta_prime;
    return result;
}



Eigen::Vector2d ParticleFilter::measurement_model(Eigen::Vector3d x, Eigen::Vector2d landmark_location) {
    double range = sqrt(
            pow(landmark_location(0, 0) - x(0, 0), 2) +
            pow(landmark_location(1, 0) - x(1, 0), 2));
    double bearing = atan2(
            landmark_location(1, 0) - x(1, 0),
            landmark_location(0, 0) - x(0, 0)) - x(2, 0);
    Eigen::Vector2d z;
    z << range, bearing;
    return z;
}

std::vector<Particle> ParticleFilter::low_variance_sampler(std::vector<Particle> X) {
    std::vector<Particle> X_bar;
    double r = random(0, 1.0 / NUM_PARTICLES);
    double c = X.at(0).weight;
    int i = 1;
    for (int particle = 0; particle < NUM_PARTICLES; ++particle) {
        double u = r + (double)(particle - 1) * 1 / NUM_PARTICLES;
        while (u > c) {
            i = i + 1;
            c = c + X.at(i).weight;
        }
        X_bar.emplace_back(X.at(i));
    }
    return X_bar;
}


std::vector<Particle>
ParticleFilter::monte_carlo_localization(std::vector<Particle> prev_X,
                                         std::vector<Eigen::Vector3d> u,
                                         Eigen::Vector2d z) {
    std::vector<Particle> X_bar;
    double sum = 0;
    for (int particle = 0; particle < NUM_PARTICLES; ++particle) {
        Eigen::Vector3d x = sample_motion_model(u, prev_X.at(particle).x);
        double weight = sample_measurement_model(z, x, map);
        sum += weight;
        Particle p = (Particle){.x = x, .weight = weight};
        X_bar.emplace_back(p);
    }
    Eigen::VectorXd weights(NUM_PARTICLES);
    //Normalize the weights
    for (int i = 0; i < X.capacity(); ++i) {
        X.at(i).weight = X.at(i).weight / sum;
        weights(i,0) = X.at(i).weight;
    }

    double effective_particles = 1.0 / (weights * weights.transpose())(0,0);
    if (effective_particles < RESAMPLE_PARTICLES) {
        X = low_variance_sampler(X);
    }

    return X;
}