#include "main.h"
#include "lemlib/configs.h"
#include "pros/motor_group.hpp"
#include "lemlib/autos.h"
#include <vector>
#include <cstdlib>
#include <iostream>
#include <random>

const int no_particles = 500;

float lat_noise = 1.0f; //inches
float ang_noise = 1.0f; //degrees
float particles[no_particles][4]; //500 particles, each with x, y, theta, weight

// 1. Obtain a random seed from the hardware
std::random_device rd;

// 2. Initialize the standard Mersenne Twister engine with the seed
std::mt19937 gen(rd());

float dsin(float degrees) {
    return sin(degrees * M_PI / 180.0);
}

float dcos(float degrees) {
    return cos(degrees * M_PI / 180.0);
}

double gaussian() {
    // 3. Define the Gaussian distribution parameters
    double mean = 0.0;       // Center of the distribution (μ)
    double stddev = 1.0;     // Spread/Standard deviation (σ)
    
    std::normal_distribution<double> gaussian(mean, stddev);

    double random_num = gaussian(gen);
    return random_num;
}

//call when initializing or resetting position and pass in pose=(0,0,0)

void set_particles(lemlib::Pose pose) {
    for (int i = 0; i < no_particles; i++) {
        particles[i][0] = pose.x + gaussian() * lat_noise; // x
        particles[i][1] = pose.y + gaussian() * lat_noise; // y
        particles[i][2] = pose.theta + gaussian() * ang_noise; // theta
        particles[i][3] = 1.0f / no_particles; // weight
    }
}


//x_target and y_target are relative to the initialized pose(0,0,0)
void predict_models(double x_target, double y_target, double theta_target, lemlib::Pose last_pose) {
    double delta_x = x_target - last_pose.x;
    double delta_y = y_target - last_pose.y;
    
    // if target - last pose is less than -180, add 360 to delta_theta to make it positive

    if (theta_target < -180) {
        theta_target += 360;
    } else if (theta_target > 180) {
        theta_target -= 360;
    }

    for (int i = 0; i < no_particles; i++) { //add noise to each particle based on the movement of the robot
        double noise_theta = theta_target + gaussian() * ang_noise;
        double noise_x = delta_x + gaussian() * lat_noise;
        double noise_y = delta_y + gaussian() * lat_noise;

        double dx = noise_x * dcos(noise_theta) + noise_y * dsin(noise_theta);
        double dy = noise_y * dcos(noise_theta) - noise_x * dsin(noise_theta);
        double dtheta = noise_theta - particles[i][2];

        particles[i][0] += dx; // x
        particles[i][1] += dy; // y
        particles[i][2] += dtheta; // theta
    }
}

