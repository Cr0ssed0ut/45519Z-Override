#include "main.h"
#include "lemlib/configs.h"
#include "pros/motor_group.hpp"
#include "lemlib/autos.h"
#include <cmath>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <random>
#include <algorithm>

const int no_particles = 500; //500 monte carlo particles for localization

const float right_dist_offset = 1;
const float left_dist_offset = 1;
const float front_dist_offset = 1;
//these const floats are placeholders for now

float lat_noise = 0.02f; //multiplier for lateral movement noise.
float ang_noise = 0.02f; //multiplier for angular movement noise.

//see https://www.sciencedirect.com/topics/biochemistry-genetics-and-molecular-biology/gaussian-distribution */

float particles[no_particles][4]; //500 particles, each with x, y, theta, weight

// 1. Obtain a random seed from the hardware
std::random_device rd;

// 2. Initialize the standard Mersenne Twister engine with the seed
std::mt19937 gen(rd());


// sin and cos functions that take degrees instead of radians. Lemlib uses degrees.
float dsin(float degrees) {
    return sin(degrees * M_PI / 180.0);
}
float dcos(float degrees) {
    return cos(degrees * M_PI / 180.0);
}
float dsec(float degrees) {
    return 1.0 / dcos(degrees);
}

//random number generator with gaussian distribution, mean=0, stddev=1
double gaussian(float dx, float dy) {
    // 3. Define the Gaussian distribution parameters
    //hypotenuse of the triangle formed by dx and dy, used to scale the standard deviation
    double d = sqrt(dx * dx + dy * dy);
    double mean = 0.0;       // Center of the distribution (μ)
    double stddev = lat_noise * d + 0.001;     // Spread/Standard deviation (σ)
    
    std::normal_distribution<double> gaussian(mean, stddev);

    double random_num = gaussian(gen);
    return random_num;
}

//angular gaussian generator. N(0,1).
double agaussian(float dtheta) {
    // 3. Define the Gaussian distribution parameters
    double mean = 0.0;       // Center of the distribution (μ)
    double stddev = ang_noise * std::abs(dtheta) + 0.001;     // Spread/Standard deviation (σ)
    
    std::normal_distribution<double> gaussian(mean, stddev);

    double random_num = gaussian(gen);
    return random_num;
}

//call when initializing or resetting position and pass in pose=(0,0,0)
//set initial particles to be around the initial pose with some noise. 
//arbitrary dx=1 and dy=1 for initial noise.
void set_particles(lemlib::Pose pose) {
    for (int i = 0; i < no_particles; i++) {
        particles[i][0] = pose.x + gaussian(1,1); // x
        particles[i][1] = pose.y + gaussian(1,1); // y
        particles[i][2] = pose.theta + gaussian(1,1); // theta
        particles[i][3] = 1.0f / no_particles; // weight
    }
}

double wrap(double angle) {
    if (angle < -180) {
        while (angle < -180) {
            angle += 360;
        }
    } else if (angle > 180) {
        while (angle > 180) {
            angle -= 360;
        }
    }
    return angle;
}

//x_target and y_target are relative to the initialized pose(0,0,0).

void predict_models(lemlib::Pose current_pose,
                    lemlib::Pose last_pose) {

    // Measured displacement in global coordinates
    double dx = current_pose.x - last_pose.x;
    double dy = current_pose.y - last_pose.y;

    // Convert global displacement into the robot's local frame.
    // theta = 0 points along +Y; positive theta rotates toward +X.
    double last_theta = last_pose.theta;

    double forward =
        dx * dsin(last_theta) + dy * dcos(last_theta);

    double lateral =
        dx * dcos(last_theta) - dy * dsin(last_theta);

    double dtheta = wrap(current_pose.theta - last_pose.theta);

    for (int i = 0; i < no_particles; i++) {

        // Particle's current heading
        double theta = particles[i][2];

        // Motion-dependent noise.
        // Calibrate these coefficients using actual robot data.
        double noisy_forward =
            forward + gaussian(forward, 0);

        double noisy_lateral =
            lateral + gaussian(lateral, 0);

        double noisy_dtheta =
            dtheta + agaussian(dtheta);

        // Approximate the direction during the movement
        double mid_theta = wrap(theta + noisy_dtheta / 2.0);

        // Transform local movement into global coordinates
        particles[i][0] +=
            noisy_forward * dsin(mid_theta)
            + noisy_lateral * dcos(mid_theta);

        particles[i][1] +=
            noisy_forward * dcos(mid_theta)
            - noisy_lateral * dsin(mid_theta);

        // Update heading and normalize to [-180, 180]
        particles[i][2] = wrap(
            particles[i][2] + noisy_dtheta
        );
    }
}

double square(double x) {
    return x * x;
}

//input the appropriate distance sensor (likely the one facing the wall)
void weight_particles(char sensor) {
    //placeholder
    float offset = 0;
    int confidence = 0;

    double distances[3] = {
        left_dist.get() / 25.4,
        right_dist.get() / 25.4,
        front_dist.get() / 25.4
    };

    std::sort(distances, distances + 3);

    double actual_distance = (distances[0] + distances[1]) / 2.0;
    
    float particle_distance = 0; //placeholder - replace with actual theoretical distance
    double sum_weights = 0;
    

    for (int i = 0; i < no_particles; i++) {
        float theta = 90 - particles[i][2];
        double y=particles[i][1];

        particle_distance = y * dsec(theta) - offset; 
        //see notebook for math proving y * secx + o

        //weight based on gaussian likelihood formula
        float stdev = 0;
        if (actual_distance * 25.4 < 200) {
            stdev = 15 / 25.4;
        } else {
            stdev = 0.05 * actual_distance;
        }

        float weight = std::exp(-square((particle_distance - actual_distance)) 
                                / (2 * square(stdev)));
        particles[i][3] = weight;
    }

    sum_weights = 0;
    for (int i = 0; i < no_particles; i++) {
        sum_weights += particles[i][3];
    }

    for (int i = 0; i < no_particles; i++) {
        particles[i][3] /= sum_weights;
    }

}

void MCLinitialize() {
    chassis.setPose(0, 0, 0); // reset robot position to (0, 0) with heading 0 degrees
    set_particles(chassis.getPose());
}

void MCLMoveToPose(double x, double y, double theta, int timeout, auto sensor) {
    chassis.moveToPose(x, y, theta, 
        timeout,lemlib::MoveToPoseParams{}, 
        true); 

    auto last_pose = chassis.getPose();
    while (chassis.isInMotion()) {
        // wait for the robot to reach the target point
        auto current_pose = chassis.getPose();
        predict_models(current_pose, last_pose);

        weight_particles(sensor);

        pros::delay(20);
        last_pose = current_pose;
    }
}

