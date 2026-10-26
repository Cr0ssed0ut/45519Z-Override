#include "main.h"
#include "lemlib/configs.h"
#include "lemlib/autos.h"
#include <cmath>
#include <cstdlib>
#include <random>
#include <algorithm>

const int no_particles = 500; //500 monte carlo particles for localization

const float right_dist_offset = 1;
const float left_dist_offset = 1;
const float front_dist_offset = 1;
const float back_dist_offset = 1;
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
void set_particles(lemlib::Pose pose, int n = no_particles) {
    for (int i = 0; i < n; i++) {
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

static float stdev = 15/25.4;

// Distance from (x,y) along heading 'deg' (0 = +Y, clockwise +) to the 144x144 field walls (±72)
double ray_to_wall(double x, double y, double deg) {
    double ux = dsin(deg), uy = dcos(deg);
    double t = 1e9;
    if (ux >  1e-6) t = std::min(t, ( 72 - x) / ux);
    if (ux < -1e-6) t = std::min(t, (-72 - x) / ux);
    if (uy >  1e-6) t = std::min(t, ( 72 - y) / uy);
    if (uy < -1e-6) t = std::min(t, (-72 - y) / uy);
    return t;
}

void weight_particles() {
    // order: front, right, back, left -> heading offsets 0, 90, 180, 270
    const double offsets[4] = {0, 90, 180, 270};
    const double sensor_off[4] = {front_dist_offset, right_dist_offset, back_dist_offset,left_dist_offset}; // inches from center
    double meas[4] = {
        front_dist.get() / 25.4, right_dist.get() / 25.4,
        back_dist.get()  / 25.4, left_dist.get()  / 25.4
    };
    bool valid[4];
    for (int j = 0; j < 4; j++) valid[j] = (meas[j] > 0 && meas[j] < 72);

    double sum = 0;
    for (int i = 0; i < no_particles; i++) {
        double x = particles[i][0], y = particles[i][1], th = particles[i][2];
        double w = particles[i][3];
        for (int j = 0; j < 4; j++) {
            if (!valid[j]) continue;
            double expected = ray_to_wall(x, y, th + offsets[j]) - sensor_off[j];
            double err = expected - meas[j];
            w *= std::exp(-err * err / (2 * stdev * stdev));
        }
        particles[i][3] = w;
        sum += w;
    }

    if (sum < 1e-12) {                       // filter lost track: reset to uniform
        for (int i = 0; i < no_particles; i++) particles[i][3] = 1.0f / no_particles;
        return;
    }
    for (int i = 0; i < no_particles; i++) particles[i][3] /= sum;
}

void resample() {
    // Temporary storage for the new particle set (static keeps it off the stack)
    static float new_particles[no_particles][4];

    const double spacing = 1.0 / no_particles;   // gap between sample points on the "weight line"

    // Pick one random starting point within the first gap.
    // All other sample points are evenly spaced from it.
    std::uniform_real_distribution<double> start_dist(0.0, spacing);
    double start_offset = start_dist(gen);

    // Walk along the cumulative weights of the old particles
    double cumulative_weight = particles[0][3];
    int source_index = 0;                         // which old particle we're currently on

    for (int new_index = 0; new_index < no_particles; new_index++) {
        double sample_point = start_offset + new_index * spacing;

        // Advance to the old particle whose weight "covers" this sample point
        while (sample_point > cumulative_weight && source_index < no_particles - 1) {
            source_index++;
            cumulative_weight += particles[source_index][3];
        }

        // Copy that old particle into the new set, with equal weight
        new_particles[new_index][0] = particles[source_index][0];  // x
        new_particles[new_index][1] = particles[source_index][1];  // y
        new_particles[new_index][2] = particles[source_index][2];  // theta
        new_particles[new_index][3] = spacing;                     // weight
    }

    memcpy(particles, new_particles, sizeof(particles));
}

lemlib::Pose estimate() {
    double x = 0, y = 0, s = 0, c = 0;
    for (int i = 0; i < no_particles; i++) {
        double w = particles[i][3];
        x += w * particles[i][0];
        y += w * particles[i][1];
        s += w * dsin(particles[i][2]);
        c += w * dcos(particles[i][2]);
    }
    return lemlib::Pose(x, y, atan2(s, c) * 180.0 / M_PI);
}

void MCLinitialize(float x, float y, float z) {
    chassis.setPose(x,y,z); 
    set_particles(chassis.getPose());
}

void MCLMoveToPose(double x, double y, double theta, int timeout) {
    chassis.moveToPose(x, y, theta, 
        timeout,lemlib::MoveToPoseParams{}, 
        true); 

    pros::delay(10);
    auto last_pose = chassis.getPose();
    while (chassis.isInMotion()) {
        auto cur = chassis.getPose();
        predict_models(cur, last_pose);
        weight_particles();
           double s2 = 0;
        for (int i = 0; i < no_particles; i++) s2 += particles[i][3] * particles[i][3];
        if (1.0 / s2 < no_particles / 2.0) resample();
        last_pose = cur;
        pros::delay(20);
    }
    chassis.setPose(estimate());
}

