#include "pendelum.h"
#include "doublependelum.h"


int main() {
    // Initialize the two pendulums with some initial conditions
    Pendelum pendulum1(2.0, 10.0, M_PI - 0.001, 0.0); // l=1.0, m=1.0, theta=45 degrees, theta_dot=0
    Pendelum pendulum2(1.0, 5.0, M_PI, 0.0); // l=1.0, m=1.0, theta=30 degrees, theta_dot=0

    DoublePendelum double_pendulum(pendulum1, pendulum2);

    double dt = 0.01; // Time step
    int num_steps = 2000; // Number of steps to simulate

    std::vector<State> trajectory = double_pendulum.calculate_trajectory(dt, num_steps + 1);

    // Output the trajectory
    for (const auto& state : trajectory) {
        std::cout << "Theta1: " << state.angle_1 << ", Theta1_dot: " << state.angular_velocity_1
                  << ", Theta2: " << state.angle_2 << ", Theta2_dot: " << state.angular_velocity_2 << std::endl;
    }
    return 0; 
}