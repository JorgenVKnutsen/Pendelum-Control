#include "doublependelum.h"




//Hardcoded dynamics

std::vector<double> DoublePendelum::get_derivatives(){
    double g = 9.81; 

    double l1 = pendelum1.get_length(); 
    double l2 = pendelum2.get_length(); 

    double m1 = pendelum1.get_mass();
    double m2 = pendelum2.get_mass(); 

    double theta1 = pendelum1.get_angle();
    double theta2 = pendelum2.get_angle();

    double theta1_dot = pendelum1.get_angular_velocity();
    double theta2_dot = pendelum2.get_angular_velocity();


    double cos_theta_diff = cos(theta1 - theta2);
    double sin_theta_diff = sin(theta1 - theta2);
    

    // First term in the equation for theta1_ddot
    double numerator1 = m2 * (l1 * sin_theta_diff * theta1_dot * theta1_dot - g * sin(theta2)) * cos_theta_diff;
    double denominator1 = l1 * (m1 + m2 * cos_theta_diff * cos_theta_diff) + l1 * m2;
    double term1 = numerator1 / denominator1;

    // Second term in the equation for theta1_ddot
    double numerator2 = -l2 * m2 * sin_theta_diff * theta2_dot * theta2_dot - g * m1 * sin(theta1) - g * m2 * sin(theta1);
    double denominator2 = l1 * (m1 + m2 * cos_theta_diff * cos_theta_diff) + l1 * m2;
    double term2 = numerator2 / denominator2;

    // Compute theta1_ddot
    double theta1_ddot = term1 + term2;


    //Theta 2 dot
    // first part
    double numer1_ = (m1 + m2) * (l1 * sin_theta_diff * theta1_dot * theta1_dot - g * sin(theta2));
    double denum1 = l2 * (m1 + m2 * cos_theta_diff * cos_theta_diff + m2);

    double term1_ = numer1_ / denum1;

    // Second term in the equation for theta2_ddot
    double numerator2a = -l1 * m1 * cos_theta_diff - l1 * m2 * cos_theta_diff;
    double numerator2b = -l2 * m2 * sin_theta_diff * theta2_dot * theta2_dot - g * m1 * sin(theta1) - g * m2 * sin(theta1);
    double numer2_= numerator2a * numerator2b;

    double denominator2_ = l1 * (m1 + m2) * (l2 * m1 + l2 * m2 * cos_theta_diff * cos_theta_diff + l2 * m2);

    double term2_ = numer2_ / denominator2_;

    // Compute theta2_ddot
    double theta2_ddot = term1_ + term2_;
    
    std::vector<double> derivatives = {theta1_dot, theta1_ddot, theta2_dot, theta2_ddot};
    return derivatives; 
}




void DoublePendelum::update_rk4(double dt){

    // Store the initial state
    double theta1_0 = pendelum1.get_angle();
    double theta1_dot_0 = pendelum1.get_angular_velocity();
    double theta2_0 = pendelum2.get_angle();
    double theta2_dot_0 = pendelum2.get_angular_velocity();

    // Get k1
    std::vector<double> k1 = get_derivatives();



    // Update the state to calculate k2
    update_state(k1[0] * 0.5 * dt, k1[1] * 0.5 * dt, k1[2] * 0.5 * dt, k1[3] * 0.5 * dt);
    std::vector<double> k2 = get_derivatives();

    // Revert to initial state and update to calculate k3
    pendelum1.update_angle(theta1_0 + 0.5 * dt * k2[0]);
    pendelum1.update_angular_velocity(theta1_dot_0 + 0.5 * dt * k2[1]);
    pendelum2.update_angle(theta2_0 + 0.5 * dt * k2[2]);
    pendelum2.update_angular_velocity(theta2_dot_0 + 0.5 * dt * k2[3]);
    std::vector<double> k3 = get_derivatives();

    // Update the state to calculate k4
    update_state(k3[0] * dt, k3[1] * dt, k3[2] * dt, k3[3] * dt);
    std::vector<double> k4 = get_derivatives();

    // Combine the slopes to get the new state
    double new_theta1_dot = theta1_dot_0 + (dt / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
    double new_theta2_dot = theta2_dot_0 + (dt / 6.0) * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]);
    double new_theta1 = theta1_0 + (dt / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
    double new_theta2 = theta2_0 + (dt / 6.0) * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]);

    // Update the state of the pendulums with the new values
    pendelum1.update_angular_velocity(new_theta1_dot);
    pendelum2.update_angular_velocity(new_theta2_dot);
    pendelum1.update_angle(new_theta1);
    pendelum2.update_angle(new_theta2);
}




void DoublePendelum::update_state(double theta1_dot, double theta1_ddot, double theta2_dot, double theta2_ddot) {
    // Update the angular velocities
    pendelum1.update_angular_velocity(theta1_dot + theta1_ddot * dt);
    pendelum2.update_angular_velocity(theta2_dot + theta2_ddot * dt);

    // Update the angles
    pendelum1.update_angle(pendelum1.get_angle() + theta1_dot * dt);
    pendelum2.update_angle(pendelum2.get_angle() + theta2_dot * dt);
}


    
std::vector<State> DoublePendelum::calculate_trajectory(double dt, int num_steps) {
    std::string file_name = "../output.csv";
    open_file(file_name);
    write_to_file("Theta1,Theta1_dot, Theta2, Theta2_dot\n");

    std::vector<State> trajectory;

    for (int i = 0; i < num_steps; ++i) {
        update_rk4(dt); 
        State state = {pendelum1.get_angle(), pendelum1.get_angular_velocity(),
                       pendelum2.get_angle(), pendelum2.get_angular_velocity()};
        trajectory.push_back(state);

        write_to_file(std::to_string(state.angle_1) + "," + 
                std::to_string(state.angular_velocity_1) + "," + 
                std::to_string(state.angle_2) + "," + 
                std::to_string(state.angular_velocity_2) + "\n");
    }
    close_file(); 
    return trajectory;
}




void DoublePendelum::open_file(const std::string& file_name) {
    file.open(file_name);
    if (!file.is_open()) {
        std::cout << "File opened" << std::endl; 
        throw std::ios_base::failure("Failed to open the file");
    }
}

void DoublePendelum::write_to_file(const std::string& data) {
    if (file.is_open()) {
        file << data;
    } else {
        std::cerr << "File is not open!" << std::endl;
    }
}

void DoublePendelum::close_file() {
    if (file.is_open()) {
        file.close();
    }
}