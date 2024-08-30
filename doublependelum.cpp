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
    std::vector<double> states = get_derivatives(); 



}