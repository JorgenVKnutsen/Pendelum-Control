#ifndef DPENDELUM_H
#define DPENDELUM_H

#include "pendelum.h"
#include <cmath>
#include <vector>

#include <fstream> 


struct State {
    double angle_1;
    double angular_velocity_1;
    double angle_2;
    double angular_velocity_2;
};



class DoublePendelum{
    
    public:
    double dt = 0.1;  

    Pendelum pendelum1; 
    Pendelum pendelum2; 
    DoublePendelum(Pendelum p1, Pendelum p2) : pendelum1(p1), pendelum2(p2) {}



    std::vector<double> get_derivatives(); 
    void update_rk4(double dt); 
    void update_state(double theta1_dot, double theta1_ddot, double theta2_dot, double theta2_ddot); 

    std::vector<State> trajectory; 
    std::vector<State> calculate_trajectory(double dt, int num_steps); 


    void open_file(const std::string& file_name);
    void write_to_file(const std::string& data);
    void close_file(); 

    private: 
    std::ofstream file;
};


#endif