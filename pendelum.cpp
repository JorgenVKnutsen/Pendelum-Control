#include "pendelum.h"

double Pendelum::get_length() const {
    return length;
}

double Pendelum::get_mass() const {
    return mass;
}

double Pendelum::get_angle() const {
    return angle;
}

double Pendelum::get_angular_velocity() const {
    return angular_velocity;
}


void Pendelum::update_angle(double new_angle){
    angle = new_angle; 
}

void Pendelum::update_angular_velocity(double new_angular_velocity){
    angular_velocity = new_angular_velocity; 
} 


