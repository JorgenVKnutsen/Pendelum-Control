#ifndef PENDELUM_H
#define PENDELUM_H

#include <iostream> 
#include <vector> 



class Pendelum{
    public: 
        double get_length() const;
        double get_mass() const;
        double get_angle() const;
        double get_angular_velocity() const;

        void update_angle(double new_angle);
        void update_angular_velocity(double new_angular_velocity); 


        Pendelum(double l, double m, double a, double a_v) : length(l), mass(m), angle(a), angular_velocity(a_v) {}


    private: 
        double length;
        double mass;
        double angle; 
        double angular_velocity; 
}; 


#endif
