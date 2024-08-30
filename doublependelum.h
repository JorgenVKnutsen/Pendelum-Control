#ifndef DPENDELUM_H
#define DPENDELUM_H

#include "pendelum.h"
#include <cmath>
#include <vector>


class DoublePendelum{
    public: 
    Pendelum pendelum1; 
    Pendelum pendelum2; 
    DoublePendelum(Pendelum p1, Pendelum p2) : pendelum1(p1), pendelum2(p2) {}


    std::vector<double> get_derivatives(); 
    void update_rk4(double dt); 




};


#endif