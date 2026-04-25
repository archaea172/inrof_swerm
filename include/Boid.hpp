#pragma once

#include <vector>

class Boid
{
public:
    Boid();
    ~Boid();
private:
    double make_separation_poser(std::vector<double> x_i, std::vector<double> x_j, double Ir);
    double make_alignment_power(std::vector<double> x_i, std::vector<double> x_j, double Ir);
    double make_gravity_power(std::vector<double> x_i, std::vector<double> x_j, double Ir);
};
