#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

class Boid
{
public:
    Boid();
    ~Boid();
private:
    Eigen::Vector2d make_separation_poser(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);
    Eigen::Vector2d make_alignment_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j,double Ir);
    Eigen::Vector2d make_gravity_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);
};
