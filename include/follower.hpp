#pragma once

#include "Boid.hpp"

class follower
: public Boid
{
public:
    follower(double ir, double k_separation, double k_alignment, double k_gravity, double k_follow, double v_max);
    void update_pos(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j, Eigen::Vector2d x_l, double Ir);
    Eigen::Vector2d make_follow_power(Eigen::Vector2d x_i, Eigen::Vector2d x_l);
protected:
    double k_follow;
};