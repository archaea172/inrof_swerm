#include "follower.hpp"

follower::follower(double ir, double k_separation, double k_alignment, double k_gravity, double k_follow, double v_max)
: Boid(ir, k_separation, k_alignment, k_gravity, v_max)
{
    this->k_follow = k_follow;
}

Eigen::Vector2d follower::make_follow_power(Eigen::Vector2d x_i, Eigen::Vector2d x_l)
{
    Eigen::Vector2d l_i_diff = x_l - x_i;

    return l_i_diff / l_i_diff.norm();
}

void follower::update_pos(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j, Eigen::Vector2d x_l, double Ir)
{
    this->vel = this->k_separation * this->make_separation_power(x_i, x_j, Ir) +
        this->k_alignment * this->make_alignment_power(x_i, x_j, v_j, Ir) +
        this->k_gravity * this->make_gravity_power(x_i, x_j, Ir) +
        this->k_follow * this->make_follow_power(x_i, x_l);
        
    this->vel << std::clamp(this->vel(0), -v_max_, v_max_), std::clamp(this->vel(1), -v_max_, v_max_);
    this->pos += this->vel;
}
