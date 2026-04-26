#pragma once

#include "Boid.hpp"

class follower
: public Boid
{
public:
    follower(double ir, double k_separation, double k_alignment, double k_gravity, double k_follow, double v_max);
    void update_pos(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j, const Eigen::Vector2d& x_l, double Ir);
    Eigen::Vector2d make_follow_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_l);
protected:
    double k_follow;
};

class followers
{
public:
    followers(int follower_num, double ir, double k_separation, double k_alignment, double k_gravity, double k_follow, double v_max);
    void control_loop(const Eigen::Vector2d& leader_pos);

    std::vector<follower> Followers;

private:
    Eigen::MatrixXd remove_col(const Eigen::MatrixXd& A, int k);

    int follower_num_;
    double ir;
};