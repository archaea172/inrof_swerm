#include "follower.hpp"

follower::follower(double ir, double k_separation, double k_alignment, double k_gravity, double k_follow, double v_max)
: Boid(ir, k_separation, k_alignment, k_gravity, v_max)
{
    this->k_follow = k_follow;
}

Eigen::Vector2d follower::make_follow_power(const Eigen::Vector2d& x_i, const Eigen::Vector2d& x_l)
{
    Eigen::Vector2d l_i_diff = x_l - x_i;

    return l_i_diff / l_i_diff.norm();
}

void follower::update_pos(const Eigen::Vector2d& x_i, const Eigen::MatrixXd& x_j, const Eigen::MatrixXd& v_j, const Eigen::Vector2d& x_l, double Ir)
{
    this->vel = this->k_separation * this->make_separation_power(x_i, x_j, Ir) +
        this->k_alignment * this->make_alignment_power(x_i, x_j, v_j, Ir) +
        this->k_gravity * this->make_gravity_power(x_i, x_j, Ir) +
        this->k_follow * this->make_follow_power(x_i, x_l);
        
    this->vel << std::clamp(this->vel(0), -v_max_, v_max_), std::clamp(this->vel(1), -v_max_, v_max_);
    this->pos += this->vel;
}

followers::followers(int follower_num, double ir, double k_separation, double k_alignment, double k_gravity, double k_follow, double v_max)
{
    Followers.reserve(follower_num);
    for (int i = 0; i < follower_num; ++i)
    {
        Followers.emplace_back(ir, k_separation, k_alignment, k_gravity, k_follow, v_max);
    }
    this->follower_num_ = follower_num;
    this->ir = ir;
}

void followers::control_loop(const Eigen::Vector2d& leader_pos)
{
    Eigen::MatrixXd pre_pos(2, Followers.size());
    Eigen::MatrixXd pre_vel(2, Followers.size());
    for (int i = 0; i < static_cast<int>(Followers.size()); ++i) 
    {
        pre_pos.col(i) = Followers[i].pos;
        pre_vel.col(i) = Followers[i].vel;
    }
    for (int i = 0; i < this->follower_num_; ++i)
    {
        Eigen::Vector2d x_i = this->Followers[i].pos;
        Eigen::MatrixXd x_j = this->remove_col(pre_pos, i);
        Eigen::MatrixXd v_j = this->remove_col(pre_vel, i);
        this->Followers[i].update_pos(x_i, x_j, v_j, leader_pos, ir);
    }
}

Eigen::MatrixXd followers::remove_col(const Eigen::MatrixXd& A, int k)
{
    Eigen::MatrixXd B(A.rows(), A.cols() - 1);
    B.leftCols(k) = A.leftCols(k);
    B.rightCols(A.cols() - k - 1) = A.rightCols(A.cols() - k - 1);
    return B;
}
