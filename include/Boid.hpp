#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <vector>
#include <cmath>
#include <random>

class Boid
{
public:
    Boid(double ir, double k_separation, double k_alignment, double k_gravity, double v_max);
    ~Boid();
    void update_pos(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j, double Ir);

    Eigen::Vector2d pos;
    Eigen::Vector2d vel;

private:
    Eigen::Vector2d make_separation_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);
    Eigen::Vector2d make_alignment_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j,double Ir);
    Eigen::Vector2d make_gravity_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);

    double k_separation, k_alignment, k_gravity;
    double v_max_;
};

class Boids
{
public:
    Boids(int boid_num, double ir, double k_separation, double k_alignment, double k_gravity);
    void control_loop();

    std::vector<Boid> boids;
private:
    Eigen::MatrixXd remove_col(const Eigen::MatrixXd& A, int k);

    int boid_num_;
    double ir;
};
