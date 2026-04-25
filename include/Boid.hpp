#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <random>

class Boid
{
public:
    Boid(double ir);
    ~Boid();
    void update_pos(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j, double Ir);

    Eigen::Vector2d pos;
    Eigen::Vector2d vel;

private:
    Eigen::Vector2d make_separation_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);
    Eigen::Vector2d make_alignment_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j,double Ir);
    Eigen::Vector2d make_gravity_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);

    double k_separation, k_alignment, k_gravity;
};

class Boids
{
public:
    Boids(int boid_num, double ir);
    void control_loop();

    std::vector<Boid> boids;
private:
    Eigen::MatrixXd remove_col(const Eigen::MatrixXd& A, int k);

    int boid_num_;
    double ir;
};
