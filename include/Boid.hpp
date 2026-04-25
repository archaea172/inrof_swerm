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
    Eigen::Vector2d make_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j,double Ir);
    Eigen::Vector2d x;

private:
    Eigen::Vector2d make_separation_poser(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);
    Eigen::Vector2d make_alignment_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j,double Ir);
    Eigen::Vector2d make_gravity_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir);

    double k_separation, k_alignment, k_gravity;
};

class Boids
{
public:
    Boids(int boid_num, double ir);
    std::vector<Boid> boids;

};
