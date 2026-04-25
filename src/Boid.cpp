#include "Boid.hpp"

Boid::Boid()
{
}

Boid::~Boid()
{
}

Eigen::Vector2d Boid::make_power()
{
    
}

Eigen::Vector2d Boid::make_separation_poser(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir)
{
    double I_r_2 = pow(Ir, 2);
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (I_r_2 > D_square) 
        {
            Eigen::Vector2d posVD_i = i_j_diff / pow(D_square, 1.5);
            sum += posVD_i;
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return -vel;
}

Eigen::Vector2d Boid::make_alignment_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j,double Ir)
{
    double I_r_2 = pow(Ir, 2);
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (I_r_2 > D_square) 
        {
            double velVN = v_j.col(i).norm();
            Eigen::Vector2d velD = v_j.row(i) / std::max(velVN, 1.0);
            sum += velD;
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return vel;
}

Eigen::Vector2d Boid::make_gravity_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir)
{
    double I_r_2 = pow(Ir, 2);
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int in_num = 0;
    for (size_t i = 0; i < x_j.cols(); i++) 
    {
        Eigen::Vector2d i_j_diff = x_i - x_j.col(i);
        double D_square = i_j_diff.squaredNorm();
        if (I_r_2 > D_square) 
        {
            double use_d = std::max(sqrt(D_square), 1.0);
            sum += i_j_diff / use_d;
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return vel;
}