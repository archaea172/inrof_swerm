#include "Boid.hpp"

Boid::Boid()
{
}

Boid::~Boid()
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
            sum(0) += posVD_i(0);
            sum(1) += posVD_i(1);
            in_num++;
        }
    }

    Eigen::Vector2d vel;
    vel << 0, 0;

    if (in_num != 0) vel = sum / in_num;

    return vel;
}