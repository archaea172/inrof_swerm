#include "Boid.hpp"

Boid::Boid(double ir)
{
    static std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<> dist(-ir, ir);
    
    this->pos << dist(gen), dist(gen);
    this->vel << 0, 0;
}

Boid::~Boid()
{
}

void Boid::update_pos(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, Eigen::MatrixXd v_j, double Ir)
{
    this->vel = this->k_separation * this->make_separation_power(x_i, x_j, Ir) +
        this->k_alignment * this->make_alignment_power(x_i, x_j, v_j, Ir) +
        this->k_gravity * this->make_gravity_power(x_i, x_j, Ir);
        
    this->pos += this->vel;
}

Eigen::Vector2d Boid::make_separation_power(Eigen::Vector2d x_i, Eigen::MatrixXd x_j, double Ir)
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

Boids::Boids(int boid_num, double ir)
{
    boids.reserve(boid_num);
    for (int i = 0; i < boid_num; ++i)
    {
        boids.emplace_back(ir);
    }
    this->boid_num_ = boid_num;
    this->ir = ir;
}

void Boids::control_loop()
{
    Eigen::MatrixXd pre_pos(2, boids.size());
    Eigen::MatrixXd pre_vel(2, boids.size());
    for (int i = 0; i < boids.size(); ++i) 
    {
        pre_pos.col(i) = boids[i].pos;
        pre_vel.col(i) = boids[i].vel;
    }
    for (int i = i; i < this->boid_num_; ++i)
    {
        Eigen::Vector2d x_i = this->boids[i].pos;
        Eigen::MatrixXd x_j = this->remove_col(pre_pos, i);
        Eigen::MatrixXd v_j = this->remove_col(pre_vel, i);
        this->boids[i].update_pos(x_i, x_j, v_j, ir);
    }
}

Eigen::MatrixXd Boids::remove_col(const Eigen::MatrixXd& A, int k)
{
    Eigen::MatrixXd B(A.rows(), A.cols() - 1);
    B.leftCols(k) = A.leftCols(k);
    B.rightCols(A.cols() - k - 1) = A.rightCols(A.cols() - k - 1);
    return B;
}