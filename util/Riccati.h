//
// Created by han on 16-9-18.
//

#ifndef HUMANOID_RICCATI_H
#define HUMANOID_RICCATI_H


#include <eigen3/Eigen/Dense>

namespace humanoid {

    class Riccati {
    public:
        Riccati(Eigen::Matrix3d& A, Eigen::Vector3d& B, Eigen::RowVector3d& C, double R, Eigen::Matrix<double, 4, 4>& Q);
        ~Riccati() = default;

        Eigen::Matrix<double, 4, 4> get_p();

        bool calc();
    private:
        Eigen::Matrix<double, 4, 4> A_;
        Eigen::Matrix<double, 4, 1> B_;
        Eigen::Matrix<double, 4, 4> Q_;
        Eigen::Matrix<double, 1, 1> R_;
        Eigen::Matrix<double, 4, 4> P_;
        Eigen::Matrix<double, 4, 4> P_tem_;
        Eigen::Matrix<double, 1, 4> C_;

        static int max_iteration_time;
    };
}


#endif //HUMANOID_RICCATI_H
