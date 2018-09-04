//
// Created by han on 16-9-18.
//

#include "Riccati.h"

namespace humanoid {

    int Riccati::max_iteration_time = 1000;


    Riccati::Riccati(Eigen::Matrix3d& A, Eigen::Vector3d& B, Eigen::RowVector3d& C, double R, Eigen::Matrix<double, 4, 4>& Q){
        Eigen::Matrix<double, 3, 1> row_zero = Eigen::Matrix<double, 3, 1>::Zero();
        A_ << 1 , C*A, row_zero, A;
        B_ << C*B, B;
        C_ << 1, 0, 0, 0;
        R_ << R;
        Q_ = Q;
        P_ = Eigen::Matrix<double, 4, 4>::Identity();
    }

    bool Riccati::calc(){
        //Newton iterative method
        bool is_converged = false;

        for(int i = 0; i != max_iteration_time; ++i){
            auto ax = A_.transpose()*P_;
            auto axa = ax * A_;
            auto axb = ax * B_;
            auto m = (R_ + B_.transpose()*P_*B_);
            P_tem_ = axa - axb * m.inverse() * axb.transpose() + Q_;
            auto p_diff = P_tem_ - P_;
            double error = p_diff.norm() / P_tem_.norm();
            P_ = P_tem_;
            if(error < 1e-10){
                is_converged = true;
                break;
            }
        }
        return is_converged;
    }

    Eigen::Matrix<double, 4, 4> Riccati::get_p(){
        return P_;
    };
}
