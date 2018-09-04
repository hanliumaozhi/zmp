//
// Created by han on 16-9-19.
//

#include "PreviewController.h"

namespace humanoid {

    PreviewController::PreviewController(Eigen::Matrix3d& A, Eigen::Vector3d& B, Eigen::RowVector3d& C, double R, Eigen::Matrix<double, 4, 4>& Q, std::shared_ptr<std::vector<double>> output_val)
    {
        Eigen::Matrix<double, 3, 1> row_zero = Eigen::Matrix<double, 3, 1>::Zero();
        A_ << 1 , C*A, row_zero, A;
        B_ << C*B, B;
        R_ << R;
        Q_ = Q;

        a_ = A;
        b_ = B;
        c_ = C;

        preview_value_list_ = output_val;
        riccati_ = std::make_shared<Riccati>(A, B, C, R, Q);

        g_list_ = std::make_shared<std::vector<double>>();

        CoM_list_ = std::make_shared<std::vector<double>>();
        ZMP_list_ = std::make_shared<std::vector<double>>();
        CoM_Velocity_list_ = std::make_shared<std::vector<double>>();
        CoM_accelerate_list_ = std::make_shared<std::vector<double>>();
        u_ = std::make_shared<std::vector<double>>();
    }

    std::shared_ptr<std::vector<double>> PreviewController::get_g()
    {
        return g_list_;
    }

    bool PreviewController::calc_g(){
        if(not riccati_->calc()){
            return false;
        }

        Eigen::Matrix<double, 4, 1> X;
        Eigen::Matrix<double, 1, 4> K;
        Eigen::Matrix<double, 4, 4> P;
        Eigen::Matrix<double, 4, 4> A_c;
        Eigen::Matrix<double, 4, 1> X_init_tem;

        X_init_tem << 1, 0, 0, 0;

        P = riccati_->get_p();
        K = (R_ + B_.transpose()*P*B_).inverse()*B_.transpose()*P*A_;
        K_e_ = K(0, 0);
        K_x_ = K.block(0, 1, 1, 3);

        A_c = A_ - B_*K;
        X = -A_c.transpose()*P*X_init_tem;
        g_list_->push_back((-K_e_));
        for (size_t i = 1; i != preview_value_list_->size() ; ++i) {
            auto G_t = (R_ + B_.transpose()*P*B_).inverse()*B_.transpose()*X;
            g_list_->push_back(G_t(0, 0));
            X = A_c.transpose()*X;
        }

    }

    bool PreviewController::calc(double begin_position) {
        calc_g();
        double total_error = 0;
        double u;
        Eigen::Matrix<double, 1, 1> U;
        //begin com posiztion, velocity acceleration
        Eigen::Matrix<double, 3, 1> com;
        com << begin_position, 0, 0;
        CoM_list_->push_back(com(0, 0));
        CoM_Velocity_list_->push_back(com(1, 0));
        CoM_accelerate_list_->push_back(com(2, 0));
        ZMP_list_->push_back(((c_*com)(0, 0)));
        for(size_t i = 1; i != preview_value_list_->size(); ++i) {
            total_error += (ZMP_list_->at((i-1)) - preview_value_list_->at((i-1)));
            u = -K_e_*total_error - (K_x_*com)(0, 0) - calc_g_product((i-1));
            U(0, 0) = u;
            u_->push_back(u);
            com = a_ * com + b_ * U;
            CoM_list_->push_back(com(0, 0));
            CoM_Velocity_list_->push_back(com(1, 0));
            CoM_accelerate_list_->push_back(com(2, 0));
            ZMP_list_->push_back((c_*com)(0, 0));
        }
        return true;
    }

    double PreviewController::calc_g_product(size_t index){
        double result_val = 0.0;
        size_t total_length = preview_value_list_->size();
        for (size_t i = 0; i < total_length; ++i) {
            if((index+i) < total_length){
                result_val += (g_list_->at(i) * preview_value_list_->at((index+i)));
            }else{
                result_val += (g_list_->at(i) * preview_value_list_->at((total_length-1)));
            }
        }
        return result_val;
    }

    std::shared_ptr<std::vector<double>> PreviewController::get_zmp(){
        return ZMP_list_;

    }

    std::shared_ptr<std::vector<double>> PreviewController::get_com(){
        return CoM_list_;
    }

    std::shared_ptr<std::vector<double>> PreviewController::get_com_velocity(){
        return CoM_Velocity_list_;
    }

    std::shared_ptr<std::vector<double>> PreviewController::get_com_accelerate(){
        return CoM_accelerate_list_;
    }

    std::shared_ptr<std::vector<double>> PreviewController::get_u(){
        return u_;
    }
}
