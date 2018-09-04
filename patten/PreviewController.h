#ifndef HUMANOID_PREVIEWCONTROLLER_H
#define HUMANOID_PREVIEWCONTROLLER_H

#include <memory>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "../util/Riccati.h"

namespace humanoid {
    /**
     *  @class PreviewController
     *
     *  @brief a simple implemention for paper
     *
     *  @author liu song
     */
    class PreviewController {
    private:
        std::shared_ptr<std::vector<double>> preview_value_list_;
        std::shared_ptr<std::vector<double>> g_list_;
        std::shared_ptr<std::vector<double>> CoM_list_;
        std::shared_ptr<std::vector<double>> ZMP_list_;
        std::shared_ptr<std::vector<double>> CoM_Velocity_list_;
        std::shared_ptr<std::vector<double>> CoM_accelerate_list_;
        std::shared_ptr<std::vector<double>> u_;

        Eigen::Matrix<double, 4, 4> A_;
        Eigen::Matrix<double, 4, 1> B_;
        Eigen::Matrix<double, 4, 4> Q_;
        Eigen::Matrix<double, 1, 1> R_;

        Eigen::Matrix<double, 3, 3> a_;
        Eigen::Matrix<double, 3, 1> b_;
        Eigen::Matrix<double, 1, 3> c_;

        double K_e_;
        Eigen::Matrix<double, 1, 3> K_x_;

        std::shared_ptr<Riccati> riccati_;
    protected:
        bool calc_g();
        double calc_g_product(size_t index);
    public:
        PreviewController(Eigen::Matrix3d& A, Eigen::Vector3d& B, Eigen::RowVector3d& C, double R, Eigen::Matrix<double, 4, 4>& Q, std::shared_ptr<std::vector<double>> output_val);
        ~PreviewController() = default;
        std::shared_ptr<std::vector<double>> get_g();
        std::shared_ptr<std::vector<double>> get_zmp();
        std::shared_ptr<std::vector<double>> get_com();
        std::shared_ptr<std::vector<double>> get_com_velocity();
        std::shared_ptr<std::vector<double>> get_com_accelerate();
        std::shared_ptr<std::vector<double>> get_u();


        bool calc(double begin_position);
    };
}


#endif //HUMANOID_PREVIEWCONTROLLER_H
