//
// Created by han on 16-9-20.
//

#ifndef HUMANOID_ZMP_H
#define HUMANOID_ZMP_H

#include <eigen3/Eigen/Dense>

#include "Common.h"
#include "patten/PreviewController.h"
#include "trajectory/TrajectoryGenerator.h"


namespace humanoid {

    class ZMP {
    public:
        ZMP(int control_freqency, int double_support_ticks, int single_support_ticks, point2_t first_zmp_point, double com_heigh);
        ~ZMP() = default;

        void set_zmp_position(double x, double y);
        void set_com_position(double x, double y);

        void init(std::shared_ptr<std::vector<point2_t>> zmp_list);

        void calc();

        std::shared_ptr<std::vector<double>> get_zmp_ref_x();
        std::shared_ptr<std::vector<double>> get_zmp_ref_y();

        std::shared_ptr<std::vector<double>> get_zmp_x();
        std::shared_ptr<std::vector<double>> get_zmp_y();

        std::shared_ptr<std::vector<double>> get_com_x();
        std::shared_ptr<std::vector<double>> get_com_y();

        std::shared_ptr<std::vector<double>> get_com_x_velocity();
        std::shared_ptr<std::vector<double>> get_com_y_velocity();

        std::shared_ptr<std::vector<double>> get_com_x_accelerate();
        std::shared_ptr<std::vector<double>> get_com_y_accelerate();

        std::shared_ptr<std::vector<double>> get_x_u();
        std::shared_ptr<std::vector<double>> get_y_u();

    private:
        int control_freqency_;
        int double_support_ticks_;
        int single_support_ticks_;

        double tick_time_;
        double double_support_time_;
        double single_support_time_;

        double zmp_x_;
        double zmp_y_;

        double com_x_;
        double com_y_;

        point2_t first_zmp_point_;

        std::shared_ptr<std::vector<point2_t>> zmp_list_;

        std::shared_ptr<std::vector<double>> zmp_ref_x_;
        std::shared_ptr<std::vector<double>> zmp_ref_y_;

        std::shared_ptr<PreviewController> preview_x_;
        std::shared_ptr<PreviewController> preview_y_;

        Eigen::Matrix3d A_;
        Eigen::Matrix<double, 3, 1> B_;
        Eigen::Matrix<double, 1, 3> C_;
        Eigen::Matrix4d Q_;
        double r_;
        double g_;
        double h_;

    };
}

#endif //HUMANOID_ZMP_H
