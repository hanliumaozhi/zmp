//
// Created by han on 16-10-9.
//

#ifndef HUMANOID_CYCLOID_H
#define HUMANOID_CYCLOID_H

//实现`A 3-TIER INFRASTRUCTURE: VIRTUAL-, MINI-, ONLINE-HUBO STAIRCLIMBING AS A CASE STUDY` 这篇论文描述的算法
//


#include <memory>
#include <cmath>

#include "Common.h"
#include "ZMP.h"


namespace humanoid {
    class Cycloid {
    public:
        Cycloid(int control_frequency, int double_support_ticks, int up_single_support_ticks, int front_single_support_ticks, point2_t first_zmp_point, double com_heigh);
        ~Cycloid() = default;

        void init(std::shared_ptr<std::vector<point2_t>> zmp_list, double stair_heigh);

        void calc();

        void set_position(double x, double y);

        void set_zmp_x_weight(double weight);

        std::shared_ptr<std::vector<double>> get_zmp_ref_x();
        std::shared_ptr<std::vector<double>> get_zmp_ref_y();

        std::shared_ptr<std::vector<double>> get_zmp_x();
        std::shared_ptr<std::vector<double>> get_zmp_y();

        std::shared_ptr<std::vector<double>> get_com_x();
        std::shared_ptr<std::vector<double>> get_com_y();
        std::shared_ptr<std::vector<double>> get_com_z();

        std::shared_ptr<std::vector<double>> get_com_x_velocity();
        std::shared_ptr<std::vector<double>> get_com_y_velocity();
    private:
        std::shared_ptr<ZMP> zmp_;
        int control_frequency_;
        int double_support_ticks_;
        int up_single_support_ticks_;
        int front_single_support_ticks_;
        point2_t first_zmp_point_;

        int single_support_ticks_;

        double CoM_heigh_;
        double stair_heigh_;

        double zmp_x_;
        double zmp_y_;

        double com_x_;
        double com_y_;

        std::shared_ptr<std::vector<point2_t>> zmp_list_;

        std::shared_ptr<std::vector<double>> zmp_x_list_;
        std::shared_ptr<std::vector<double>> com_x_list_;
        std::shared_ptr<std::vector<double>> com_z_list_;

        std::shared_ptr<std::vector<double>> com_x_velocity_list_;

        double zmp_x_weight_;

        //A_ is the amplitude
        double A_;

        //f_ is the frequency
        double f_;
    };
}


#endif //HUMANOID_CYCLOID_H
