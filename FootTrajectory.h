//
// Created by han on 16-9-21.
//

#ifndef HUMANOID_FOOTTRAJECTORY_H
#define HUMANOID_FOOTTRAJECTORY_H

#include <memory>
#include <vector>

#include "Common.h"
#include "trajectory/TrajectoryGenerator.h"


namespace humanoid {


    class FootTrajectory {
    public:
        FootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks, int single_support_ticks, int control_freqency);
        FootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks, int single_support_ticks, int control_freqency, bool is_left_first);
        ~FootTrajectory() = default;

        void set_zmp_list_2D(std::shared_ptr<std::vector<point2_t>> zmp_list);

        void init(double foot_height);
        void calc();

        std::shared_ptr<std::vector<point3_t>> get_right();
        std::shared_ptr<std::vector<point3_t>> get_left();
    private:
        point3_t right_foot_;
        point3_t left_foot_;
        int double_support_ticks_;
        int single_support_ticks_;

        bool is_left_first_;

        double single_support_time_;

        double foot_height_;

        std::shared_ptr<std::vector<point3_t>> right_position_list_;
        std::shared_ptr<std::vector<point3_t>> left_position_list_;

        std::shared_ptr<std::vector<point3_t>> zmp_list_;

        std::shared_ptr<TrajectoryGenerator> x_list_;
        std::shared_ptr<TrajectoryGenerator> y_list_;
        std::shared_ptr<TrajectoryGenerator> z_list_;

        std::shared_ptr<std::vector<double>> x_value_list_;
        std::shared_ptr<std::vector<double>> y_value_list_;
        std::shared_ptr<std::vector<double>> z_value_list_;

    };
}


#endif //HUMANOID_FOOTTRAJECTORY_H
