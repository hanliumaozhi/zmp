//
// Created by han on 16-10-13.
//

#ifndef HUMANOID_CYCLOIDFOOTTRAJECTORY_H
#define HUMANOID_CYCLOIDFOOTTRAJECTORY_H


#include <vector>
#include <memory>
#include <cmath>

#include "Common.h"

namespace humanoid {
    class CycloidFootTrajectory {
    public:
        CycloidFootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks, int up_single_support_ticks, int front_single_support_ticks, int control_freqency);
        CycloidFootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks, int up_single_support_ticks, int front_single_support_ticks, int control_freqency, bool is_left_first);

        void init(double stair_height, std::shared_ptr<std::vector<point2_t>> zmp_list, double up_over_stair);
        void calc();

        std::shared_ptr<std::vector<point3_t>> get_right();
        std::shared_ptr<std::vector<point3_t>> get_left();
        ~CycloidFootTrajectory() = default;
    private:
        point3_t right_foot_;
        point3_t left_foot_;

        int double_support_ticks_;

        int single_support_ticks_;
        int up_single_support_ticks_;
        int front_single_support_ticks_;

        int control_freqency_;
        double stair_height_;

        std::shared_ptr<std::vector<point3_t>> zmp_list_;

        std::shared_ptr<std::vector<point3_t>> right_position_list_;
        std::shared_ptr<std::vector<point3_t>> left_position_list_;

        double time_interval_;

        bool is_left_first_;

        double up_over_stair_;

    };
}


#endif //HUMANOID_CYCLOIDFOOTTRAJECTORY_H
