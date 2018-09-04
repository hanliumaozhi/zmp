//
// Created by han on 17-11-13.
//

#ifndef HUMANOID_ORIENTATIONTRAJECTORY_H
#define HUMANOID_ORIENTATIONTRAJECTORY_H

#include "Common.h"
#include "trajectory/TrajectoryGenerator.h"

namespace humanoid {
    //enum class OrientType {COM, FOOT};

    class OrientationTrajectory {
    public:
        OrientationTrajectory(point3_t right_foot, point3_t left_foot, point3_t com,
                              int double_support_ticks, int single_support_ticks, int control_frequency,
                              bool is_left_first = true);
        OrientationTrajectory(const OrientationTrajectory&) = delete;

        bool set_waypoint(point3_list_ptr right_foot_list_, point3_list_ptr left_foot_list_,
                            point3_list_ptr com_list_);

        void calc();

        //for public access
        point3_list_ptr rf_list_;
        point3_list_ptr lf_list_;
        point3_list_ptr com_list_;

    private:
        point3_t right_foot_;
        point3_t left_foot_;
        point3_t com_;
        bool is_left_first_;
        int dsp_ticks_;
        int ssp_ticks_;
        int control_frequency_;

        double ssp_time_;

        point3_list_ptr right_foot_waypoint_list_;
        point3_list_ptr left_foot_waypoint_list_;
        point3_list_ptr com_waypoint_list_;

        std::shared_ptr<TrajectoryGenerator> roll_traj_gener_;
        std::shared_ptr<TrajectoryGenerator> pitch_traj_gener_;
        std::shared_ptr<TrajectoryGenerator> yaw_traj_gener_;


    };
}


#endif //HUMANOID_ORIENTATIONTRAJECTORY_H
