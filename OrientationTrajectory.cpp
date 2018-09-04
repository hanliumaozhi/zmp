//
// Created by han on 17-11-13.
//

#include "OrientationTrajectory.h"

namespace humanoid {
    OrientationTrajectory::OrientationTrajectory(point3_t right_foot, point3_t left_foot, point3_t com,
                                                 int double_support_ticks, int single_support_ticks,
                                                 int control_frequency,
                                                 bool is_left_first) {
        is_left_first_ = is_left_first;
        right_foot_ = right_foot;
        left_foot_ = left_foot;
        ssp_ticks_ = single_support_ticks;
        dsp_ticks_ = double_support_ticks;
        control_frequency_ = control_frequency;
        ssp_time_ = (1.0 / control_frequency) * single_support_ticks;
        com_ = com;
        is_left_first_ = is_left_first;
        roll_traj_gener_ = std::make_shared<TrajectoryGenerator>(TrajectoryType::TWO_POINTS, ssp_time_, ssp_ticks_);
        pitch_traj_gener_ = std::make_shared<TrajectoryGenerator>(TrajectoryType::TWO_POINTS, ssp_time_, ssp_ticks_);
        yaw_traj_gener_ = std::make_shared<TrajectoryGenerator>(TrajectoryType::TWO_POINTS, ssp_time_, ssp_ticks_);
        lf_list_ = std::make_shared<std::vector<point3_t>>();
        rf_list_ = std::make_shared<std::vector<point3_t>>();
        com_list_ = std::make_shared<std::vector<point3_t>>();
    }


    bool OrientationTrajectory::set_waypoint(point3_list_ptr right_foot_list, point3_list_ptr left_foot_list,
                                             point3_list_ptr com_list)
    {
        if((right_foot_list->size() == left_foot_list->size()) and (left_foot_list->size() == com_list->size()))
        {
            right_foot_waypoint_list_ = right_foot_list;
            left_foot_waypoint_list_  = left_foot_list;
            com_waypoint_list_ = com_list;
            return true;
        }
        else
        {
            return false;
        }
    }

    void OrientationTrajectory::calc()
    {
        if(is_left_first_){
            for (int i = 0; i != (ssp_ticks_ + dsp_ticks_) ; ++i) {
                right_foot_waypoint_list_->push_back(right_foot_);
                left_foot_waypoint_list_->push_back(left_foot_);
            }
            for (auto i = 0; i < com_waypoint_list_->size(); ++i) {
                // for dsp
                for (auto j = 0; j < dsp_ticks_; ++j) {
                    rf_list_->push_back(right_foot_);
                    lf_list_->push_back(left_foot_);
                    com_list_->push_back(com_);
                }


                roll_traj_gener_->set_position(std::get<0>(left_foot_), std::get<0>(left_foot_waypoint_list_->at(i)));
                roll_traj_gener_->calc();
                pitch_traj_gener_->set_position(std::get<1>(left_foot_), std::get<1>(left_foot_waypoint_list_->at(i)));
                pitch_traj_gener_->calc();
                yaw_traj_gener_->set_position(std::get<2>(left_foot_), std::get<2>(left_foot_waypoint_list_->at(i)));
                yaw_traj_gener_->calc();
                for (auto j = 0; j < ssp_ticks_ ; ++j) {
                    lf_list_->push_back(std::make_tuple(roll_traj_gener_->p_data->at(j),
                                                        pitch_traj_gener_->p_data->at(j),
                                                        yaw_traj_gener_->p_data->at(j)));
                }

                roll_traj_gener_->set_position(std::get<0>(right_foot_), std::get<0>(right_foot_waypoint_list_->at(i)));
                roll_traj_gener_->calc();
                pitch_traj_gener_->set_position(std::get<1>(right_foot_), std::get<1>(right_foot_waypoint_list_->at(i)));
                pitch_traj_gener_->calc();
                yaw_traj_gener_->set_position(std::get<2>(right_foot_), std::get<2>(right_foot_waypoint_list_->at(i)));
                yaw_traj_gener_->calc();
                for (auto j = 0; j < ssp_ticks_ ; ++j) {
                    rf_list_->push_back(std::make_tuple(roll_traj_gener_->p_data->at(j),
                                                        pitch_traj_gener_->p_data->at(j),
                                                        yaw_traj_gener_->p_data->at(j)));
                }

                roll_traj_gener_->set_position(std::get<0>(com_), std::get<0>(com_waypoint_list_->at(i)));
                roll_traj_gener_->calc();
                pitch_traj_gener_->set_position(std::get<1>(com_), std::get<1>(com_waypoint_list_->at(i)));
                pitch_traj_gener_->calc();
                yaw_traj_gener_->set_position(std::get<2>(com_), std::get<2>(com_waypoint_list_->at(i)));
                yaw_traj_gener_->calc();
                for (auto j = 0; j < ssp_ticks_ ; ++j) {
                    com_list_->push_back(std::make_tuple(roll_traj_gener_->p_data->at(j),
                                                        pitch_traj_gener_->p_data->at(j),
                                                        yaw_traj_gener_->p_data->at(j)));
                }

                com_ = com_waypoint_list_->at(i);
                right_foot_ = right_foot_waypoint_list_->at(i);
                left_foot_ = left_foot_waypoint_list_->at(i);
            }

            for (int i = 0; i < (dsp_ticks_ + ssp_ticks_); ++i) {
                com_list_->push_back(com_);
                rf_list_->push_back(right_foot_);
                lf_list_->push_back(left_foot_);
            }
        }else{
            //TODO: implement
        }
    }
}
