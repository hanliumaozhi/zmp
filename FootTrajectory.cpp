//
// Created by han on 16-9-21.
//

#include "FootTrajectory.h"

namespace humanoid {
    FootTrajectory::FootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks, int single_support_ticks, int control_freqency):
                    right_foot_(right_foot),
                    left_foot_(left_foot),
                    double_support_ticks_(double_support_ticks),
                    single_support_ticks_(single_support_ticks),
                    is_left_first_(true){
                    right_position_list_ = std::make_shared<std::vector<point3_t>>();
                    left_position_list_ = std::make_shared<std::vector<point3_t>>();
                    single_support_time_ = (1.0/control_freqency) * single_support_ticks;
                }

    FootTrajectory::FootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks, int single_support_ticks, int control_freqency, bool is_left_first):
            right_foot_(right_foot),
            left_foot_(left_foot),
            double_support_ticks_(double_support_ticks),
            single_support_ticks_(single_support_ticks),
            is_left_first_(is_left_first){
        right_position_list_ = std::make_shared<std::vector<point3_t>>();
        left_position_list_ = std::make_shared<std::vector<point3_t>>();
        single_support_time_ = (1.0/control_freqency) * single_support_ticks;
    }

    void FootTrajectory::set_zmp_list_2D(std::shared_ptr<std::vector<point2_t>> zmp_list){
        zmp_list_ = std::make_shared<std::vector<point3_t>>();
        for(auto& item : *zmp_list){
            zmp_list_->push_back(std::make_tuple(std::get<0>(item), std::get<1>(item), 0));
        }
    }

    void FootTrajectory::init(double foot_height){
        foot_height_ = foot_height;
        x_list_ = std::make_shared<TrajectoryGenerator>(TrajectoryType::TWO_POINTS, single_support_time_, single_support_ticks_);
        y_list_ = std::make_shared<TrajectoryGenerator>(TrajectoryType::TWO_POINTS, single_support_time_, single_support_ticks_);
        z_list_ = std::make_shared<TrajectoryGenerator>(TrajectoryType::THREE_POINTS_FLAT, single_support_time_, single_support_ticks_);

        x_value_list_ = x_list_->get_result();
        y_value_list_ = y_list_->get_result();
        z_value_list_ = z_list_->get_result();
    }

    void FootTrajectory::calc(){
        for(int i = 0; i != (double_support_ticks_+single_support_ticks_); ++i){
            right_position_list_->push_back(right_foot_);
            left_position_list_->push_back(left_foot_);
        }
        for (int i = 0; i < zmp_list_->size(); ++i) {
            for (int j = 0; j < double_support_ticks_; ++j) {
                right_position_list_->push_back(right_foot_);
                left_position_list_->push_back(left_foot_);
            }
            if(i%2 == 0){
                if(is_left_first_) {
                    x_list_->set_position(std::get<0>(left_foot_), std::get<0>(zmp_list_->at(i)));
                    x_list_->calc();
                    y_list_->set_position(std::get<1>(left_foot_), std::get<1>(zmp_list_->at(i)));
                    y_list_->calc();
                    z_list_->set_H(foot_height_);
                    z_list_->calc();
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        left_position_list_->push_back(
                                std::make_tuple(x_value_list_->at(j), y_value_list_->at(j), z_value_list_->at(j)));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        right_position_list_->push_back(right_foot_);
                    }
                    left_foot_ = zmp_list_->at(i);
                } else {
                    x_list_->set_position(std::get<0>(right_foot_), std::get<0>(zmp_list_->at(i)));
                    x_list_->calc();
                    y_list_->set_position(std::get<1>(right_foot_), std::get<1>(zmp_list_->at(i)));
                    y_list_->calc();
                    z_list_->set_H(foot_height_);
                    z_list_->calc();
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        right_position_list_->push_back(std::make_tuple(x_value_list_->at(j), y_value_list_->at(j), z_value_list_->at(j)));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        left_position_list_->push_back(left_foot_);
                    }
                    right_foot_ = zmp_list_->at(i);
                }
            }else{
                if(is_left_first_) {
                    x_list_->set_position(std::get<0>(right_foot_), std::get<0>(zmp_list_->at(i)));
                    x_list_->calc();
                    y_list_->set_position(std::get<1>(right_foot_), std::get<1>(zmp_list_->at(i)));
                    y_list_->calc();
                    z_list_->set_H(foot_height_);
                    z_list_->calc();
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        right_position_list_->push_back(
                                std::make_tuple(x_value_list_->at(j), y_value_list_->at(j), z_value_list_->at(j)));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        left_position_list_->push_back(left_foot_);
                    }
                    right_foot_ = zmp_list_->at(i);
                }else{
                    x_list_->set_position(std::get<0>(left_foot_), std::get<0>(zmp_list_->at(i)));
                    x_list_->calc();
                    y_list_->set_position(std::get<1>(left_foot_), std::get<1>(zmp_list_->at(i)));
                    y_list_->calc();
                    z_list_->set_H(foot_height_);
                    z_list_->calc();
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        left_position_list_->push_back(
                                std::make_tuple(x_value_list_->at(j), y_value_list_->at(j), z_value_list_->at(j)));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        right_position_list_->push_back(right_foot_);
                    }
                    left_foot_ = zmp_list_->at(i);
                }
            }
        }

        // TODO: the end time
        for(int i = 0; i != 1*(double_support_ticks_+single_support_ticks_); ++i){
            right_position_list_->push_back(right_foot_);
            left_position_list_->push_back(left_foot_);
        }
    }

    std::shared_ptr<std::vector<point3_t>> FootTrajectory::get_right(){
        return right_position_list_;
    }

    std::shared_ptr<std::vector<point3_t>> FootTrajectory::get_left(){
        return left_position_list_;
    }
}
