//
// Created by han on 16-10-13.
//


#include "CycloidFootTrajectory.h"


namespace humanoid {
    CycloidFootTrajectory::CycloidFootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks,
                                                 int up_single_support_ticks, int front_single_support_ticks,
                                                 int control_freqency):
                                                    right_foot_(right_foot),
                                                    left_foot_(left_foot),
                                                    double_support_ticks_(double_support_ticks),
                                                    up_single_support_ticks_(up_single_support_ticks),
                                                    front_single_support_ticks_(front_single_support_ticks),
                                                    control_freqency_(control_freqency),
                                                    is_left_first_(true){

        single_support_ticks_ = (up_single_support_ticks + front_single_support_ticks);
    }

    CycloidFootTrajectory::CycloidFootTrajectory(point3_t right_foot, point3_t left_foot, int double_support_ticks,
                                                 int up_single_support_ticks, int front_single_support_ticks,
                                                 int control_freqency, bool is_left_first):
                                                    right_foot_(right_foot),
                                                    left_foot_(left_foot),
                                                    double_support_ticks_(double_support_ticks),
                                                    up_single_support_ticks_(up_single_support_ticks),
                                                    front_single_support_ticks_(front_single_support_ticks),
                                                    control_freqency_(control_freqency),
                                                    is_left_first_(is_left_first){

        single_support_ticks_ = (up_single_support_ticks + front_single_support_ticks);
    }



    void CycloidFootTrajectory::init(double stair_height, std::shared_ptr<std::vector<point2_t>> zmp_list, double up_over_stair){
        stair_height_ = stair_height;
        zmp_list_ = std::make_shared<std::vector<point3_t>>();
        int heigh_counter = 1;
        for(int i=0; i != zmp_list->size(); ++i){
            if(i%2 == 0){
                zmp_list_->push_back(std::make_tuple(std::get<0>(zmp_list->at(i)), std::get<1>(zmp_list->at(i)), (stair_height*(heigh_counter))));
            }else{
                zmp_list_->push_back(std::make_tuple(std::get<0>(zmp_list->at(i)), std::get<1>(zmp_list->at(i)), (stair_height*(heigh_counter))));
                ++heigh_counter;
            }
            /*if((i+1) ==  zmp_list->size()){
                zmp_list_->push_back(std::make_tuple(std::get<0>(zmp_list->at(i)), std::get<1>(zmp_list->at(i)), (stair_height*(i))));
            }else{
                zmp_list_->push_back(std::make_tuple(std::get<0>(zmp_list->at(i)), std::get<1>(zmp_list->at(i)), (stair_height*(i+1))));
            }*/

        }
        right_position_list_ = std::make_shared<std::vector<point3_t>>();
        left_position_list_ = std::make_shared<std::vector<point3_t>>();
        time_interval_ = 1.0/control_freqency_;
        up_over_stair_ = up_over_stair;
    }

    void CycloidFootTrajectory::calc(){
        double Sd;
        double max_heigh;
        double total_f = (control_freqency_)*1.0/(single_support_ticks_);
        double up_f = (control_freqency_)*1.0/(up_single_support_ticks_);
        double front_f = (control_freqency_)*1.0/(front_single_support_ticks_);
        std::shared_ptr<std::vector<double>> x_values_t = std::make_shared<std::vector<double>>();
        std::shared_ptr<std::vector<double>> z_values_t = std::make_shared<std::vector<double>>();
        for(int i = 0; i != (double_support_ticks_ + single_support_ticks_); ++i){
            right_position_list_->push_back(right_foot_);
            left_position_list_->push_back(left_foot_);
        }
        for (int i = 0; i < zmp_list_->size(); ++i) {
            for (int j = 0; j < double_support_ticks_; ++j) {
                right_position_list_->push_back(right_foot_);
                left_position_list_->push_back(left_foot_);
            }
            x_values_t->clear();
            z_values_t->clear();
            if(i%2 == 0){
                if(is_left_first_) {
                    Sd = std::get<0>(zmp_list_->at(i)) - std::get<0>(left_foot_);
                    max_heigh = std::get<2>(zmp_list_->at(i)) - std::get<2>(left_foot_);
                    max_heigh += up_over_stair_;
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        x_values_t->push_back(((Sd / (2 * PIH)) * (2 * PIH * total_f * j * time_interval_ -
                                                                  sin(2 * PIH * total_f * j * time_interval_))) +
                                              std::get<0>(left_foot_));
                    }

                    for (int j = 0; j < up_single_support_ticks_; ++j) {
                        z_values_t->push_back(((max_heigh / (2 * PIH)) * (2 * PIH * up_f * j * time_interval_ -
                                                                         sin(2 * PIH * up_f * j * time_interval_))));
                    }

                    for (int j = 0; j < front_single_support_ticks_; ++j) {
                        z_values_t->push_back(((up_over_stair_ / (2 * PIH)) *
                                               (sin(2 * PIH * front_f * j * time_interval_) -
                                                2 * PIH * front_f * j * time_interval_) + max_heigh));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        left_position_list_->push_back(std::make_tuple(x_values_t->at(j), std::get<1>(left_foot_),
                                                                       (z_values_t->at(j) + std::get<2>(left_foot_))));
                        right_position_list_->push_back(right_foot_);
                    }
                    left_foot_ = zmp_list_->at(i);
                }else{
                    Sd = std::get<0>(zmp_list_->at(i)) - std::get<0>(right_foot_);
                    max_heigh = std::get<2>(zmp_list_->at(i)) - std::get<2>(right_foot_);
                    max_heigh += up_over_stair_;
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        x_values_t->push_back(((Sd/(2*PIH))*(2*PIH*total_f*j*time_interval_-sin(2*PIH*total_f*j*time_interval_)))+std::get<0>(right_foot_));
                    }

                    for (int j = 0; j < up_single_support_ticks_; ++j) {
                        z_values_t->push_back(((max_heigh/(2*PIH))*(2*PIH*up_f*j*time_interval_-sin(2*PIH*up_f*j*time_interval_))));
                    }

                    for (int j = 0; j < front_single_support_ticks_; ++j) {
                        z_values_t->push_back(((up_over_stair_/(2*PIH))*(sin(2*PIH*front_f*j*time_interval_)-2*PIH*front_f*j*time_interval_) + max_heigh));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        right_position_list_->push_back(std::make_tuple(x_values_t->at(j), std::get<1>(right_foot_), (z_values_t->at(j)+std::get<2>(right_foot_))));
                        left_position_list_->push_back(left_foot_);
                    }
                    right_foot_ = zmp_list_->at(i);
                }
            }else{
                if(is_left_first_) {
                    Sd = std::get<0>(zmp_list_->at(i)) - std::get<0>(right_foot_);
                    max_heigh = std::get<2>(zmp_list_->at(i)) - std::get<2>(right_foot_);
                    max_heigh += up_over_stair_;
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        x_values_t->push_back(((Sd / (2 * PIH)) * (2 * PIH * total_f * j * time_interval_ -
                                                                  sin(2 * PIH * total_f * j * time_interval_))) +
                                              std::get<0>(right_foot_));
                    }

                    for (int j = 0; j < up_single_support_ticks_; ++j) {
                        z_values_t->push_back(((max_heigh / (2 * PIH)) * (2 * PIH * up_f * j * time_interval_ -
                                                                         sin(2 * PIH * up_f * j * time_interval_))));
                    }

                    for (int j = 0; j < front_single_support_ticks_; ++j) {
                        z_values_t->push_back(((up_over_stair_ / (2 * PIH)) *
                                               (sin(2 * PIH * front_f * j * time_interval_) -
                                                2 * PIH * front_f * j * time_interval_) + max_heigh));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        right_position_list_->push_back(std::make_tuple(x_values_t->at(j), std::get<1>(right_foot_),
                                                                        (z_values_t->at(j) +
                                                                         std::get<2>(right_foot_))));
                        left_position_list_->push_back(left_foot_);
                    }
                    right_foot_ = zmp_list_->at(i);
                }else{
                    Sd = std::get<0>(zmp_list_->at(i)) - std::get<0>(left_foot_);
                    max_heigh = std::get<2>(zmp_list_->at(i)) - std::get<2>(left_foot_);
                    max_heigh += up_over_stair_;
                    for (int j = 0; j < single_support_ticks_; ++j) {
                        x_values_t->push_back(((Sd / (2 * PIH)) * (2 * PIH * total_f * j * time_interval_ -
                                                                  sin(2 * PIH * total_f * j * time_interval_))) +
                                              std::get<0>(left_foot_));
                    }

                    for (int j = 0; j < up_single_support_ticks_; ++j) {
                        z_values_t->push_back(((max_heigh / (2 * PIH)) * (2 * PIH * up_f * j * time_interval_ -
                                                                         sin(2 * PIH * up_f * j * time_interval_))));
                    }

                    for (int j = 0; j < front_single_support_ticks_; ++j) {
                        z_values_t->push_back(((up_over_stair_ / (2 * PIH)) *
                                               (sin(2 * PIH * front_f * j * time_interval_) -
                                                2 * PIH * front_f * j * time_interval_) + max_heigh));
                    }

                    for (int j = 0; j < single_support_ticks_; ++j) {
                        left_position_list_->push_back(std::make_tuple(x_values_t->at(j), std::get<1>(left_foot_),
                                                                       (z_values_t->at(j) + std::get<2>(left_foot_))));
                        right_position_list_->push_back(right_foot_);
                    }
                    left_foot_ = zmp_list_->at(i);
                }
            }
        }
        for(int i = 0; i != 3*(double_support_ticks_ + single_support_ticks_); ++i){
            right_position_list_->push_back(right_foot_);
            left_position_list_->push_back(left_foot_);
        }
    }

    std::shared_ptr<std::vector<point3_t>> CycloidFootTrajectory::get_right(){
        return right_position_list_;
    }

    std::shared_ptr<std::vector<point3_t>> CycloidFootTrajectory::get_left(){
        return left_position_list_;
    }
}
