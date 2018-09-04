//
// Created by han on 16-9-20.
//

#include "ZMP.h"

namespace humanoid{
    ZMP::ZMP(int control_freqency, int double_support_ticks, int single_support_ticks, point2_t first_zmp_point, double com_heigh):
        control_freqency_(control_freqency),
        double_support_ticks_(double_support_ticks),
        single_support_ticks_(single_support_ticks),
        first_zmp_point_(first_zmp_point),
        h_(com_heigh){
        tick_time_ = (1.0/control_freqency);
        double_support_time_ = tick_time_ * double_support_ticks;
        single_support_time_ = tick_time_ * single_support_ticks;
        zmp_x_ = 0.0;
        zmp_y_ = 0.0;
        zmp_ref_x_ = std::make_shared<std::vector<double>>();
        zmp_ref_y_ = std::make_shared<std::vector<double>>();
        com_x_ = 0.0;
        com_y_ = 0.0;
    }

    void ZMP::set_zmp_position(double x, double y){
        zmp_x_ = x;
        zmp_y_ = y;
    }

    void ZMP::init(std::shared_ptr<std::vector<point2_t>> zmp_list){
        zmp_list_ = zmp_list;
        r_ = 1e-6;
        g_ = 9.8;
        Eigen::Vector4d Q;
        Q << 1, 0, 0, 0;
        Q_ = Q.asDiagonal();
        double T = tick_time_;
        A_ << 1, T, (T*T)/2.0, 0, 1, T, 0, 0, 1;
        B_ << (T*T*T)/6.0, (T*T)/2.0, T;
        C_ << 1, 0, (-h_)/g_;

        TrajectoryGenerator t_x(TrajectoryType::TWO_POINTS, double_support_time_, double_support_ticks_);
        TrajectoryGenerator t_y(TrajectoryType::TWO_POINTS, double_support_time_, double_support_ticks_);
        std::shared_ptr<std::vector<double>> x_value_list = t_x.get_result();
        std::shared_ptr<std::vector<double>> y_value_list = t_y.get_result();
        point2_t pre_point;
        //init
        for(int i = 0; i != (double_support_ticks_ + single_support_ticks_); ++i){
            zmp_ref_x_->push_back(zmp_x_);
            zmp_ref_y_->push_back(zmp_y_);
        }
        //begin
        t_x.set_position(zmp_x_, std::get<0>(first_zmp_point_));
        t_y.set_position(zmp_y_, std::get<1>(first_zmp_point_));
        t_x.calc();
        t_y.calc();
        for (auto item: *x_value_list) {
            zmp_ref_x_->push_back(item);
        }
        for (auto item: *y_value_list) {
            zmp_ref_y_->push_back(item);
        }
        for(int i = 0; i != single_support_ticks_; ++i){
            zmp_ref_x_->push_back(std::get<0>(first_zmp_point_));
            zmp_ref_y_->push_back(std::get<1>(first_zmp_point_));
        }

        pre_point = first_zmp_point_;
        for(int i = 0; i != (zmp_list->size()-1); ++i){
            t_x.set_position(std::get<0>(pre_point), std::get<0>(zmp_list->at(i)));
            t_y.set_position(std::get<1>(pre_point), std::get<1>(zmp_list->at(i)));
            t_x.calc();
            t_y.calc();
            for (auto item: *x_value_list) {
                zmp_ref_x_->push_back(item);
            }
            for (auto item: *y_value_list) {
                zmp_ref_y_->push_back(item);
            }
            for(int j = 0; j != single_support_ticks_; ++j){
                zmp_ref_x_->push_back(std::get<0>(zmp_list->at(i)));
                zmp_ref_y_->push_back(std::get<1>(zmp_list->at(i)));
            }
            pre_point = zmp_list->at(i);
        }
        //end
        point2_t end_zmp_point = std::make_tuple((std::get<0>(zmp_list->at((zmp_list->size()-1)))+std::get<0>(pre_point))/2.0, (std::get<1>(zmp_list->at((zmp_list->size()-1)))+std::get<1>(pre_point))/2.0);

        t_x.set_position(std::get<0>(pre_point), std::get<0>(end_zmp_point));
        t_y.set_position(std::get<1>(pre_point), std::get<1>(end_zmp_point));
        t_x.calc();
        t_y.calc();
        for (auto item: *x_value_list) {
            zmp_ref_x_->push_back(item);
        }
        for (auto item: *y_value_list) {
            zmp_ref_y_->push_back(item);
        }
        // TODO:the end time
        for(int i = 0; i != (1*single_support_ticks_+0*double_support_ticks_); ++i){
            zmp_ref_x_->push_back(std::get<0>(end_zmp_point));
            zmp_ref_y_->push_back(std::get<1>(end_zmp_point));
        }

        preview_x_ = std::make_shared<PreviewController>(A_, B_, C_, r_, Q_, zmp_ref_x_);
        preview_y_ = std::make_shared<PreviewController>(A_, B_, C_, r_, Q_, zmp_ref_y_);

    }

    void ZMP::set_com_position(double x, double y){
        com_x_ = x;
        com_y_ = y;
    }

    void ZMP::calc(){
        preview_x_->calc(com_x_);
        preview_y_->calc(com_y_);
    }

    std::shared_ptr<std::vector<double>> ZMP::get_zmp_ref_x(){
        return zmp_ref_x_;
    }

    std::shared_ptr<std::vector<double>> ZMP::get_zmp_ref_y(){
        return zmp_ref_y_;
    }

    std::shared_ptr<std::vector<double>> ZMP::get_zmp_x(){
        return preview_x_->get_zmp();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_zmp_y(){
        return preview_y_->get_zmp();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_com_x(){
        return preview_x_->get_com();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_com_y(){
        return preview_y_->get_com();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_com_x_velocity(){
        return preview_x_->get_com_velocity();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_com_y_velocity(){
        return preview_y_->get_com_velocity();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_com_x_accelerate(){
        return preview_x_->get_com_accelerate();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_com_y_accelerate(){
        return preview_y_->get_com_accelerate();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_x_u(){
        return preview_x_->get_u();
    }

    std::shared_ptr<std::vector<double>> ZMP::get_y_u(){
        return preview_y_->get_u();
    }
}
