//
// Created by han on 16-9-20.
//

#include "TrajectoryGenerator.h"

namespace humanoid{
    TrajectoryGenerator::TrajectoryGenerator(TrajectoryType type, double total_time, int interval_number):
                        type_(type),
                        T_(total_time),
                        interval_number_(interval_number){
                            interval_unit_ = total_time / interval_number;
                            result_list_ = std::make_shared<std::vector<double>>();
                            p_data = result_list_;
                        }

    void TrajectoryGenerator::set_position(double begin, double end){
        shift_val_ = begin;
        end_ = (end - begin);
        if(type_ == TrajectoryType::TWO_POINTS_INC){
            C_5_ = (6*end_)/(T_*T_*T_*T_*T_);
            C_4_ = (-15*end_)/(T_*T_*T_*T_);
            C_3_ = (10*end_)/(T_*T_*T_);
        }
    }

    void TrajectoryGenerator::set_H(double height){
        H_ = height;
    }

    std::shared_ptr<std::vector<double>> TrajectoryGenerator::get_result(){
        return result_list_;
    };

    void TrajectoryGenerator::calc_3_flat() {
        result_list_->clear();

        double C_4 = (16*H_)/(T_*T_*T_*T_);
        double C_3 = (-32*H_)/(T_*T_*T_);
        double C_2 = (16*H_)/(T_*T_);

        double t;
        double value;
        for (int i = 0; i != interval_number_ ; ++i) {
            t = (interval_unit_*i);
            value = 0;
            value += (t*t) * C_2;
            value += (t*t*t) * C_3;
            value += (t*t*t*t) * C_4;
            result_list_->push_back(value);
        }
    }

    void TrajectoryGenerator::calc_2(){
        result_list_->clear();

        double C_5 = (6*end_)/(T_*T_*T_*T_*T_);
        double C_4 = (-15*end_)/(T_*T_*T_*T_);
        double C_3 = (10*end_)/(T_*T_*T_);

        double t;
        double value;
        for (int i = 0; i != interval_number_; ++i) {
            t = (interval_unit_*i);
            value = 0;
            value += (t*t*t) * C_3;
            value += (t*t*t*t) * C_4;
            value += (t*t*t*t*t) * C_5;
            result_list_->push_back((value+shift_val_));
        }
    }

    void TrajectoryGenerator::calc(){
        if(type_ == TrajectoryType::TWO_POINTS){
            calc_2();
        } else{
            calc_3_flat();
        }
    }

    double TrajectoryGenerator::calc(int index){

        t_ = (interval_unit_*index);
        value_ = 0;
        value_ += (t_*t_*t_) * C_3_;
        value_ += (t_*t_*t_*t_) * C_4_;
        value_ += (t_*t_*t_*t_*t_) * C_5_;

        return (value_+shift_val_);
    }
}
