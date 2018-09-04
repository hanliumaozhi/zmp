//
// Created by han on 16-10-9.
//

#include "Cycloid.h"


namespace humanoid {

    Cycloid::Cycloid(int control_frequency, int double_support_ticks, int up_single_support_ticks, int front_single_support_ticks, point2_t first_zmp_point, double com_heigh):
            control_frequency_(control_frequency),
            double_support_ticks_(double_support_ticks),
            up_single_support_ticks_(up_single_support_ticks),
            front_single_support_ticks_(front_single_support_ticks),
            first_zmp_point_(first_zmp_point),
            CoM_heigh_(com_heigh){
                single_support_ticks_ = up_single_support_ticks + front_single_support_ticks;
                zmp_x_weight_ = 1.0;
                zmp_x_ = 0.0;
                com_x_ = 0.0;
                zmp_y_ = 0.0;
                com_y_ = 0.0;
            }


    void Cycloid::init(std::shared_ptr<std::vector<point2_t>> zmp_list, double stair_heigh){
        zmp_list_ = zmp_list;
        zmp_ = std::make_shared<ZMP>(control_frequency_, double_support_ticks_, single_support_ticks_, first_zmp_point_, CoM_heigh_);
        zmp_->set_com_position(com_x_, com_y_);
        zmp_->set_zmp_position(zmp_x_, zmp_y_);
        zmp_->init(zmp_list_);
        zmp_->calc();
        A_ = std::abs(std::get<0>(zmp_list_->at(0))-com_x_)/2.0;
        f_ = 1.0/((double_support_ticks_+single_support_ticks_)*1.0/control_frequency_);

        zmp_x_list_ = std::make_shared<std::vector<double>>();
        com_x_list_ = std::make_shared<std::vector<double>>();
        com_x_velocity_list_ = std::make_shared<std::vector<double>>();
        com_z_list_ = std::make_shared<std::vector<double>>();
        stair_heigh_ = stair_heigh;
    }

    void Cycloid::calc(){
        int half_single_support_ticks = single_support_ticks_/2;
        double current_x_base = com_x_;
        double current_zmp_x_base = zmp_x_;
        double tick_time = 1.0/(control_frequency_);
        double current_com_z_base = CoM_heigh_;
        double half_stair = stair_heigh_/2.0;
        for(int i = 0; i != (double_support_ticks_ + half_single_support_ticks); ++i){
            com_x_list_->push_back(0.0);
            com_x_velocity_list_->push_back(0.0);
            zmp_x_list_->push_back(0.0);
            com_z_list_->push_back(current_com_z_base);
        }
        for (int i = 0; i < zmp_list_->size(); ++i) {
            for (int j = 0; j < (double_support_ticks_ + single_support_ticks_); ++j) {
                com_x_list_->push_back((A_/(2*PIH))*(2*PIH*f_*tick_time*j-std::sin((2*PIH*f_*tick_time*j))) + current_x_base);
                com_z_list_->push_back((half_stair/(2*PIH))*(2*PIH*f_*tick_time*j-std::sin((2*PIH*f_*tick_time*j))) + current_com_z_base);
                com_x_velocity_list_->push_back((A_/(2*PIH))*(2*PIH*f_-2*PIH*f_*std::cos((2*PIH*f_*tick_time*j))));
                zmp_x_list_->push_back((A_/(2*PIH*zmp_x_weight_))*(zmp_x_weight_*2*PIH*f_*tick_time*j-(1+(CoM_heigh_/GRAVITY)*(2*PIH*f_)*(2*PIH*f_)*std::sin(2*PIH*f_*tick_time*j))) + current_zmp_x_base);
            }
            //current_x_base = com_x_list_->at((com_x_list_->size()-1));
            current_x_base += A_;
            current_zmp_x_base = zmp_x_list_->at((zmp_x_list_->size()-1));
            current_com_z_base += half_stair;
        }

        for(int i = 0; i != (3*double_support_ticks_ + 3*single_support_ticks_ + half_single_support_ticks); ++i){
            com_x_list_->push_back(current_x_base);
            com_x_velocity_list_->push_back(0.0);
            zmp_x_list_->push_back(current_zmp_x_base);
            com_z_list_->push_back(current_com_z_base);
        }

    }

    void Cycloid::set_position(double x, double y){
        zmp_x_ = x;
        com_x_ = x;
        zmp_y_ = y;
        com_y_ = y;
    }

    void Cycloid::set_zmp_x_weight(double weight){
        zmp_x_weight_ = weight;
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_zmp_ref_x(){
        return zmp_->get_zmp_ref_x();
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_zmp_ref_y(){
        return zmp_->get_zmp_ref_y();
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_zmp_x(){
        return zmp_x_list_;
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_zmp_y(){
        return zmp_->get_zmp_y();
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_com_x(){
        return com_x_list_;
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_com_y(){
        return zmp_->get_com_y();
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_com_x_velocity(){
        return com_x_velocity_list_;
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_com_y_velocity(){
        return zmp_->get_com_y_velocity();
    }

    std::shared_ptr<std::vector<double>> Cycloid::get_com_z()
    {
        return com_z_list_;
    }

}