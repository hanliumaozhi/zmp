//
// Created by han on 16-9-28.
//

#include "utest.h"

#include "matplotlibcpp.h"
#include "../Cycloid.h"

#include <iostream>

namespace plt = matplotlibcpp;

void test()
{

    // 这下面是脚的位置的坐标点，默认是先动左脚。
    //
    std::shared_ptr<std::vector<humanoid::point2_t>> point_list = std::make_shared<std::vector<humanoid::point2_t>>();
    point_list->push_back(std::make_tuple(0.15, 0.1));
    point_list->push_back(std::make_tuple(0.3, -0.1));
    point_list->push_back(std::make_tuple(0.45, 0.1));
    point_list->push_back(std::make_tuple(0.45, -0.1));


    humanoid::point2_t first_point = std::make_tuple(0.0, -0.1); //该点为zmp第一个要达到的航点

    //这里初始化zmp类，其中100为控制频率，15为双脚支撑期的tick，85为单脚支撑期的tick
    //first_point 为第一个航点的坐标
    // 0.8 为COM的高度
    // 然后此时com 和 zmp的位置都默认为0，可以通过 set_zmp_position 和 set_com_position 去改变值。
    humanoid::ZMP zmp(100, 20, 160, first_point, 0.8);

    //初始化计算，这里主要实现的生成zmp_ref曲线，参数为脚的坐标点的序列
    zmp.init(point_list);

    //zmp计算，得到zmp曲线，com曲线，com 加速度曲线（这个在稳定器中有用到）
    zmp.calc();

    std::shared_ptr<std::vector<double>> x = std::make_shared<std::vector<double>>();
    for (int i = 0; i < 1440; ++i) {
        x->push_back((0.01*i));
    }

    std::shared_ptr<std::vector<double>> zmp_ref_x = zmp.get_zmp_ref_x();
    std::shared_ptr<std::vector<double>> zmp_ref_y = zmp.get_zmp_ref_y();

    std::shared_ptr<std::vector<double>> zmp_x = zmp.get_zmp_x();
    std::shared_ptr<std::vector<double>> zmp_y = zmp.get_zmp_y();

    std::shared_ptr<std::vector<double>> com_x = zmp.get_com_x();
    std::shared_ptr<std::vector<double>> com_y = zmp.get_com_y();

    std::shared_ptr<std::vector<double>> com_x_vec = zmp.get_com_x_velocity();

    //下面为步行开始时脚的坐标，因为默认是从左脚开始，使用右脚的坐标与上式的zmp第一个航点相同
    humanoid::point3_t r_f = std::make_tuple(0.0, -0.1, 0.0);
    humanoid::point3_t l_f = std::make_tuple(0.0, 0.1, 0.0);

    //脚的路径类
    //其中前2个为左右脚坐标
    //后2个为双脚支撑期，单脚支撑期的tick
    //最后一个为控制频率
    humanoid::FootTrajectory ft(r_f, l_f, 20, 160, 100);

    //输入zmp的航点数据
    ft.set_zmp_list_2D(point_list);

    //初始化，其中参数为抬脚高度
    ft.init(0.03);

    //计算
    ft.calc();

    std::shared_ptr<std::vector<humanoid::point3_t>> rr = ft.get_right();
    std::shared_ptr<std::vector<humanoid::point3_t>> ll = ft.get_left();

    std::shared_ptr<std::vector<double>> l_x = std::make_shared<std::vector<double>>();
    std::shared_ptr<std::vector<double>> l_y = std::make_shared<std::vector<double>>();
    std::shared_ptr<std::vector<double>> l_z = std::make_shared<std::vector<double>>();
    std::shared_ptr<std::vector<double>> r_x = std::make_shared<std::vector<double>>();
    std::shared_ptr<std::vector<double>> r_y = std::make_shared<std::vector<double>>();
    std::shared_ptr<std::vector<double>> r_z = std::make_shared<std::vector<double>>();

    for (int i = 0; i < 1440; ++i) {
        l_x->push_back(std::get<0>(ll->at(i)));
        l_y->push_back(std::get<1>(ll->at(i)));
        l_z->push_back(std::get<2>(ll->at(i)));

        r_x->push_back(std::get<0>(rr->at(i)));
        r_y->push_back(std::get<1>(rr->at(i)));
        r_z->push_back(std::get<2>(rr->at(i)));
    }

    plt::subplot(2, 2, 1);
    plt::named_plot("zmp_ref_x", *x, *zmp_ref_x, "r-");
    plt::named_plot("zmp_x", *x, *zmp_x, "b-");
    plt::named_plot("com_x", *x, *com_x, "y-");
    plt::title("x axis");
    plt::legend();
    plt::subplot(2, 2, 2);
    plt::named_plot("zmp_ref_y", *x, *zmp_ref_y, "r-");
    plt::named_plot("zmp_y", *x, *zmp_y, "b-");
    plt::named_plot("com_y", *x, *com_y, "y-");
    plt::title("y axis");
    plt::legend();
    plt::subplot(2, 2, 3);
    plt::named_plot("x", *x, *l_x, "r-");
    plt::named_plot("y", *x, *l_y, "b-");
    plt::named_plot("z", *x, *l_z, "y-");
    plt::title("left");
    plt::ylim(-0.2, 0.5);
    plt::legend();
    plt::subplot(2, 2, 4);
    plt::named_plot("x", *x, *r_x, "r-");
    plt::named_plot("y", *x, *r_y, "b-");
    plt::named_plot("z", *x, *r_z, "y-");
    plt::title("right");
    plt::ylim(-0.2, 0.5);
    plt::legend();
    plt::show();
}


void cycloid_test()
{

    std::shared_ptr<std::vector<humanoid::point2_t>> point_list = std::make_shared<std::vector<humanoid::point2_t>>();
    point_list->push_back(std::make_tuple(0.15, 0.1));
    point_list->push_back(std::make_tuple(0.3, -0.1));
    point_list->push_back(std::make_tuple(0.45, 0.1));
    point_list->push_back(std::make_tuple(0.45, -0.1));


    humanoid::point2_t first_point = std::make_tuple(0.0, -0.1);

    humanoid::ZMP zmp(100, 20, 160, first_point, 0.8);

    zmp.init(point_list);

    zmp.calc();

    humanoid::Cycloid cyc(100, 20, 80, 80, first_point, 0.8);


    cyc.set_zmp_x_weight(-2.0);

    cyc.init(point_list, 0.05);

    cyc.calc();

    std::shared_ptr<std::vector<double>> x = std::make_shared<std::vector<double>>();
    for (int i = 0; i < 1440; ++i) {
        x->push_back((0.01*i));
    }

    std::shared_ptr<std::vector<double>> zmp_velocity_list = zmp.get_com_x_velocity();
    std::shared_ptr<std::vector<double>> cyc_velocity_list = cyc.get_com_x_velocity();

    std::shared_ptr<std::vector<double>> zmp_position_list = zmp.get_com_x();
    std::shared_ptr<std::vector<double>> cyc_position_list = cyc.get_com_x();

    std::shared_ptr<std::vector<double>> zmp_zmp_list = zmp.get_zmp_x();
    std::shared_ptr<std::vector<double>> cyc_zmp_list = cyc.get_zmp_x();

    plt::subplot(3, 1, 1);
    plt::named_plot("zmp", *x, *zmp_velocity_list, "r-");
    plt::named_plot("cyc", *x, *cyc_velocity_list, "b-");
    plt::title("velocity");
    plt::legend();
    plt::subplot(3, 1, 2);
    plt::named_plot("zmp", *x, *zmp_position_list, "r-");
    plt::named_plot("cyc", *x, *cyc_position_list, "b-");
    plt::title("position");
    plt::legend();
    plt::subplot(3, 1, 3);
    plt::named_plot("zmp", *x, *zmp_zmp_list, "r-");
    plt::named_plot("cyc", *x, *cyc_zmp_list, "b-");
    plt::title("zmp");
    plt::legend();

    plt::show();
}


void cycloid_foot_test()
{
    std::shared_ptr<std::vector<humanoid::point2_t>> point_list = std::make_shared<std::vector<humanoid::point2_t>>();
    point_list->push_back(std::make_tuple(0.15, 0.1));
    point_list->push_back(std::make_tuple(0.3, -0.1));
    point_list->push_back(std::make_tuple(0.45, 0.1));
    point_list->push_back(std::make_tuple(0.45, -0.1));

    humanoid::point3_t r_f = std::make_tuple(0.0, -0.1, 0.0);
    humanoid::point3_t l_f = std::make_tuple(0.0, 0.1, 0.0);

    humanoid::CycloidFootTrajectory ft(r_f, l_f, 20, 80, 80, 100);

    ft.init(0.2, point_list, 0.2);

    ft.calc();

    std::shared_ptr<std::vector<double>> x = std::make_shared<std::vector<double>>();
    for (int i = 0; i < 1440; ++i) {
        x->push_back((0.01*i));
    }

    std::shared_ptr<std::vector<humanoid::point3_t>> rr = ft.get_right();
    std::shared_ptr<std::vector<humanoid::point3_t>> ll = ft.get_left();

    std::shared_ptr<std::vector<double>> r_z = std::make_shared<std::vector<double>>();
    std::shared_ptr<std::vector<double>> l_z = std::make_shared<std::vector<double>>();

    for (auto &item: *rr) {
        r_z->push_back(std::get<2>(item));
    }

    for (auto &item: *ll) {
        l_z->push_back(std::get<2>(item));
    }

    plt::plot(*x, *l_z, "r-", *x, *r_z, "b-");
    plt::show();
}

void cycloid_export()
{
    int control_tick = 50;
    int double_support_tick = 90;
    int up_single_support_tick = 150;
    int front_single_support_tick = 150;
    double com_height = 0.75634;

    std::shared_ptr<std::vector<humanoid::point2_t>> point_list = std::make_shared<std::vector<humanoid::point2_t>>();
    /*point_list->push_back(std::make_tuple(0.26, -0.05));
    point_list->push_back(std::make_tuple(0.52, 0.05));
    point_list->push_back(std::make_tuple(0.78, -0.05));
    point_list->push_back(std::make_tuple(1.04, 0.05));
    point_list->push_back(std::make_tuple(1.3, -0.05));
    point_list->push_back(std::make_tuple(1.3, 0.05));*/
    point_list->push_back(std::make_tuple(0.26, -0.1));
    point_list->push_back(std::make_tuple(0.26, 0.1));
    point_list->push_back(std::make_tuple(0.52, -0.1));
    point_list->push_back(std::make_tuple(0.52, 0.1));
    point_list->push_back(std::make_tuple(0.78, -0.1));
    point_list->push_back(std::make_tuple(0.78, 0.1));
    point_list->push_back(std::make_tuple(1.04, -0.1));
    point_list->push_back(std::make_tuple(1.04, 0.1));
    point_list->push_back(std::make_tuple(1.3, -0.1));
    point_list->push_back(std::make_tuple(1.3, 0.1));


    humanoid::point2_t first_point = std::make_tuple(0.0, 0.1);

    humanoid::Cycloid cyc(control_tick, double_support_tick, up_single_support_tick, front_single_support_tick, first_point, com_height);

    cyc.set_zmp_x_weight(-2.0);

    cyc.init(point_list, 0.05);

    cyc.calc();

    //std::shared_ptr<std::vector<double>> x = std::make_shared<std::vector<double>>();
    //for (int i = 0; i < 1440; ++i) {
    //    x->push_back((0.01*i));
    //}

    //std::shared_ptr<std::vector<double>> zmp_velocity_list = zmp.get_com_x_velocity();
    //std::shared_ptr<std::vector<double>> cyc_velocity_list = cyc.get_com_x_velocity();

    //std::shared_ptr<std::vector<double>> zmp_position_list = zmp.get_com_x();
    //std::shared_ptr<std::vector<double>> cyc_position_list = cyc.get_com_x();

    //std::shared_ptr<std::vector<double>> zmp_zmp_list = zmp.get_zmp_x();
    //std::shared_ptr<std::vector<double>> cyc_zmp_list = cyc.get_zmp_x();

    std::shared_ptr<std::vector<double>> com_x_list = cyc.get_com_x();
    std::shared_ptr<std::vector<double>> com_y_list = cyc.get_com_y();
    std::shared_ptr<std::vector<double>> com_z_list = cyc.get_com_z();

    //for test
    humanoid::point3_t r_f = std::make_tuple(0.0, -0.1, 0.0);
    humanoid::point3_t l_f = std::make_tuple(0.0, 0.1, 0.0);

    humanoid::CycloidFootTrajectory ft(r_f, l_f, double_support_tick, up_single_support_tick, front_single_support_tick, control_tick, false);

    ft.init(0.05, point_list, 0.02);

    ft.calc();

    std::shared_ptr<std::vector<humanoid::point3_t>> rr = ft.get_right();
    std::shared_ptr<std::vector<humanoid::point3_t>> ll = ft.get_left();



    std::cout<<control_tick<<std::endl;
    std::cout<<0<<std::endl;

    for (int i = 0; i < com_x_list->size(); ++i) {
        std::cout<<(com_x_list->at(i) - std::get<0>(ll->at(i)))<<" ";
        std::cout<<(com_y_list->at(i) - std::get<1>(ll->at(i)))<<" ";
        std::cout<<(com_z_list->at(i) - std::get<2>(ll->at(i)))<<" ";
        std::cout<<(com_x_list->at(i) - std::get<0>(rr->at(i)))<<" ";
        std::cout<<(com_y_list->at(i) - std::get<1>(rr->at(i)))<<" ";
        std::cout<<(com_z_list->at(i) - std::get<2>(rr->at(i)))<<std::endl;
    }
}