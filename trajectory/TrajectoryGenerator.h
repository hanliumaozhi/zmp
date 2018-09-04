//
// Created by han on 16-9-20.
//

#ifndef HUMANOID_TRAJECTORYGENERATOR_H
#define HUMANOID_TRAJECTORYGENERATOR_H

#include <memory>
#include <vector>

namespace humanoid {
    enum class TrajectoryType {TWO_POINTS, THREE_POINTS_FLAT, TWO_POINTS_INC};
    class TrajectoryGenerator {
    public:
        TrajectoryGenerator(TrajectoryType type, double total_time, int interval_number);
        ~TrajectoryGenerator() = default;

        void set_position(double begin, double end);

        void set_H(double height);

        std::shared_ptr<std::vector<double>> get_result();

        void calc();

        double calc(int index);

        std::shared_ptr<std::vector<double>> p_data;


    protected:

        void calc_3_flat();
        void calc_2();

    private:
        TrajectoryType type_;
        double T_;
        int interval_number_;
        double interval_unit_;
        double H_;

        double shift_val_;
        double end_;

        double t_;
        double value_;
        double C_4_;
        double C_3_;
        double C_5_;


        std::shared_ptr<std::vector<double>> result_list_;


    };
}


#endif //HUMANOID_TRAJECTORYGENERATOR_H
