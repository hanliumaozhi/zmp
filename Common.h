//
// Created by han on 16-9-21.
//

#ifndef HUMANOID_COMMON_H
#define HUMANOID_COMMON_H

#include <tuple>
#include <vector>
#include <memory>

#define PIH 3.14159265
#define GRAVITY 9.81

namespace humanoid {

    using point3_t = std::tuple<double, double, double>;
    using point2_t = std::tuple<double, double>;

    using point3_list_ptr = std::shared_ptr<std::vector<point3_t>>;
}

#endif //HUMANOID_COMMON_H
