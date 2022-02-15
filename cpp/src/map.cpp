//
// Created by DSlobodzian on 2/7/2022.
//

#include "map.hpp"

Map::Map() {
    Landmark l1(16.459252 - 13.2334, 8.22960 - 0.3683, blue_plate);
    Landmark l2(16.459252 - 13.2334, 8.22960 - 2.85, red_plate);
    Landmark l3(16.45925 / 2.0, 8.22960 / 2.0, blue_plate);
    Landmark l4(13.2334, 0.3683, red_plate);
    Landmark l5(13.2334, 2.85, goal);
    landmarks_.emplace_back(l1);
    landmarks_.emplace_back(l2);
    landmarks_.emplace_back(l3);
    landmarks_.emplace_back(l4);
    landmarks_.emplace_back(l5);
}

std::vector<Landmark> Map::get_landmarks() {
    return landmarks_;
}

