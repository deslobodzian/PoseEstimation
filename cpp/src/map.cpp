//
// Created by DSlobodzian on 2/7/2022.
//

#include "map.hpp"

Map::Map(std::vector<Landmark> landmarks) {
    landmarks_ = landmarks;
}

std::vector<Landmark> Map::get_landmarks() {
    return landmarks_;
}

