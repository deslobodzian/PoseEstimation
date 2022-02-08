//
// Created by DSlobodzian on 2/7/2022.
//
#pragma once

#include <iostream>
#include <vector>

enum game_elements {
    blue_ball = 0,
    rad_ball = 1,
    blue_plate = 2,
    red_plate = 3,
    goal = 4
};

struct Landmark {
    double x;
    double y;
    game_elements game_element;
    Landmark(double x_pos, double y_pos, game_elements element) {
        x = x_pos;
        y = y_pos;
        game_element = element;
    }
};

class Map {

private:
    std::vector<Landmark> landmarks_;

public:
    Map(std::vector<Landmark> landmarks);
    ~Map() = default;

    std::vector<Landmark> get_landmarks();

};


