//
// Created by DSlobodzian on 2/7/2022.
//
#pragma once

#include <iostream>
#include <vector>

#define FIELD_LENGTH

// number correlates to the ID of the YoloV5 model.
enum game_elements {
    blue_ball = 0,
    goal = 1,
    red_ball = 2,
    blue_plate = 3, // not used
    red_plate = 4 // not used
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
    Map();
    ~Map() = default;

    std::vector<Landmark> get_landmarks();

};


