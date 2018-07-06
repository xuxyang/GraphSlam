//
//  robot.cpp
//  GraphSLAM
//
//  Created by XUXIA YANG on 7/5/18.
//  Copyright © 2018 XUXIA YANG. All rights reserved.
//

#include <stdio.h>
#include <cmath>
#include "robot.h"

using namespace std;

void sense(vector<Position> &landmarks, Position &robot_position, vector<Measurement> &measurements) {
    measurements.clear();
    
    for (unsigned short int i = 0; i < num_landmarks; i++) {
        float dx = landmarks[i].x - robot_position.x + get_rand() * measurement_noise;
        float dy = landmarks[i].y - robot_position.y + get_rand() * measurement_noise;
        if (abs(dx) <= measurement_range && abs(dy) <= measurement_range)
            measurements.push_back(Measurement{ i, Position{dx, dy} });
    }
}

bool move(Position &move_position, Position &robot_position){
    float x = robot_position.x + move_position.x + get_rand() * motion_noise;
    float y = robot_position.y + move_position.y + get_rand() * motion_noise;
    
    if(x < 0.0 || x > world_size || y < 0.0 || y > world_size)
        return false;
    else{
        robot_position.x = x;
        robot_position.y = y;
        return true;
    }
}
