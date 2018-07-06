//
//  robot.h
//  GraphSLAM
//
//  Created by XUXIA YANG on 7/5/18.
//  Copyright © 2018 XUXIA YANG. All rights reserved.
//

#ifndef ROBOT_H
#define ROBOT_H

#include "data_structure.h"

void sense(std::vector<Position> &landmarks, Position &robot_position, std::vector<Measurement> &measurements);
bool move(Position &move_position, Position &robot_position);

#endif /* ROBOT_H */
