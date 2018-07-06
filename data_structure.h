#pragma once

#ifndef DATA_STRUCTURE_H
#define DATA_STRUCTURE_H

#include <vector>

struct Position {
	float x, y;
};

struct Measurement {
	unsigned short int landmark_index;
	Position measure_distance;
};

struct SensorMotionData {
    std::vector<Measurement> sensor_data;
    Position motion_data;
};

static const unsigned short int num_landmarks = 5;
static unsigned short int num_steps = 20;
static float world_size = 100;

static float measurement_range = 50.0;
static float motion_noise = 2.0;
static float measurement_noise = 2.0;
static float move_distance = 20.0;

float get_rand();

#endif
