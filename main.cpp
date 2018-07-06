#include <vector>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include "Eigen/Dense"
#include "data_structure.h"
#include "robot.h"

using namespace std;
using namespace Eigen;

void make_landmarks(vector<Position> &landmarks);
void make_data(vector<SensorMotionData> &data);
void update_omega_xi(MatrixXf &omega, VectorXf &xi, unsigned short int index1, unsigned short int index2, float omega_update, Position xi_update);
void expand_omega_xi(MatrixXf &omega, VectorXf &xi, MatrixXf &expanded_omega, VectorXf &expanded_xi);
void reduce_omega_xi(MatrixXf &omega, VectorXf &xi, MatrixXf &expanded_omega, VectorXf &expanded_xi);
void slam(vector<SensorMotionData> &data, MatrixXf &omega, VectorXf &xi, VectorXf &mu_online);
void myperformance_test(int iterations);

static const unsigned short int matrix_size = 2 * (1 + num_landmarks);

int main() {
    //myperformance_test(100000);
    
    vector<SensorMotionData> data;
    data.reserve(num_steps);
    make_data(data);
    
    MatrixXf omega = MatrixXf::Zero(matrix_size, matrix_size);
    VectorXf xi = VectorXf::Zero(matrix_size);
    
    omega(0, 0) = 1;
    omega(1, 1) = 1;
    xi(0) = world_size / 2;
    xi(1) = xi[0];
    
    VectorXf mu_online(matrix_size);
    slam(data, omega, xi, mu_online);
    
    cout << mu_online << endl;
    
    return 0;
}

void make_landmarks(vector<Position> &landmarks){
    landmarks.clear();
    
    for (unsigned short int i = 0; i < num_landmarks; i++) {
        landmarks.push_back({ get_rand()*world_size, get_rand()*world_size});
    }
}

void make_data(vector<SensorMotionData> &data){
    unsigned short int i, j = 0;
    float orientation;
    Position move_position;
    float init_position = world_size/2;
    Position robot_position{ init_position,  init_position };
    
    bool seen[num_landmarks];
    
    vector<Position> landmarks;
    landmarks.reserve(num_landmarks);
    
    bool complete = false;
    
    vector<Measurement> measurements;
    measurements.reserve(num_landmarks);
    
    while(!complete) {
        data.clear();
        
        make_landmarks(landmarks);
        for(i = 0; i < num_landmarks; i++){
            seen[i] = false;
        }
        
        orientation = get_rand() * 2.0 * M_PI;
        move_position.x = (float)(move_distance * cos(orientation));
        move_position.y = (float)(move_distance * sin(orientation));
        
        for(i = 0; i < num_steps; i++){
            sense(landmarks, robot_position, measurements);
            for(j = 0; j < measurements.size(); j++)
                seen[measurements[j].landmark_index] = true;
            
            while(!move(move_position, robot_position)){
                orientation = get_rand() * 2.0 * M_PI;
                move_position.x = (float)(move_distance * cos(orientation));
                move_position.y = (float)(move_distance * sin(orientation));
            }
            
            SensorMotionData sensor_motion{measurements, move_position};
            data.push_back(sensor_motion);
        }
        
        complete = true;
        for(i = 0; i < num_landmarks; i++){
            if(!seen[i]){
                complete = false;
                break;
            }
        }
    }
    
    cout << "Original landmark locations:" << "\n";
    for(i = 0; i < num_landmarks; i++){
        cout << landmarks[i].x << "\n";
        cout << landmarks[i].y << "\n";
    }
    cout << "End of original landmark locations:" << "\n";
    
}

void update_omega_xi(MatrixXf &omega, VectorXf &xi, unsigned short int index1, unsigned short int index2, float omega_update, Position xi_update){
    omega(index1, index1) += omega_update;
    omega(index1+1, index1+1) += omega_update;
    omega(index2, index2) += omega_update;
    omega(index2+1, index2+1) += omega_update;
    omega(index1, index2) += -omega_update;
    omega(index1+1, index2+1) += -omega_update;
    omega(index2, index1) += -omega_update;
    omega(index2+1, index1+1) += -omega_update;
    xi(index1) += -xi_update.x;
    xi(index1+1) += -xi_update.y;
    xi(index2) += xi_update.x;
    xi(index2+1) += xi_update.y;
}

void expand_omega_xi(MatrixXf &omega, VectorXf &xi, MatrixXf &expanded_omega, VectorXf &expanded_xi){
    const unsigned short int expanded_matrix_size = matrix_size + 2;
    const unsigned short int sub_block_size = matrix_size - 2;
    expanded_omega = MatrixXf::Zero(expanded_matrix_size, expanded_matrix_size);
    expanded_xi = VectorXf::Zero(expanded_matrix_size);
    
    expanded_omega.topLeftCorner<2, 2>() = omega.topLeftCorner<2, 2>();
    expanded_omega.topRightCorner<2, sub_block_size>() = omega.topRightCorner<2, sub_block_size>();
    expanded_omega.bottomLeftCorner<sub_block_size, 2>() = omega.bottomLeftCorner<sub_block_size, 2>();
    expanded_omega.bottomRightCorner<sub_block_size, sub_block_size>() = omega.bottomRightCorner<sub_block_size, sub_block_size>();
    expanded_xi.head<2>() = xi.head<2>();
    expanded_xi.tail<sub_block_size>() = xi.tail<sub_block_size>();
}

void reduce_omega_xi(MatrixXf &omega, VectorXf &xi, MatrixXf &expanded_omega, VectorXf &expanded_xi){
    MatrixXf omega_A_transpose = expanded_omega.topRightCorner<2, matrix_size>().transpose();
    Matrix2f omega_B_inv = expanded_omega.topLeftCorner<2, 2>().inverse();
    omega = expanded_omega.bottomRightCorner<matrix_size, matrix_size>() - omega_A_transpose * omega_B_inv * expanded_omega.topRightCorner<2, matrix_size>();
    xi = expanded_xi.tail<matrix_size>() - omega_A_transpose * omega_B_inv * expanded_xi.head<2>();
}

void slam(vector<SensorMotionData> &data, MatrixXf &omega, VectorXf &xi, VectorXf &mu_online){
    const unsigned short int extended_matrix_size = matrix_size + 2;
    MatrixXf expanded_omega(extended_matrix_size, extended_matrix_size);
    VectorXf expanded_xi(extended_matrix_size);
    
    float measure_info_strength = 1.0/measurement_noise;
    float motion_info_strength = 1.0/motion_noise;
    
    for(unsigned short int i = 0; i < data.size(); i++){
        for(Measurement measure : data[i].sensor_data){
            update_omega_xi(omega, xi, 0, (measure.landmark_index*2+2), measure_info_strength, Position{measure.measure_distance.x*measure_info_strength, measure.measure_distance.y*measure_info_strength});
        }
        expand_omega_xi(omega, xi, expanded_omega, expanded_xi);
        update_omega_xi(expanded_omega, expanded_xi, 0, 2, motion_info_strength, Position{data[i].motion_data.x*motion_info_strength, data[i].motion_data.y*motion_info_strength});
        reduce_omega_xi(omega, xi, expanded_omega, expanded_xi);
    }
    
    mu_online = omega.inverse() * xi;
    
}

void myperformance_test(int iterations){
    unsigned short int expanded_matrix_size = matrix_size + 2;
    std::clock_t start;
    double duration;
    
    MatrixXf omega = MatrixXf::Zero(matrix_size, matrix_size);
    VectorXf xi = VectorXf::Zero(matrix_size);
    
    MatrixXf expanded_omega = MatrixXf::Zero(expanded_matrix_size, expanded_matrix_size);
    VectorXf expanded_xi = VectorXf::Zero(expanded_matrix_size);
    
    vector<SensorMotionData> data;
    data.reserve(num_steps);
    
    start = std::clock();
    for (int i = 0; i < iterations; i++) {
        reduce_omega_xi(omega, xi, expanded_omega, expanded_xi);
    }
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << duration << endl;
}

