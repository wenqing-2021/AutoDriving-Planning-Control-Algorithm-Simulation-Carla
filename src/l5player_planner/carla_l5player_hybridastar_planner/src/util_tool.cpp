#include "carla_l5player_hybridastar_planner/util_tool.h"

using namespace std;

pair<double, double> GetLinearFunc(const pair<double, double>& vertex_1, const pair<double, double>& vertex_2){
    double k = (vertex_2.second - vertex_1.second) / (vertex_1.first - vertex_2.first);
    double b = vertex_2.second - k * vertex_2.first;
    return std::make_pair(k, b);
}

/**
 * @description: generate rotation matrix for two points. The maorigin is the first point.
 * @return {*} the rotation matrix, shape is (2, 2)
 */
Eigen::Matrix2d GetRotationMatrix(const pair<double, double>& vertex_1, const pair<double, double>& vertex_2){
    pair<double, double> vector_1(1.0, 0.0);
    pair<double, double> vector_2(vertex_2.first - vertex_1.first,
                                  vertex_2.second - vertex_1.second);
    double rotation_angle = GetVectorAngle(vector_1, vector_2);
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix.row(0) << cos(rotation_angle), -sin(rotation_angle);
    rotation_matrix.row(1) << sin(rotation_angle), cos(rotation_angle);
    
    return rotation_matrix;
}

/**
 * @description: get the angle between two vectors
 * @return {*} the angle (rad)
 */
double GetVectorAngle(const std::pair<double, double>& vector_1, const std::pair<double, double>& vector_2){
    double angle = atan2(vector_1.second, vector_1.first) - atan2(vector_2.second, vector_2.first);

    while (angle > M_PI){
        angle -= M_2_PI;
    }
    while (angle < -M_PI){
        angle += M_2_PI;
    }
    return angle;
    
}

