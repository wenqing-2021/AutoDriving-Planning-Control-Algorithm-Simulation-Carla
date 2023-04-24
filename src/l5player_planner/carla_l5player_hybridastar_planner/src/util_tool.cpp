#include "carla_l5player_hybridastar_planner/util_tool.h"

std::pair<double, double> GetLinearFunc(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2){
    double k = (vertex_2.second - vertex_1.second) / (vertex_1.first - vertex_2.first);
    double b = vertex_2.second - k * vertex_2.first;
    return std::make_pair(k, b);
}

/**
 * @description: generate rotation matrix for two points. The maorigin is the first point.
 * @return {*} the rotation matrix, shape is (2, 2)
 */
Eigen::Matrix2d GetRotationMatrix(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2){
    std::pair<double, double> vector_1(1.0, 0.0);
    std::pair<double, double> vector_2(vertex_2.first - vertex_1.first,
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

double arrangeAngle(double angle){
    while(angle>M_PI) angle -= 2*M_PI;
    while(angle<-M_PI) angle += 2*M_PI;

    return angle;
}

double polar_r(double x, double y){
    return sqrt(x*x + y*y);
}

double polar_theta(double x, double y){
    return atan2(y, x);
}

void calc_tauOmega(double& tau, double& omega, double u , double v, double xi, double eta, double phi){
    double delta = arrangeAngle(u-v);
    double A = sin(u) - sin(delta);
    double B = cos(u) - cos(delta) - 1.0;

    double t1 = atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;

    if(t2 < 0.0){
        tau = arrangeAngle(t1 + M_PI);
    }else{
        tau = arrangeAngle(t1);
    }

    omega = arrangeAngle(tau - u + v - phi);
}
