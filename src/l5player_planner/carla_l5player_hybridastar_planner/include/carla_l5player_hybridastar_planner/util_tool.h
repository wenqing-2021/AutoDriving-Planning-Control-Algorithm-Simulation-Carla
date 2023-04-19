#ifndef HYBRIDASTAR_UTIL_TOOH_H_
#define HYBRIDASTAR_UTIL_TOOH_H_


#include <math.h>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>

std::pair<double, double> GetLinearFunc(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2);

Eigen::Matrix2d GetRotationMatrix(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2);

double GetVectorAngle(const std::pair<double, double>& vector_1, const std::pair<double, double>& vector_2);

#endif