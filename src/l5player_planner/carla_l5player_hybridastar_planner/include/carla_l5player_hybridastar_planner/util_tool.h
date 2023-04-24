#ifndef HYBRIDASTAR_UTIL_TOOH_H_
#define HYBRIDASTAR_UTIL_TOOH_H_
#pragma once

#include <math.h>
#include <string>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>

std::pair<double, double> GetLinearFunc(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2);

Eigen::Matrix2d GetRotationMatrix(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2);

double GetVectorAngle(const std::pair<double, double>& vector_1, const std::pair<double, double>& vector_2);

// RS_CURVE_USE
double arrangeAngle(double angle);

double polar_r(double x, double y);

double polar_theta(double x, double y);

void calc_tauOmega(double& tau, double& omega, double u , double v, double xi, double eta, double phi);

#endif