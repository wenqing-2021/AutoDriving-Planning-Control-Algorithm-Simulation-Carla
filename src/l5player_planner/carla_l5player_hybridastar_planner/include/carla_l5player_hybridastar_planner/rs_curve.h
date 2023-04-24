#ifndef RS_CURVE_H_
#define RS_CURVE_H_
#pragma once

/*
reference https://github.com/tamago117/ReedsSheppPath/blob/master/include/ReedsSheppPath.hpp
*/

#include <vector>
#include <string>
#include <math.h>
#include <numeric>
#include <limits.h>

#include "carla_l5player_hybridastar_planner/common.h"
#include "carla_l5player_hybridastar_planner/util_tool.h"

class ReedsSheppPath
{
public:
    ReedsSheppPath(){};
    ~ReedsSheppPath(){};

    Path planning(double sx, double sy, double syaw, double gx, double gy, double gyaw, double max_curvature, double step_size);
    void calc_allPath(std::vector<Path>& paths, double sx, double sy, double syaw, double gx, double gy, double gyaw, double max_curvature, double step_size);

private:
    struct path_detail{
        bool flag;
        double t;
        double u;
        double v;
    };

    void generate_allPath(std::vector<Path>& paths, double sx, double sy, double syaw, double gx, double gy, double gyaw, double max_curvature, double step_size);
    void SCS(double x, double y, double phi, std::vector<Path>& paths, double step_size);
    void CSC(double x, double y, double phi, std::vector<Path>& paths, double step_size);
    void CCC(double x, double y, double phi, std::vector<Path>& paths, double step_size);
    void CCCC(double x, double y, double phi, std::vector<Path>& paths, double step_size);
    void CCSC(double x, double y, double phi, std::vector<Path>& paths, double step_size);
    void CCSCC(double x, double y, double phi, std::vector<Path>& paths, double step_size);
    path_detail LSL(double x, double y, double phi);
    path_detail LSR(double x, double y, double phi);
    path_detail LRL(double x, double y, double phi);
    path_detail SLS(double x, double y, double phi);
    path_detail LRLRn(double x, double y, double phi);
    path_detail LRLRp(double x, double y, double phi);
    path_detail LRSR(double x, double y, double phi);
    path_detail LRSL(double x, double y, double phi);
    path_detail LRSLR(double x, double y, double phi);

    void set_path(std::vector<Path>& paths, const std::vector<double>& lengths, const std::vector<std::string>& ctypes, double step_size);
    void generate_local_course(std::vector<double>& px, std::vector<double>& py, std::vector<double>& pyaw, std::vector<bool> directions, const Path& path, double max_curvature, double step_size);
    void interpolate(std::vector<double>& px, std::vector<double>& py, std::vector<double>& pyaw, std::vector<bool>& directions,
                     int ind, double length, std::string mode, double maxc, double ox, double oy, double oyaw);
};

#endif