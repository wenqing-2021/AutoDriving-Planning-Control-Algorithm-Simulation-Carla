#ifndef HYBRIDASTAR_NODE_H_
#define HYBRIDASTAR_NODE_H_

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <float.h>

#define inf DBL_MAX

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode{
    int id; // 1 --> openset, -1 --> closed set
    Eigen::Vector3d pose;
    Eigen::Vector2i index;

    double g_score, f_score;
    GridNodePtr FatherNode;

    std::multimap<double, GridNodePtr>::iterator node_mapit;

    GridNode(Eigen::Vector2i _index, Eigen::Vector3d _pose){
        id = 0;
        index = _index;
        pose = _pose;

        g_score = inf;
        f_score = inf;
        FatherNode = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};

#endif

