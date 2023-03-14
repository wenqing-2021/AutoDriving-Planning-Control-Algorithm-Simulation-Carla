#ifndef PLANNER_HYBRIDASTAR_SEARCH_H_
#define PLANNER_HYBRIDASTAR_SEARCH_H_

#include "carla_l5player_hybridastar_planner/node3d.h"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <string>
#include <algorithm>
#include <vector>

namespace l5player{
namespace planner{

class HybridAstarNode : public rclcpp :: Node {
    public:
        HybridAstarNode();
        ~HybridAstarNode();
        
        bool Search(Eigen::Vector3d start_pt, 
                    Eigen::Vector3d end_pt);
        
        void Plan();
    
    private:
        bool InitialMap();
        
        bool Expand(GridNodePtr current_pt, 
                    std::vector<GridNodePtr> & neighbor_pt,
                    std::vector<double> & neighbor_costs);

        double GetHeu(GridNodePtr node1, GridNodePtr node2);

        bool CollisionCheck(const GridNodePtr currentPtr);
        // bool RSCheck()

        bool GetResults(Eigen::Vector3d & hybridastar_resutls);

        double resolution_;
        int map_x_size_, map_y_size_, map_xy_size_;

        double map_xu_, map_yu_;
        double map_xl_, map_yl_;

        double delta_t_ = 0.0;
        GridNodePtr **GridNodeMap;
        Eigen::Vector2i goal_index;

};

} // planner
} // l5player


#endif