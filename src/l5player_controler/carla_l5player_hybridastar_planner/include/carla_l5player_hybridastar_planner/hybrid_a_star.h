#ifndef PLANNER_HYBRIDASTAR_SEARCH_H_
#define PLANNER_HYBRIDASTAR_SEARCH_H_

#include "carla_l5player_hybridastar_planner/node3d.h"
#include "carla_l5player_hybridastar_planner/common.h"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <string>
#include <algorithm>
#include <vector>

namespace l5player{
namespace planner{

class HybridAstarNode : public rclcpp :: Node {
    /*
     * note: The carla coordinate rotates clockwise by 90 degree
    */
    public:
        HybridAstarNode();
        ~HybridAstarNode();
        
        bool SearchPath(const Eigen::Vector3d & start_pt, 
                    const Eigen::Vector3d & end_pt);
        
        bool InitialMap();
        bool InitialHybrid();
        void VehiclePoseCallback(nav_msgs::msg::Odometry::SharedPtr vehicle_pose_msg);
    
    private:
        bool SetObstacles();
        bool ExpandNode(GridNodePtr current_pt, 
                        std::vector<GridNodePtr> & neighbor_pt,
                        std::vector<double> & neighbor_costs);

        double ComputeH(GridNodePtr node1, GridNodePtr node2);

        bool CollisionCheck(const GridNodePtr currentPtr);

        bool GetResults(std::vector<Eigen::Vector3d> & hybridastar_resutls);

        Eigen::Vector3d GridIndex2Pose(const Eigen::Vector2i & grid_index);
        Eigen::Vector2i Pose2GridIndex(const Eigen::Vector3d & vehicle_pose);
        
        // map configuration
        double map_resolution_;
        int map_x_size_, map_y_size_, map_xy_size_;
        double map_xu, map_yu;
        double map_xl, map_yl;
        uint8_t *map_value_;
        GridNodePtr **GridNodeMap;

        // hybrid settings
        bool reach_ = false; // if reach the flag, change to true
        bool start_ = false; // if get the start position, change to true. if reach the goal ,change to false 
        double max_steering_angle_;
        double min_steering_angle_;
        int discrete_num_; // should be a 
        double max_v_;
        double min_v_;

        double delta_ds_; // expand 
        
        VehicleState vehicle_;
        Eigen::Vector3d goal_pose;
        Eigen::Vector3d start_pose;
        std::multimap<double, GridNodePtr> openset;

        rclcpp::SyncParametersClient::SharedPtr world_param_client_;
        rclcpp::SyncParametersClient::SharedPtr vehicle_param_client_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_pose_sub_;
};

} // planner
} // l5player


#endif