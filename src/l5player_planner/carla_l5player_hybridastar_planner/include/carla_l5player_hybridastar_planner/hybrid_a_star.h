#ifndef PLANNER_HYBRIDASTAR_SEARCH_H_
#define PLANNER_HYBRIDASTAR_SEARCH_H_

#include "carla_l5player_hybridastar_planner/node3d.h"
#include "carla_l5player_hybridastar_planner/common.h"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <string>
#include <algorithm>
#include <vector>
#include <math.h>

namespace l5player{
namespace planner{

class HybridAstarNode : public rclcpp :: Node {
    /*
     * note: The carla coordinate rotates clockwise by 90 degree
    */
    public:
        HybridAstarNode();
        ~HybridAstarNode(){};
        
        bool SearchPath(const Eigen::Vector3d & start_pt, 
                        const Eigen::Vector3d & end_pt);
        
        bool InitialMap();
        bool InitialHybrid();
        void VehiclePoseCallback(nav_msgs::msg::Odometry::SharedPtr vehicle_pose_msg);
        void PI2PI(double & theta);
    
    private:
        bool SetObstacles();
        bool ExpandNode(const GridNodePtr & current_pt);

        double ComputeH(const GridNodePtr &node1, const GridNodePtr &node2);
        double ComputeG(const GridNodePtr &node1, const GridNodePtr &node2);

        bool CollisionCheck(const Eigen::Vector3d & current_pose);

        bool GetResults(std::vector<Eigen::Vector3d> & hybridastar_resutls);

        Eigen::Vector3d GridIndex2Pose(const Eigen::Vector2i & grid_index);
        Eigen::Vector2i Pose2GridIndex(const Eigen::Vector3d & vehicle_pose);
        
        // map configuration
        double map_resolution_;
        int map_x_size_, map_y_size_, map_xy_size_;
        double map_xu, map_yu;
        double map_xl, map_yl;
        GridNodePtr **GridNodeMap;

        // hybrid settings
        bool reach_ = false; // if reach the flag, change to true
        bool start_ = false; // if get the start position, change to true. if reach the goal ,change to false 
        double max_steering_angle_;
        double min_steering_angle_;
        int discrete_num_; // 
        double max_v_;
        double min_v_;

        // hybrid g score computation coefficient
        double coeffi_heading;
        double coeffi_gear;
        double g_gear;

        double delta_ds_; // expand length
        int ds_num_; // split the expand length into pieces for collision check

        // collision check
        int circle_num;
        double safe_dis;
        
        VehicleState vehicle_;
        Eigen::Vector3d goal_pose;
        Eigen::Vector3d start_pose;
        std::multimap<double, GridNodePtr> openset;

        // world client
        rclcpp::SyncParametersClient::SharedPtr world_param_client_;
        rclcpp::SyncParametersClient::SharedPtr vehicle_param_client_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_pose_sub_;
};

} // planner
} // l5player


#endif