#ifndef PLANNER_HYBRIDASTAR_SEARCH_H_
#define PLANNER_HYBRIDASTAR_SEARCH_H_
#pragma once

#include "carla_l5player_hybridastar_planner/node3d.h"
#include "carla_l5player_hybridastar_planner/common.h"
#include "carla_l5player_hybridastar_planner/util_tool.h"
#include "carla_l5player_hybridastar_planner/rs_curve.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
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
        
        bool InitialMap();
        bool InitialHybridAstar();
        bool InitialObstacle();
        void LoadObstacle();
        void SetObstacleData(const std::vector<std::pair<double, double>>& obstacles_vector);
        
        void PI2PI(double & theta);
        bool SearchPath();

        void VehiclePoseCallback(nav_msgs::msg::Odometry::SharedPtr vehicle_pose_msg);
        void GoalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg);

        // pub obstacle position
        void PubObstacleCallback();
        // pub planned path
        void PubPathCallback();
    
    private:
        // bool SetObstacles();
        void ExpandNode(const GridNodePtr & current_pt);

        double ComputeH(const GridNodePtr &node1);
        double ComputeG(const GridNodePtr &node1, const GridNodePtr &node2);

        bool CollisionCheck(const Eigen::Vector3d & current_pose);

        // bool GetResults(std::vector<Eigen::Vector3d> & hybridastar_resutls);

        Eigen::Vector2d GridIndex2Posi(const Eigen::Vector2i & grid_index);
        Eigen::Vector2i Pose2GridIndex(const Eigen::Vector3d & vehicle_pose);
        
        // map configuration
        double map_resolution_;
        int map_x_size_, map_y_size_, map_xy_size_;
        double map_xu, map_yu;
        double map_xl, map_yl;
        GridNodePtr **GridNodeMap;
        uint8_t * map_data; // 0 --> no obstacle, 1 --> has obstacle
        std::vector<std::pair<double, double>> obstacle_position;
        std::vector<Eigen::Vector3d> obstacle_boundary_position;
        std::vector<int> obstalce_vertex_num;

        // vehicle settings
        bool reach_ = false; // if reach the goal, change to true
        bool start_ = false; // if get the start position, change to true. if reach the goal ,change to false 
        double max_steering_angle_;
        double min_steering_angle_;
        double max_v_;
        double min_v_;
        // obstacle data file path
        std::string obstacle_data_path_;

        // hybrid g score computation coefficient
        int steering_num_; // discrete the steering angle
        double delta_ds_; // expand length = sqrt(2) * map_resolution_
        int ds_num_; // split the expand length into pieces for collision check
        double coeffi_heading_;
        double coeffi_gear_;
        double g_gear_;
        double radius_flag_;

        // collision check
        int circle_num_;
        double safe_dis_;

        // rs curve planning
        ReedsSheppPath* rs_planner;

        // final path
        Path final_path;
        
        VehicleState vehicle_;
        Eigen::Vector3d goal_pose_;
        Eigen::Vector3d start_pose_;
        std::multimap<double, GridNodePtr> openset;

        // world client & subscriber
        rclcpp::SyncParametersClient::SharedPtr world_param_client_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_pose_sub_;

        // goal subscriber
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

        // publisher
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_position_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_vertex_num_pub_;
        rclcpp::TimerBase::SharedPtr obstacle_pub_timer_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr path_pub_;
};

} // planner
} // l5player


#endif