#ifndef PLANNER_HYBRIDASTAR_VISUAL_H_
#define PLANNER_HYBRIDASTAR_VISUAL_H_
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include <math.h>
#include <numeric>

#include "carla_l5player_hybridastar_planner/node3d.h"
#include "carla_l5player_hybridastar_planner/common.h"

class VisualNode : public rclcpp::Node{
    public:
        VisualNode();
        ~VisualNode(){};

        void VisualBoundaryCallback();
        void VisualVechicleCallback();
        void VisualObstacleCallback();
        void VisualPathCallback();

        visualization_msgs::msg::MarkerArray GetPolygon(const std::vector<geometry_msgs::msg::Point>& polygon_points);

        bool GetBoundaryVertex();
        bool GetVehicleVertex();

        void VehiclePoseCallback(nav_msgs::msg::Odometry::SharedPtr vehicle_msgs);
        void ObstaclePosCallback(std_msgs::msg::Float64MultiArray::SharedPtr obstacle_position_msgs);
        void ObsVertexNumCallback(std_msgs::msg::Int32MultiArray::SharedPtr obstacle_vertex_num_msgs);
        void AstarPathCallback(Path astar_path_msgs);
    
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualboundary_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualpath_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualvehicle_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualobstacle_pub_;

        rclcpp::TimerBase::SharedPtr boundary_timer_;
        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::TimerBase::SharedPtr vehicle_timer_;
        rclcpp::TimerBase::SharedPtr obstacle_timer_;

        rclcpp::SyncParametersClient::SharedPtr world_param_client_;

        visualization_msgs::msg::MarkerArray boundary_lines_;
        visualization_msgs::msg::MarkerArray vehicle_lines_;
        visualization_msgs::msg::MarkerArray obstacle_lines_;
        std::vector<visualization_msgs::msg::MarkerArray> obstacle_lines_vector;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obstacle_position_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_vertex_num_sub_;
        rclcpp::Subscription<Path>::SharedPtr astar_path_sub_;

        VehicleState vehicle_;

        Eigen::Vector2d rl_point;
        Eigen::Vector2d fl_point;
        Eigen::Vector2d fr_point;
        Eigen::Vector2d rr_point;

        std::vector<int> vertex_num_vector;
};

#endif