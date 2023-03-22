/*
 * @Author: wenqing-2021 1140349586@qq.com
 * @Date: 2023-03-13 09:26:13
 * @LastEditors: wenqing-2021
 * @LastEditTime: 2023-03-20
 * @FilePath: /AutoDriving-Planning-Control-Algorithm-Simulation-Carla/src/l5player_controler/carla_l5player_hybridastar_planner/include/carla_l5player_hybridastar_planner/visualization.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PLANNER_HYBRIDASTAR_VISUAL_H_
#define PLANNER_HYBRIDASTAR_VISUAL_H_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include <math.h>

#include "carla_l5player_hybridastar_planner/node3d.h"
#include "carla_l5player_hybridastar_planner/common.h"

class VisualNode : public rclcpp::Node{
    public:
        VisualNode();
        ~VisualNode(){};

        void VisualBoundaryCallback();
        void VisualVechicleCallback();

        visualization_msgs::msg::MarkerArray GetPolygon(const std::vector<geometry_msgs::msg::Point>& polygon_points);

        void VisualPathCallback(std::vector<GridNode>);

        bool GetBoundaryVertex();
        bool GetVehicleVertex();

        void VehiclePoseCallback(nav_msgs::msg::Odometry::SharedPtr vehicle_msgs);
    
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualboundary_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualpath_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualvehicle_pub_;

        rclcpp::TimerBase::SharedPtr boundary_timer_;
        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::TimerBase::SharedPtr vehicle_timer_;

        rclcpp::SyncParametersClient::SharedPtr world_param_client_;

        visualization_msgs::msg::MarkerArray boundary_lines_;
        visualization_msgs::msg::MarkerArray vehicle_lines_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_pose_sub_;
        VehicleState vehicle_;

        Eigen::Vector2d rl_point;
        Eigen::Vector2d fl_point;
        Eigen::Vector2d fr_point;
        Eigen::Vector2d rr_point;
};

#endif