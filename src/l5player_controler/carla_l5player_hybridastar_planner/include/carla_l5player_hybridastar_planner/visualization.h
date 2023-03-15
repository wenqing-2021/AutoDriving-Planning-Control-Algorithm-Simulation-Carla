#ifndef PLANNER_HYBRIDASTAR_VISUAL_H_
#define PLANNER_HYBRIDASTAR_VISUAL_H_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <memory>
#include <chrono>
#include <string>

#include "carla_l5player_hybridastar_planner/node3d.h"

class VisualNode : public rclcpp::Node{
    public:
        VisualNode();
        ~VisualNode(){};

        void VisualBoundaryCallback();

        void VisualPathCallback(std::vector<GridNode>);

        bool ConnectBoundaryServer();
        void GetBoundaryValue();

        std::vector<Eigen::Vector2d> GetBoundary(){ return boundary_points_;};
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualboundary_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualpath_publisher_;

        rclcpp::TimerBase::SharedPtr boundary_timer_;
        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::SyncParametersClient::SharedPtr world_param_client_;

        std::vector<Eigen::Vector2d> boundary_points_;
        visualization_msgs::msg::MarkerArray boundary_lines_;
        
        double xu, yu;
        double xl, yl;
        
};



#endif