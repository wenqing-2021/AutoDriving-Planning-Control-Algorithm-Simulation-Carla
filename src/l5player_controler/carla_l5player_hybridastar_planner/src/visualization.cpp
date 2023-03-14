#include "carla_l5player_hybridastar_planner/visualization.h"

using namespace std::chrono_literals;

VisualNode::VisualNode() : Node("visual_node"){
    visualpath_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_pub", 10);
    visualboundary_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/boundary_pub", 10);
    Eigen::Vector2d point_1(this->get_parameter("map_max_x"), this->get_parameter("map_max_y"));
    Eigen::Vector2d point_2(this->get_parameter("map_max_x"), this->get_parameter("map_min_y"));
    Eigen::Vector2d point_3(this->get_parameter("map_min_x"), this->get_parameter("map_min_y"));
    Eigen::Vector2d point_4(this->get_parameter("map_min_x"), this->get_parameter("map_max_y"));
    boundary_points_.push_back(point_1);
    boundary_points_.push_back(point_2);
    boundary_points_.push_back(point_3);
    boundary_points_.push_back(point_4);

    for (int i =0; i < boundary_points_.size(); ++i){
        visualization_msgs::msg::Marker line_srip;
        line_srip.header.frame_id = "/map";
        line_srip.header.stamp = this->get_clock()->now();
        line_srip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_srip.scale.x = 0.2;
        line_srip.id = i;
        geometry_msgs::msg::Point p1, p2;

        p1.x = boundary_points_[i](0);
        p1.y = boundary_points_[i](1);

        if (i == (boundary_points_.size() - 1)){
            p2.x = boundary_points_[0](0);
            p2.y = boundary_points_[0](1);
        }
        else{
            p2.x = boundary_points_[i+1](0);
            p2.y = boundary_points_[i+1](1);
        }

        line_srip.points.push_back(p1);
        line_srip.points.push_back(p2);

        lines_.markers.push_back(line_srip);
    }

    boundary_timer_ = this->create_wall_timer(
        500ms, std::bind(&VisualNode::VisualBoundaryCallback, this));
}

void VisualNode::VisualBoundaryCallback(){
    visualboundary_publisher_->publish(lines_);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto visual_node = std::make_shared<VisualNode>();
    rclcpp::spin(visual_node);
    rclcpp::shutdown();
    return 0;
}