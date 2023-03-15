#include "carla_l5player_hybridastar_planner/visualization.h"

using namespace std::chrono_literals;

VisualNode::VisualNode() : Node("hybridastar_visual"){
    world_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "hybridastar_initial_wolrd");
    
    if (VisualNode::ConnectBoundaryServer()){
        VisualNode::GetBoundaryValue();
        RCLCPP_INFO(this->get_logger(), "successfully get boundary value");
    }
    else{
        RCLCPP_WARN(this->get_logger(), "can not connect world, please run hybridastar_initial_wolrd");
    }

    visualpath_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybridastar/path_pub", 10);
    visualboundary_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybridastar/boundary_pub", 10);

    boundary_timer_ = this->create_wall_timer(
        500ms, std::bind(&VisualNode::VisualBoundaryCallback, this));
}

bool VisualNode::ConnectBoundaryServer(){
    while (!world_param_client_->wait_for_service(1s)){
        if (!rclcpp::ok()){
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "connect town01...");
    }

    return true;
}

void VisualNode::GetBoundaryValue(){
    double xu = world_param_client_->get_parameter<double>("map_max_x");
    double yu = world_param_client_->get_parameter<double>("map_max_y");
    double xl = world_param_client_->get_parameter<double>("map_min_x");
    double yl = world_param_client_->get_parameter<double>("map_min_y");
    Eigen::Vector2d point_1(xu, yu);
    Eigen::Vector2d point_2(xu, yl);
    Eigen::Vector2d point_3(xl, yl);
    Eigen::Vector2d point_4(xl, yu);

    boundary_points_.push_back(point_1);
    boundary_points_.push_back(point_2);
    boundary_points_.push_back(point_3);
    boundary_points_.push_back(point_4);

    for (int i =0; i < int(boundary_points_.size()) ; ++i){
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "/map";
        line_strip.header.stamp = this->get_clock()->now();
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.scale.x = 0.2;
        line_strip.id = i;
        line_strip.color.g = 1.0;
        line_strip.color.a = 1.0;
        geometry_msgs::msg::Point p1, p2;

        p1.x = boundary_points_[i](0);
        p1.y = boundary_points_[i](1);

        if (i == int(boundary_points_.size() - 1)){
            p2.x = boundary_points_[0](0);
            p2.y = boundary_points_[0](1);
        }
        else{
            p2.x = boundary_points_[i+1](0);
            p2.y = boundary_points_[i+1](1);
        }

        line_strip.points.push_back(p1);
        line_strip.points.push_back(p2);

        boundary_lines_.markers.push_back(line_strip);
    }
}

void VisualNode::VisualBoundaryCallback(){
    visualboundary_publisher_->publish(boundary_lines_);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto visual_node = std::make_shared<VisualNode>();
    rclcpp::spin(visual_node);
    rclcpp::shutdown();
    return 0;
}