#include "carla_l5player_hybridastar_planner/visualization.h"

using namespace std::chrono_literals;
using namespace visualization_msgs::msg;

VisualNode::VisualNode() : Node("hybridastar_visual"){
    world_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "hybridastar_initial_wolrd");
    if (VisualNode::GetBoundaryVertex()){
        RCLCPP_INFO(this->get_logger(), "get boundary value");
    }
    else{
        RCLCPP_WARN(this->get_logger(), "can not get boundary, please check getboundaryvertex...");
    }

    vehicle_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                    "/carla/ego_vehicle/odometry", 
                                    10, 
                                    std::bind(&VisualNode::VehiclePoseCallback, this, std::placeholders::_1));
    
    obstacle_vertex_num_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
                                    "/hybridastar/search/obstacl_vertex_num_pub", 
                                    10,
                                    std::bind(&VisualNode::ObsVertexNumCallback, this, std::placeholders::_1));
    
    obstacle_position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                                    "/hybridastar/search/obstacl_pos_pub", 
                                    10, 
                                    std::bind(&VisualNode::ObstaclePosCallback, this, std::placeholders::_1));

    visualpath_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybridastar/visual/path_pub", 10);
    visualboundary_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybridastar/visual/boundary_pub", 10);
    visualvehicle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybridastar/visual/vehicle_pub", 10);
    visualobstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybridastar/visual/obstacle_pub", 10);

    boundary_timer_ = this->create_wall_timer(
        500ms, std::bind(&VisualNode::VisualBoundaryCallback, this));
    
    vehicle_timer_ = this->create_wall_timer(
        10ms, std::bind(&VisualNode::VisualVechicleCallback, this));

    obstacle_timer_ = this->create_wall_timer(
        10ms, std::bind(&VisualNode::VisualObstacleCallback, this));
}

bool VisualNode::GetBoundaryVertex(){
    while (!world_param_client_->wait_for_service(1s)){
        RCLCPP_INFO(this->get_logger(), "please connect town01...");
    }
    std::vector<double> map_x;
    std::vector<double> map_y;
    double map_xu = world_param_client_->get_parameter<double>("map_max_x");
    double map_yu = world_param_client_->get_parameter<double>("map_max_y");
    double map_xl = world_param_client_->get_parameter<double>("map_min_x");
    double map_yl = world_param_client_->get_parameter<double>("map_min_y");
    
    map_x.push_back(map_xu);
    map_x.push_back(map_xl);
    map_y.push_back(map_yu);
    map_y.push_back(map_yl);
    std::vector<geometry_msgs::msg::Point> points;
    for (int i = 0; i < 2; ++i){
        for (int j = 0; j < 2; ++j){
            geometry_msgs::msg::Point point;
            point.x = map_x[i];
            point.y = map_y[j];
            points.push_back(point);
        }
        std::reverse(map_y.begin(), map_y.end());
    }
    boundary_lines_ = VisualNode::GetPolygon(points);
    return true;
}

/**
 * @description: get the vehicle rectangle vertex. 
 * the vertex is stored in the vector, and the order is
 * rear left (rl), front left (fl), front right (fr), rear right (rr). 
 * @return Eigen::MatrixXd, shape:[2, 4], rear left (rl), fl, fr, rr
 */
bool VisualNode::GetVehicleVertex(){
    // assume the vehicle position is at the center of the rectangle
    Eigen::MatrixXd vehicle_position(2, 4);
    vehicle_position << vehicle_.x, vehicle_.x, vehicle_.x, vehicle_.x,
                        vehicle_.y, vehicle_.y, vehicle_.y, vehicle_.y;

    Eigen::Matrix2d rotation_matrix;
    rotation_matrix.row(0) << cos(vehicle_.yaw), sin(vehicle_.yaw);
    rotation_matrix.row(1) << -sin(vehicle_.yaw), cos(vehicle_.yaw);

    // in the vehicle body coordinate
    // rl_point(-vehicle_.width/2, -vehicle_.length/2); 
    // fl_point(-vehicle_.width/2, vehicle_.length/2);
    // fr_point(vehicle_.width/2, vehicle_.length/2);
    // rr_point(vehicle_.width/2, -vehicle_.length/2);
    Eigen::MatrixXd vehi_vertex_p(4,2);
    vehi_vertex_p << -vehicle_.width/2, -vehicle_.length/2,
                     -vehicle_.width/2, vehicle_.length/2,
                     vehicle_.width/2, vehicle_.length/2,
                     vehicle_.width/2, -vehicle_.length/2;

    Eigen::Matrix2d inv_rotation_matrix = rotation_matrix.inverse();
    vehi_vertex_p.transposeInPlace();

    Eigen::MatrixXd vehicle_vertex = (inv_rotation_matrix * vehi_vertex_p) + vehicle_position; // shape:[2,4]
    std::vector<geometry_msgs::msg::Point> vehicle_points;
    // std::cout << "row num: " << vehicle_vertex.rows() << std::endl;
    // std::cout << "col num: " << vehicle_vertex.cols() << std::endl;
    for (int i = 0; i < 4; ++i){
        geometry_msgs::msg::Point point;
        point.x = vehicle_vertex(0,i);
        point.y = vehicle_vertex(1,i);
        vehicle_points.push_back(point);
    }
    vehicle_lines_ = VisualNode::GetPolygon(vehicle_points);
    return true;
}

void VisualNode::VehiclePoseCallback(nav_msgs::msg::Odometry::SharedPtr vehicle_pose_msg){
    vehicle_.x =  vehicle_pose_msg->pose.pose.position.x;
    vehicle_.y =  vehicle_pose_msg->pose.pose.position.y;

    tf2::Quaternion quat_tf;
    tf2::convert(vehicle_pose_msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicle_.roll, vehicle_.pitch, vehicle_.yaw);
    vehicle_.yaw -= M_PI/2 ;
    while (vehicle_.yaw < -M_PI){
        vehicle_.yaw += M_2_PI;
    }
    while (vehicle_.yaw > M_PI){
        vehicle_.yaw -= M_2_PI;
    }

    if (!VisualNode::GetVehicleVertex()){
        RCLCPP_WARN(this->get_logger(), "failed to get vehicle vertex.");
    }
}

void VisualNode::ObstaclePosCallback(std_msgs::msg::Float64MultiArray::SharedPtr obstacle_position_msgs){
    std::vector<double> position_data = obstacle_position_msgs->data;
    int start = 0;
    std::vector<geometry_msgs::msg::Point> obstacle_points_vector;
    for (auto vertex_num : vertex_num_vector){
        int end = start + vertex_num;
        for (int index = start; index < end; index++){
            geometry_msgs::msg::Point point_i;
            point_i.x = position_data[index*2];
            point_i.y = position_data[index*2+1];
            obstacle_points_vector.push_back(point_i);
        }
        start = end;
        obstacle_lines_ = GetPolygon(obstacle_points_vector);
        int index = 0;
        for (auto& each_line : obstacle_lines_.markers){
            each_line.id = start - index;
            index++;
        }
        obstacle_lines_vector.push_back(obstacle_lines_);
        obstacle_points_vector.clear();
    }
}

void VisualNode::ObsVertexNumCallback(std_msgs::msg::Int32MultiArray::SharedPtr obstacle_vertex_num_msgs){
    vertex_num_vector = obstacle_vertex_num_msgs->data;
}

void VisualNode::VisualBoundaryCallback(){
    visualboundary_pub_->publish(boundary_lines_);
}

void VisualNode::VisualVechicleCallback(){
    visualvehicle_pub_->publish(vehicle_lines_);
}

void VisualNode::VisualObstacleCallback(){
    visualization_msgs::msg::MarkerArray lines_pub;
    for (auto obstacle_line : obstacle_lines_vector){
        for (auto line_i : obstacle_line.markers){
            lines_pub.markers.push_back(line_i);
        }
    }
    visualobstacle_pub_->publish(lines_pub);
}

MarkerArray VisualNode::GetPolygon(const std::vector<geometry_msgs::msg::Point>& polygon_points){
    MarkerArray result_lines_;
    for(int i = 0; i < int(polygon_points.size()); i++){
        Marker line_strip;
        line_strip.header.frame_id = "/map";
        line_strip.header.stamp = this->get_clock()->now();
        line_strip.id = i;
        line_strip.ns = "lines";
        line_strip.type = Marker::LINE_STRIP;
        line_strip.action = Marker::ADD;
        line_strip.scale.x = 0.2;
        line_strip.color.g = 1.0;
        line_strip.color.a = 1.0;
        if ( i == (int(polygon_points.size() - 1))){
            line_strip.points.push_back(polygon_points[i]);
            line_strip.points.push_back(polygon_points[0]);
        }
        else{
            line_strip.points.push_back(polygon_points[i]);
            line_strip.points.push_back(polygon_points[i+1]);
        }
        
        result_lines_.markers.push_back(line_strip);
    }
    return result_lines_;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto visual_node = std::make_shared<VisualNode>();
    rclcpp::spin(visual_node);
    rclcpp::shutdown();
    return 0;
}