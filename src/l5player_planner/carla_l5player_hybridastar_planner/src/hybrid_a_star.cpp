#include "carla_l5player_hybridastar_planner/hybrid_a_star.h"

using namespace l5player::planner;
using namespace std::chrono_literals;

HybridAstarNode::HybridAstarNode() : Node("hybridastar_search"){
    
    if (!HybridAstarNode::InitialMap()){
        RCLCPP_WARN(this->get_logger(), "Failed to Initial map");
    }

    if (!HybridAstarNode::InitialHybrid()){
        RCLCPP_WARN(this->get_logger(), "Failed to Get Hybrid parameters");
    }

    vehicle_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                    "/carla/ego_vehicle/odometry", 
                                    10, 
                                    std::bind(&HybridAstarNode::VehiclePoseCallback, this, std::placeholders::_1));

}

bool HybridAstarNode::InitialMap(){
    while (!world_param_client_->wait_for_service(1s)){
        RCLCPP_WARN(this->get_logger(), "please open carla town01...");
    }
    map_xu = world_param_client_->get_parameter<double>("map_max_x");
    map_yu = world_param_client_->get_parameter<double>("map_max_y");
    map_xl = world_param_client_->get_parameter<double>("map_min_x");
    map_yl = world_param_client_->get_parameter<double>("map_min_y");
    map_resolution_ = world_param_client_->get_parameter<double>("resolution");

    delta_ds_ = std::sqrt(2) * map_resolution_;

    map_x_size_ = (int)((map_xu - map_xl) / map_resolution_);
    map_y_size_ = (int)((map_yu - map_yl) / map_resolution_);
    map_xy_size_ = map_x_size_ * map_y_size_;

    map_value_ = new uint8_t [map_xy_size_];
    memset(map_value_, 0, map_xy_size_ * sizeof(uint8_t)); // initial the map value

    GridNodeMap = new GridNodePtr * [map_x_size_]; // 二级指针
    for (int i = 0; i < map_x_size_; i++){
        GridNodeMap[i] = new GridNodePtr [map_y_size_];
        for (int j = 0; j < map_y_size_; j++){
            Eigen::Vector2i tmp_index(i,j);
            Eigen::Vector3d tmp_pose((double)(i*map_resolution_), (double)(j*map_resolution_), 0.0);
            GridNodeMap[i][j] = new GridNode(tmp_index, tmp_pose);
        }
    }

    return true;
}

bool HybridAstarNode::InitialHybrid(){
    this->declare_parameter<double>("max_v", max_v_);
    this->declare_parameter<double>("min_v", min_v_);
    this->declare_parameter<double>("max_steering_angle", max_steering_angle_);
    this->declare_parameter<double>("min_steering_angle", min_steering_angle_);
    this->declare_parameter<double>("discrete_num", discrete_num_);

    return true;
}

void HybridAstarNode::VehiclePoseCallback(nav_msgs::msg::Odometry::SharedPtr vehicle_pose_msg){
    double vehicle_x = vehicle_pose_msg->pose.pose.position.x;
    double vehicle_y =  vehicle_pose_msg->pose.pose.position.y;
    double vehicle_roll, vehicle_pitch, vehicle_yaw;
    tf2::Quaternion quat_tf;
    tf2::convert(vehicle_pose_msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);
    vehicle_yaw -= M_PI/2 ;
    while (vehicle_yaw < -M_PI){
        vehicle_yaw += M_2_PI;
    }
    while (vehicle_yaw > M_PI){
        vehicle_yaw -= M_2_PI;
    }

    if (!start_){
        start_pose[0] = vehicle_x;
        start_pose[1] = vehicle_y;
        start_pose[2] = vehicle_yaw;

        start_ = true;
    }
}

Eigen::Vector3d HybridAstarNode::GridIndex2Pose(const Eigen::Vector2i & grid_index){
    int index_x = grid_index[0];
    int index_y = grid_index[1];

    Eigen::Vector3d pose;
    pose[0] = index_x * map_resolution_ + map_xl;
    pose[1] = index_y * map_resolution_ + map_yl;
    pose[2] = 0.0;

    return pose;
}

Eigen::Vector2i HybridAstarNode::Pose2GridIndex(const Eigen::Vector3d & vehicle_pose){
    double x = vehicle_pose[0];
    double y = vehicle_pose[1];
    
    int index_x = (int)(x - map_xl) / map_resolution_;
    int index_y = (int)(y - map_yl) / map_resolution_;

    Eigen::Vector2i grid_index(index_x, index_y);

    return grid_index;
}

// bool ExpandNode(GridNodePtr current_pt, std::vector<GridNodePtr> & neighbor_pt, std::vector<double> & neighbor_costs){
//     Eigen::Vector3d node_pose = current_pt->pose;
    

//     return true;
//     // expand node pose
    
// }

// bool HybridAstarNode::SearchPath(const Eigen::Vector3d & start_pose, const Eigen::Vector3d & end_pose){
//     return true;
// }


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto hybridastar_node = std::make_shared<HybridAstarNode>();
    rclcpp::spin(hybridastar_node);
    rclcpp::shutdown();
    return 0;
}