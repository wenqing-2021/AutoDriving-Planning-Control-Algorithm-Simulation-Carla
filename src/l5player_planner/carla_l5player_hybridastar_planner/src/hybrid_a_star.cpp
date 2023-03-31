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
    PI2PI(vehicle_yaw);

    if (!start_){
        start_pose[0] = vehicle_x;
        start_pose[1] = vehicle_y;
        start_pose[2] = vehicle_yaw;

        start_ = true;
    }
}

void HybridAstarNode::PI2PI(double & theta){
    while (theta < -M_PI){
        theta += M_2_PI;
    }
    while (theta > M_PI){
        theta -= M_2_PI;
    }
}

Eigen::Vector3d HybridAstarNode::GridIndex2Pose(const Eigen::Vector2i & grid_index){
    int index_x = grid_index[0];
    int index_y = grid_index[1];

    Eigen::Vector3d pose(index_x * map_resolution_ + map_xl,
                         index_y * map_resolution_ + map_yl,
                         0.0);

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

bool HybridAstarNode::CollisionCheck(const Eigen::Vector3d & current_pose){
    bool collision = false;

    Eigen::MatrixXd circle_position(circle_num, 2);

    // we use fitted circles for collision check
    double radius = 0.5 * sqrt(pow(vehicle_.length / circle_num, 2) + pow(vehicle_.width, 2));
    for (int i = 0; i < circle_num; ++i){
        double coefficient = (vehicle_.length - vehicle_.wheelbase) / 2 + vehicle_.wheelbase + vehicle_.length / circle_num * (0.5 - i);
        circle_position[i,0] = vehicle_.x + coefficient * cos(current_pose[2]);
        circle_position[i,1] = vehicle_.y + coefficient * sin(current_pose[2]);
    }

    return collision;
}

double HybridAstarNode::ComputeH(const GridNodePtr &node1, const GridNodePtr &node2){
    double distance;

    // use manhattan distance
    Eigen::Vector2i index_1 = node1->index;
    Eigen::Vector2i index_2 = node2->index;

    distance = std::sqrt(pow((index_1[0] - index_2[0]), 2) + pow((index_1[1] - index_2[1]), 2));

    return distance;
}

double HybridAstarNode::ComputeG(const GridNodePtr &node1, const GridNodePtr &node2){
    double g_value;
    double cost_gear;
    double cost_heading;

    Eigen::Vector3d pose_1 = node1->pose;
    Eigen::Vector3d pose_2 = node2->pose;

    cost_heading = coeffi_heading * abs(pose_2[2] - pose_1[1]);
    if (node2->gear != node1->gear){
        cost_gear = coeffi_gear * g_gear;
    }
    else{
        cost_gear = 0.0;
    }

    g_value = cost_heading + cost_gear;

    return node1->g_score + g_value;
}


bool HybridAstarNode::ExpandNode(const GridNodePtr & current_pt){
    Eigen::Vector3d current_pose = current_pt->pose;
    int gear[] = {1, -1};
    double delta_angle = (max_steering_angle_ - min_steering_angle_) / (discrete_num_ - 1);
    // expand node
    for (auto gear_i : gear){
        double ds = delta_ds_ * gear_i;
        for (int i = 0; i < discrete_num_; ++i){
            double steering_angle_i = min_steering_angle_ + i * delta_angle;
            double heading_i = current_pose[2] + ds * tan(steering_angle_i) / vehicle_.wheelbase;
            PI2PI(heading_i);
            double x_i = current_pose[0] + ds * cos(heading_i);
            double y_i = current_pose[1] + ds * sin(heading_i);
            // CHECK this node is in the map or not
            if ((x_i > map_xu) || (x_i < map_xl) || (y_i > map_yu) || (y_i < map_yl)){
                continue;
            }
            // CHECK this node has been colosed or not
            Eigen::Vector3d node_pose(x_i, y_i, heading_i); 
            Eigen::Vector2i node_index = Pose2GridIndex(node_pose);
            GridNodePtr node_i = GridNodeMap[node_index[0]][node_index[1]];
            if (node_i->id == -1){
                continue;
            }
            // CHECK is collision on this path or not
            bool collision = false;
            double dds = ds / ds_num_;
            for (int j = 1; j < (ds_num_+1); ++j){
                double heading_j = current_pose[2] + dds * j * tan(steering_angle_i) / vehicle_.wheelbase;
                PI2PI(heading_i);
                double x_j = current_pose[0] + dds * j * cos(heading_j);
                double y_j = current_pose[1] + dds * j * cos(heading_j);
                Eigen::Vector3d pose_j(x_j, y_j, heading_j);
                collision = HybridAstarNode::CollisionCheck(pose_j);
                if (collision){
                    break;
                }
            }
            if (!collision){
                // if this node has been visited
                if (node_i->id == 1){
                    // compute g
                    double node_i_g = ComputeG(current_pt, node_i);
                    
                    // change father node
                    if (node_i_g < node_i->g_score){
                        node_i->g_score = node_i_g;
                        node_i->f_score = node_i->f_score - node_i->g_score + node_i_g;
                        node_i->FatherNode = current_pt;
                        node_i->pose = current_pose;
                        node_i->gear = gear_i;
                    }
                }
                // if this node is first visited
                else{
                    // add it into the openset
                    node_i->id = 1;
                    node_i->g_score = ComputeG(current_pt, node_i);
                    node_i->f_score = node_i->g_score + ComputeH(current_pt, node_i);
                    node_i->FatherNode = current_pt;
                    node_i->pose = current_pose;
                    node_i->gear = gear_i;
                }
            }
        }
    }
    

    return true;
    
}

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