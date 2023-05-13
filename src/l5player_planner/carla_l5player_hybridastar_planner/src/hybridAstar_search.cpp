#include "carla_l5player_hybridastar_planner/hybridAstar_search.h"

using namespace l5player::planner;
using namespace std::chrono_literals;

HybridAstarNode::HybridAstarNode() : Node("hybridastar_search"){
    
    if (!HybridAstarNode::InitialMap()){
        RCLCPP_WARN(this->get_logger(), "Failed to Initial map");
    }

    if (!HybridAstarNode::InitialHybridAstar()){
        RCLCPP_WARN(this->get_logger(), "Failed to Get Hybrid parameters");
    }

    if (!HybridAstarNode::InitialObstacle()){
        RCLCPP_WARN(this->get_logger(), "Failed to Get Obstacle parameters");
    }    

    // update vehicle pose
    vehicle_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                    "/carla/ego_vehicle/odometry", 
                                    1, 
                                    std::bind(&HybridAstarNode::VehiclePoseCallback, this, std::placeholders::_1));

    // update goal pose
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                                    "/goal_pose",
                                    1,
                                    std::bind(&HybridAstarNode::GoalPoseCallback, this, std::placeholders::_1));
    
    obstacle_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/hybridastar/search/obstacl_pos_pub", 10);
    obstacle_vertex_num_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/hybridastar/search/obstacl_vertex_num_pub", 10);
    path_pub_ = this->create_publisher<Path>("/hybridastar/search/astar_path", 10);

    obstacle_pub_timer_ = this->create_wall_timer(
        10ms, std::bind(&HybridAstarNode::PubObstacleCallback, this));

}

bool HybridAstarNode::InitialMap(){
    world_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "hybridastar_initial_wolrd");
    while (!world_param_client_->wait_for_service(1s)){
        RCLCPP_INFO(this->get_logger(), "please connect town01...");
    }
    map_xu = world_param_client_->get_parameter<double>("map_max_x");
    map_yu = world_param_client_->get_parameter<double>("map_max_y");
    map_xl = world_param_client_->get_parameter<double>("map_min_x");
    map_yl = world_param_client_->get_parameter<double>("map_min_y");
    map_resolution_ = world_param_client_->get_parameter<double>("resolution");

    RCLCPP_INFO(this->get_logger(), "map_max_x %f", this->map_xu);
    RCLCPP_INFO(this->get_logger(), "map_max_y %f", this->map_yu);
    RCLCPP_INFO(this->get_logger(), "map_min_x %f", this->map_xl);
    RCLCPP_INFO(this->get_logger(), "map_min_y %f", this->map_yl);
    RCLCPP_INFO(this->get_logger(), "resolution %f", this->map_resolution_);
    std::cout << "get map information" << std::endl;

    delta_ds_ = std::sqrt(2) * map_resolution_;

    map_x_size_ = (int)((map_xu - map_xl) / map_resolution_);
    map_y_size_ = (int)((map_yu - map_yl) / map_resolution_);
    map_xy_size_ = map_x_size_ * map_y_size_;

    map_data = new uint8_t[map_xy_size_];
    memset(map_data, 0, map_xy_size_ * sizeof(uint8_t));

    GridNodeMap = new GridNodePtr* [map_x_size_]; // 二级指针
    RCLCPP_INFO(this->get_logger(), "Start initial map ....");
    for (int i = 0; i < map_x_size_; i++){
        GridNodeMap[i] = new GridNodePtr [map_y_size_];
        for (int j = 0; j < map_y_size_; j++){
            Eigen::Vector2i tmp_index(i,j);
            Eigen::Vector3d tmp_pose((double)(i*map_resolution_), (double)(j*map_resolution_), 0.0);
            GridNodeMap[i][j] = new GridNode(tmp_index, tmp_pose);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully initial Map done");
    return true;
}

bool HybridAstarNode::InitialObstacle(){
    this->declare_parameter<std::string>("obstacle_path", "obstacle_inital_path");
    this->get_parameter<std::string>("obstacle_path", obstacle_data_path_);
    std::cout << "obstacle_data_file_path is: " << obstacle_data_path_ << std::endl;
    RCLCPP_INFO(this->get_logger(), "Start initial Obstacles position ... ");
    LoadObstacle();

    RCLCPP_INFO(this->get_logger(), "Successfully get Obstacles position done");
    return true;
}

void HybridAstarNode::LoadObstacle(){
    std::ifstream infile(obstacle_data_path_, std::ios::in);
    assert(infile.is_open());
    std::string line;
    std::vector<std::pair<double, double>> obstacle_i_vector;
    while (getline(infile, line)){
        if (line[0] == '#'){
            continue;
        }
        std::stringstream ss(line);
        std::string obstacle_pos_str;
        std::vector<std::string> tmp_position_str;
        obstacle_i_vector.clear();
        // read data in each row
        int i = 0;
        bool get_vertex_num = false;
        while (getline(ss, obstacle_pos_str, ',')){
            if (!get_vertex_num){
                // store the vertex number of each obstacle
                obstalce_vertex_num.push_back(std::atoi(obstacle_pos_str.c_str()));
                get_vertex_num = true;
                continue;
            }
            tmp_position_str.push_back(obstacle_pos_str);
            i++;
            if ( i % 2 == 0){
                // store the obstacle position in the vector
                double pt_x = std::atof(tmp_position_str[i-2].c_str());
                double pt_y = std::atof(tmp_position_str[i-1].c_str());
                obstacle_position.push_back(std::make_pair(pt_x, pt_y));
                obstacle_i_vector.push_back(std::make_pair(pt_x, pt_y));
            }
        }
        SetObstacleData(obstacle_i_vector);
    }
    infile.close();
}

/**
 * @description: set the obstacle boundary line data = 1
 * @return {*}
 */
void HybridAstarNode::SetObstacleData(const std::vector<std::pair<double, double>>& obstacles_vector){
    std::pair<double, double> vertex_1;
    std::pair<double, double> vertex_2;
    Eigen::MatrixXd vertex_1_matrix(2, 1);
    Eigen::MatrixXd vertex_2_matrix(2, 1);
    Eigen::MatrixXd new_vertex_1(2, 1);
    new_vertex_1 << 0.0, 0.0;
    Eigen::MatrixXd new_vertex_2(2, 1);
    for (int i = 0; i < int(obstacles_vector.size()); ++i){
        if (i == int(obstacles_vector.size() - 1)){
            vertex_1 = obstacles_vector[-1];
            vertex_2 = obstacles_vector[0];
        }
        else{
            vertex_1 = obstacles_vector[i+1];
            vertex_2 = obstacles_vector[i]; 
        }
        // record the obstacle point position
        vertex_1_matrix << vertex_1.first, vertex_1.second;
        vertex_2_matrix << vertex_2.first, vertex_2.second;
        // get the rotation matrix
        Eigen::Matrix2d rotation_martrix = GetRotationMatrix(vertex_1, vertex_2);
        new_vertex_2 = rotation_martrix * (vertex_2_matrix - vertex_1_matrix);
        int interval_num = int(new_vertex_2(0, 0) / map_resolution_);
        // add interval point to approximate the boundary of obstacles
        // std::cout << "interval_num" << interval_num << std::endl;
        for (int j = 0; j < interval_num + 1; ++j){
            Eigen::MatrixXd interval_point(2, 1);
            if (j == interval_num){
                interval_point << new_vertex_2(0, 0), new_vertex_2(1, 0);
            }
            else{
                interval_point << j * map_resolution_, 0.0;
            }
            Eigen::MatrixXd original_interval_point(2, 1);
            original_interval_point = rotation_martrix.inverse() * interval_point + vertex_1_matrix;
            Eigen::Vector3d point_3d(original_interval_point(0, 0),
                                     original_interval_point(1, 0),
                                     0.0);
            obstacle_boundary_position.push_back(point_3d);
            Eigen::Vector2i point_index = Pose2GridIndex(point_3d);
            map_data[(point_index[1] - 1) * map_x_size_ + point_index[0]] = 1;
            // std::cout << "map_position" << (point_index[1] - 1) * map_x_size_ + point_index[0] << std::endl;
        }
    }
}

void HybridAstarNode::PubObstacleCallback(){
    std_msgs::msg::Float64MultiArray obstacle_position_array;
    std_msgs::msg::Int32MultiArray obstacle_vertex_num_array;
    for (auto point : obstacle_position){
        obstacle_position_array.data.push_back(point.first);
        obstacle_position_array.data.push_back(point.second);
    }
    for (auto vertex_num : obstalce_vertex_num){
        obstacle_vertex_num_array.data.push_back(vertex_num);
    }
    obstacle_position_pub_->publish(obstacle_position_array);
    obstacle_vertex_num_pub_->publish(obstacle_vertex_num_array);
}

void HybridAstarNode::PubPathCallback(){
    if (reach_){
        path_pub_->publish(final_path);
    }
}

bool HybridAstarNode::InitialHybridAstar(){
    // declare parameter
    this->declare_parameter<double>("max_v", max_v_);
    this->declare_parameter<double>("min_v", min_v_);
    this->declare_parameter<double>("max_steering_angle", max_steering_angle_);
    this->declare_parameter<double>("min_steering_angle", min_steering_angle_);
    this->declare_parameter<int>("discrete_num", ds_num_);
    this->declare_parameter<int>("steering_num", steering_num_);
    this->declare_parameter<double>("radius_flag", radius_flag_);
    this->declare_parameter<double>("coeffi_heading", coeffi_heading_);
    this->declare_parameter<double>("coeffi_gear_", coeffi_gear_);
    this->declare_parameter<double>("g_gear_", g_gear_);
    this->declare_parameter<int>("circle_num", circle_num_);
    this->declare_parameter<double>("safe_dis", safe_dis_);

    // vehicle parameter
    this->get_parameter<double>("max_v", this->max_v_);
    this->get_parameter<double>("min_v", this->min_v_);
    this->get_parameter<double>("max_steering_angle", this->max_steering_angle_);
    this->get_parameter<double>("min_steering_angle", this->min_steering_angle_);

    // hybrid parameter
    this->get_parameter<int>("discrete_num", this->ds_num_);
    this->get_parameter<int>("steering_num", this->steering_num_);
    this->get_parameter<double>("radius_flag", this->radius_flag_);
    this->get_parameter<double>("coeffi_heading", this->coeffi_heading_);
    this->get_parameter<double>("coeffi_gear_", this->coeffi_gear_);
    this->get_parameter<double>("g_gear_", this->g_gear_);

    // safe check
    this->get_parameter<int>("circle_num", this->circle_num_);
    this->get_parameter<double>("safe_dis", this->safe_dis_);

    // print paramters
    RCLCPP_INFO(this->get_logger(), "max_v %f", max_v_);
    RCLCPP_INFO(this->get_logger(), "min_v %f", min_v_);
    RCLCPP_INFO(this->get_logger(), "max_steering_angle %f", max_steering_angle_);
    RCLCPP_INFO(this->get_logger(), "min_steering_angle %f", min_steering_angle_);
    RCLCPP_INFO(this->get_logger(), "discrete_num ", ds_num_);
    RCLCPP_INFO(this->get_logger(), "steering_num ", steering_num_);
    RCLCPP_INFO(this->get_logger(), "radius_flag %f", radius_flag_);
    RCLCPP_INFO(this->get_logger(), "coeffi_heading %f", coeffi_heading_);
    RCLCPP_INFO(this->get_logger(), "coeffi_gear_ %f", coeffi_gear_);
    RCLCPP_INFO(this->get_logger(), "g_gear_ %f", g_gear_);
    RCLCPP_INFO(this->get_logger(), "circle_num ", circle_num_);
    RCLCPP_INFO(this->get_logger(), "safe_dis %f", safe_dis_);

    RCLCPP_INFO(this->get_logger(), "Successfully get Hybrid Parameter");
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

    vehicle_.x = vehicle_x;
    vehicle_.y = vehicle_y;
    vehicle_.yaw = vehicle_yaw;

    if (!start_){
        start_pose_[0] = vehicle_x;
        start_pose_[1] = vehicle_y;
        start_pose_[2] = vehicle_yaw;

        start_ = true;
    }
}

void HybridAstarNode::GoalPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg){
    if (!reach_ && start_){
        goal_pose_[0] = goal_pose_msg->pose.position.x;
        goal_pose_[1] = goal_pose_msg->pose.position.y;
        tf2::Quaternion quat_tf;
        tf2::convert(goal_pose_msg->pose.orientation, quat_tf);
        double goal_roll, goal_pitch, goal_yaw;
        tf2::Matrix3x3(quat_tf).getRPY(goal_roll, goal_pitch, goal_yaw);
        PI2PI(goal_yaw);
        goal_pose_[2] = goal_yaw;
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

Eigen::Vector2d HybridAstarNode::GridIndex2Posi(const Eigen::Vector2i & grid_index){
    int index_x = grid_index[0];
    int index_y = grid_index[1];

    Eigen::Vector2d posi(index_x * map_resolution_ + map_xl,
                         index_y * map_resolution_ + map_yl);

    return posi;
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

    // we use fitted circles for collision check
    double radius = 0.5 * sqrt(pow(vehicle_.length / circle_num_, 2) + pow(vehicle_.width, 2));
    for (int i = 0; i < circle_num_; ++i){
        double coefficient = (vehicle_.length - vehicle_.wheelbase) / 2 + vehicle_.wheelbase + vehicle_.length / circle_num_ * (0.5 - i);
        double circle_x = current_pose[0] + coefficient * cos(current_pose[2]);
        double circle_y = current_pose[1] + coefficient * sin(current_pose[2]);
        for (auto obs_position : obstacle_boundary_position){
            double obs_x = obs_position[0];
            double obs_y = obs_position[1];
            double distance = std::sqrt(std::pow(circle_x - obs_x, 2) + std::pow(circle_y - obs_y, 2));
            if (distance <= radius){
                return true;
            }
        }
    }

    return collision;
}

double HybridAstarNode::ComputeH(const GridNodePtr &node1){
    double distance;

    // use manhattan distance
    Eigen::Vector2i index_1 = node1->index;
    Eigen::Vector2i index_2 = Pose2GridIndex(goal_pose_);

    distance = std::sqrt(pow((index_1[0] - index_2[0]), 2) + pow((index_1[1] - index_2[1]), 2));

    return distance;
}

double HybridAstarNode::ComputeG(const GridNodePtr &node1, const GridNodePtr &node2){
    double g_value;
    double cost_gear;
    double cost_heading;

    Eigen::Vector3d pose_1 = node1->pose;
    Eigen::Vector3d pose_2 = node2->pose;

    cost_heading = coeffi_heading_ * abs(pose_2[2] - pose_1[1]);
    if (node2->gear != node1->gear){
        cost_gear = coeffi_gear_ * g_gear_;
    }
    else{
        cost_gear = 0.0;
    }

    g_value = cost_heading + cost_gear;

    return node1->g_score + g_value;
}


void HybridAstarNode::ExpandNode(const GridNodePtr & current_pt){
    Eigen::Vector3d current_pose = current_pt->pose;
    int gear[] = {1, -1};
    double delta_angle = (max_steering_angle_ - min_steering_angle_) / (steering_num_ - 1);
    // expand node
    for (auto gear_i : gear){
        double ds = delta_ds_ * gear_i;
        for (int i = 0; i < steering_num_; ++i){
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
                    node_i->f_score = node_i->g_score + ComputeH(node_i);
                    node_i->FatherNode = current_pt;
                    node_i->pose = current_pose;
                    node_i->gear = gear_i;
                    openset.insert(std::make_pair(node_i->f_score, node_i));
                }
            }
        }
    }
}

bool HybridAstarNode::SearchPath(){
    Eigen::Vector3d current_pose = start_pose_;
    // add start node into the openset
    GridNodePtr current_ptr = new GridNode(Pose2GridIndex(current_pose), current_pose);
    current_ptr->g_score = 0;
    current_ptr->f_score = ComputeH(current_ptr);
    double goal_x, goal_y, goal_theta = goal_pose_[0], goal_pose_[1], goal_pose_[2];
    double min_turn_raduis = vehicle_.wheelbase / tan(min_steering_angle_);
    double max_curvature = 1 / min_turn_raduis;
    while(!reach_ && !openset.empty()){
        // check in the raduis
        double goal_distance = std::sqrt(std::pow(current_pose[0] - goal_pose_[0], 2) + 
                                         std::pow(current_pose[1] - goal_pose_[1], 2));
        if (goal_distance <= radius_flag_){
            Path rs_path = rs_planner.planning(current_pose[0], current_pose[1], current_pose[2],
                                               goal_x, goal_y, goal_theta, max_curvature, map_resolution_);
            // CHECK collision in the rs curve
            bool rs_collision = false;
            int rs_i = 0;
            while(!rs_collision && rs_i < rs_path.x.size()){
                Eigen::Vector3d rs_pose(rs_path.x[rs_i], rs_path.y[rs_i], rs_path.yaw[rs_i]);
                rs_collision = HybridAstarNode::CollisionCheck(rs_pose);
                rs_i++;
            }
            // if not collision, reach goal
            if (!rs_collision){
                reach_ = true;
                // update the final path
                // 1- add the rs path into the final_path
                int i = 0;
                while (i < rs_path.x.size()){
                    final_path.x.push_back(rs_path.x[i]);
                    final_path.y.push_back(rs_path.y[i]);
                    final_path.yaw.push_back(rs_path.yaw[i]);
                    i++;
                }
                // 2- add the hybrid search path into the final_path
                while (current_ptr->FatherNode != NULL){
                    final_path.x.insert(final_path.x.begin(), current_ptr->pose[0]);
                    final_path.y.insert(final_path.y.begin(), current_ptr->pose[1]);
                    final_path.yaw.insert(final_path.yaw.begin(), current_ptr->pose[2]);
                    current_ptr = current_ptr->FatherNode;
                }
                break;
            }
        }
        // if collision or not in the radius flag, and expand current node and chose next one
        HybridAstarNode::ExpandNode(current_ptr);
        current_ptr = (*openset.begin()).second;
        current_pose = current_ptr->pose;
        // remove current ptr from openset
        current_ptr->id == -1;
        openset.erase(openset.begin());
    }
    return true;
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto hybridastar_node = std::make_shared<HybridAstarNode>();
    rclcpp::spin(hybridastar_node);
    rclcpp::shutdown();
    return 0;
}