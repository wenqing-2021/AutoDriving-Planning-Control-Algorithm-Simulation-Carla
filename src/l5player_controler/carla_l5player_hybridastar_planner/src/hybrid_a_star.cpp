#include "carla_l5player_hybridastar_planner/hybrid_a_star.h"

using namespace l5player::planner;

HybridAstarNode::HybridAstarNode() : Node("hybrid_a_star"){
    this->get_parameter<double>("map_max_x", map_xu_);
    this->get_parameter<double>("map_min_x", map_xl_);
    this->get_parameter<double>("map_max_y", map_yu_);
    this->get_parameter<double>("map_min_y", map_yl_);
    
}

bool HybridAstarNode::InitialMap(){

        map_x_size_ = (int)((map_xu_ - map_xl_) / resolution_);
        map_y_size_ = (int)((map_yu_ - map_yl_) / resolution_);
        map_xy_size_ = map_x_size_ * map_y_size_;

        GridNodeMap = new GridNodePtr * [map_x_size_]; // 二级指针
        for (int i = 0; i < map_x_size_; i++){
            GridNodeMap[i] = new GridNodePtr [map_y_size_];
            for (int j = 0; j < map_y_size_; j++){
                Eigen::Vector2i tmp_index(i,j);
                Eigen::Vector3d tmp_pose((double)(i*resolution_), (double)(j*resolution_), 0.0);
                GridNodeMap[i][j] = new GridNode(tmp_index, tmp_pose);
            }
        }
    }


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    return 0;
}