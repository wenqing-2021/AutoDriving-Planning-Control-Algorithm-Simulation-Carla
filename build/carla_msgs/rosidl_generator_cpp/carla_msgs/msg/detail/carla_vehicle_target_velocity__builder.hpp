// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from carla_msgs:msg/CarlaVehicleTargetVelocity.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__BUILDER_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__BUILDER_HPP_

#include "carla_msgs/msg/detail/carla_vehicle_target_velocity__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace carla_msgs
{

namespace msg
{

namespace builder
{

class Init_CarlaVehicleTargetVelocity_velocity
{
public:
  explicit Init_CarlaVehicleTargetVelocity_velocity(::carla_msgs::msg::CarlaVehicleTargetVelocity & msg)
  : msg_(msg)
  {}
  ::carla_msgs::msg::CarlaVehicleTargetVelocity velocity(::carla_msgs::msg::CarlaVehicleTargetVelocity::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::carla_msgs::msg::CarlaVehicleTargetVelocity msg_;
};

class Init_CarlaVehicleTargetVelocity_header
{
public:
  Init_CarlaVehicleTargetVelocity_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarlaVehicleTargetVelocity_velocity header(::carla_msgs::msg::CarlaVehicleTargetVelocity::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CarlaVehicleTargetVelocity_velocity(msg_);
  }

private:
  ::carla_msgs::msg::CarlaVehicleTargetVelocity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::carla_msgs::msg::CarlaVehicleTargetVelocity>()
{
  return carla_msgs::msg::builder::Init_CarlaVehicleTargetVelocity_header();
}

}  // namespace carla_msgs

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__BUILDER_HPP_
