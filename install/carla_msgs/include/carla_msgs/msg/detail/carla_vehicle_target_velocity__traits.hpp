// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from carla_msgs:msg/CarlaVehicleTargetVelocity.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__TRAITS_HPP_
#define CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__TRAITS_HPP_

#include "carla_msgs/msg/detail/carla_vehicle_target_velocity__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<carla_msgs::msg::CarlaVehicleTargetVelocity>()
{
  return "carla_msgs::msg::CarlaVehicleTargetVelocity";
}

template<>
inline const char * name<carla_msgs::msg::CarlaVehicleTargetVelocity>()
{
  return "carla_msgs/msg/CarlaVehicleTargetVelocity";
}

template<>
struct has_fixed_size<carla_msgs::msg::CarlaVehicleTargetVelocity>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<carla_msgs::msg::CarlaVehicleTargetVelocity>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<carla_msgs::msg::CarlaVehicleTargetVelocity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__TRAITS_HPP_
