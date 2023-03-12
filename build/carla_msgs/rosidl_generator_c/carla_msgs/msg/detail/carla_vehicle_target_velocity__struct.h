// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from carla_msgs:msg/CarlaVehicleTargetVelocity.idl
// generated code does not contain a copyright notice

#ifndef CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__STRUCT_H_
#define CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/CarlaVehicleTargetVelocity in the package carla_msgs.
typedef struct carla_msgs__msg__CarlaVehicleTargetVelocity
{
  std_msgs__msg__Header header;
  float velocity;
} carla_msgs__msg__CarlaVehicleTargetVelocity;

// Struct for a sequence of carla_msgs__msg__CarlaVehicleTargetVelocity.
typedef struct carla_msgs__msg__CarlaVehicleTargetVelocity__Sequence
{
  carla_msgs__msg__CarlaVehicleTargetVelocity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} carla_msgs__msg__CarlaVehicleTargetVelocity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CARLA_MSGS__MSG__DETAIL__CARLA_VEHICLE_TARGET_VELOCITY__STRUCT_H_
