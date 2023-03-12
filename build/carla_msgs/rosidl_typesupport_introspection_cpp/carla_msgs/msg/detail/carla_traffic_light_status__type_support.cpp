// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from carla_msgs:msg/CarlaTrafficLightStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "carla_msgs/msg/detail/carla_traffic_light_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace carla_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CarlaTrafficLightStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) carla_msgs::msg::CarlaTrafficLightStatus(_init);
}

void CarlaTrafficLightStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<carla_msgs::msg::CarlaTrafficLightStatus *>(message_memory);
  typed_message->~CarlaTrafficLightStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CarlaTrafficLightStatus_message_member_array[2] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs::msg::CarlaTrafficLightStatus, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(carla_msgs::msg::CarlaTrafficLightStatus, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CarlaTrafficLightStatus_message_members = {
  "carla_msgs::msg",  // message namespace
  "CarlaTrafficLightStatus",  // message name
  2,  // number of fields
  sizeof(carla_msgs::msg::CarlaTrafficLightStatus),
  CarlaTrafficLightStatus_message_member_array,  // message members
  CarlaTrafficLightStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  CarlaTrafficLightStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CarlaTrafficLightStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CarlaTrafficLightStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace carla_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<carla_msgs::msg::CarlaTrafficLightStatus>()
{
  return &::carla_msgs::msg::rosidl_typesupport_introspection_cpp::CarlaTrafficLightStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, carla_msgs, msg, CarlaTrafficLightStatus)() {
  return &::carla_msgs::msg::rosidl_typesupport_introspection_cpp::CarlaTrafficLightStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
