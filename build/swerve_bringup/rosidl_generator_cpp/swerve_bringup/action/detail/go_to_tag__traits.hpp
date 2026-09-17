// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swerve_bringup:action/GoToTag.idl
// generated code does not contain a copyright notice

#ifndef SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__TRAITS_HPP_
#define SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swerve_bringup/action/detail/go_to_tag__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_tag_id
  {
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_Goal & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_Goal>()
{
  return "swerve_bringup::action::GoToTag_Goal";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_Goal>()
{
  return "swerve_bringup/action/GoToTag_Goal";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_Result & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_Result>()
{
  return "swerve_bringup::action::GoToTag_Result";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_Result>()
{
  return "swerve_bringup/action/GoToTag_Result";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_tag_id
  {
    out << "current_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_tag_id, out);
    out << ", ";
  }

  // member: next_tag_id
  {
    out << "next_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.next_tag_id, out);
    out << ", ";
  }

  // member: target_tag_id
  {
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: progress
  {
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.current_tag_id, out);
    out << "\n";
  }

  // member: next_tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "next_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.next_tag_id, out);
    out << "\n";
  }

  // member: target_tag_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_tag_id: ";
    rosidl_generator_traits::value_to_yaml(msg.target_tag_id, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_Feedback & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_Feedback>()
{
  return "swerve_bringup::action::GoToTag_Feedback";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_Feedback>()
{
  return "swerve_bringup/action/GoToTag_Feedback";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "swerve_bringup/action/detail/go_to_tag__traits.hpp"

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_SendGoal_Request & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_SendGoal_Request>()
{
  return "swerve_bringup::action::GoToTag_SendGoal_Request";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_SendGoal_Request>()
{
  return "swerve_bringup/action/GoToTag_SendGoal_Request";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<swerve_bringup::action::GoToTag_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<swerve_bringup::action::GoToTag_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_SendGoal_Response & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_SendGoal_Response>()
{
  return "swerve_bringup::action::GoToTag_SendGoal_Response";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_SendGoal_Response>()
{
  return "swerve_bringup/action/GoToTag_SendGoal_Response";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_SendGoal>()
{
  return "swerve_bringup::action::GoToTag_SendGoal";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_SendGoal>()
{
  return "swerve_bringup/action/GoToTag_SendGoal";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<swerve_bringup::action::GoToTag_SendGoal_Request>::value &&
    has_fixed_size<swerve_bringup::action::GoToTag_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<swerve_bringup::action::GoToTag_SendGoal_Request>::value &&
    has_bounded_size<swerve_bringup::action::GoToTag_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<swerve_bringup::action::GoToTag_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<swerve_bringup::action::GoToTag_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<swerve_bringup::action::GoToTag_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_GetResult_Request & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_GetResult_Request>()
{
  return "swerve_bringup::action::GoToTag_GetResult_Request";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_GetResult_Request>()
{
  return "swerve_bringup/action/GoToTag_GetResult_Request";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "swerve_bringup/action/detail/go_to_tag__traits.hpp"

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_GetResult_Response & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_GetResult_Response>()
{
  return "swerve_bringup::action::GoToTag_GetResult_Response";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_GetResult_Response>()
{
  return "swerve_bringup/action/GoToTag_GetResult_Response";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<swerve_bringup::action::GoToTag_Result>::value> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<swerve_bringup::action::GoToTag_Result>::value> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_GetResult>()
{
  return "swerve_bringup::action::GoToTag_GetResult";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_GetResult>()
{
  return "swerve_bringup/action/GoToTag_GetResult";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<swerve_bringup::action::GoToTag_GetResult_Request>::value &&
    has_fixed_size<swerve_bringup::action::GoToTag_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<swerve_bringup::action::GoToTag_GetResult_Request>::value &&
    has_bounded_size<swerve_bringup::action::GoToTag_GetResult_Response>::value
  >
{
};

template<>
struct is_service<swerve_bringup::action::GoToTag_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<swerve_bringup::action::GoToTag_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<swerve_bringup::action::GoToTag_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "swerve_bringup/action/detail/go_to_tag__traits.hpp"

namespace swerve_bringup
{

namespace action
{

inline void to_flow_style_yaml(
  const GoToTag_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GoToTag_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GoToTag_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace swerve_bringup

namespace rosidl_generator_traits
{

[[deprecated("use swerve_bringup::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swerve_bringup::action::GoToTag_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  swerve_bringup::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swerve_bringup::action::to_yaml() instead")]]
inline std::string to_yaml(const swerve_bringup::action::GoToTag_FeedbackMessage & msg)
{
  return swerve_bringup::action::to_yaml(msg);
}

template<>
inline const char * data_type<swerve_bringup::action::GoToTag_FeedbackMessage>()
{
  return "swerve_bringup::action::GoToTag_FeedbackMessage";
}

template<>
inline const char * name<swerve_bringup::action::GoToTag_FeedbackMessage>()
{
  return "swerve_bringup/action/GoToTag_FeedbackMessage";
}

template<>
struct has_fixed_size<swerve_bringup::action::GoToTag_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<swerve_bringup::action::GoToTag_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<swerve_bringup::action::GoToTag_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<swerve_bringup::action::GoToTag_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<swerve_bringup::action::GoToTag_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<swerve_bringup::action::GoToTag>
  : std::true_type
{
};

template<>
struct is_action_goal<swerve_bringup::action::GoToTag_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<swerve_bringup::action::GoToTag_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<swerve_bringup::action::GoToTag_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__TRAITS_HPP_
