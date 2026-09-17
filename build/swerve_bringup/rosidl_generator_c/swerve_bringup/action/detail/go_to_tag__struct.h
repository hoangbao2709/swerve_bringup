// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swerve_bringup:action/GoToTag.idl
// generated code does not contain a copyright notice

#ifndef SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__STRUCT_H_
#define SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_Goal
{
  int32_t target_tag_id;
} swerve_bringup__action__GoToTag_Goal;

// Struct for a sequence of swerve_bringup__action__GoToTag_Goal.
typedef struct swerve_bringup__action__GoToTag_Goal__Sequence
{
  swerve_bringup__action__GoToTag_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_Result
{
  bool success;
  rosidl_runtime_c__String reason;
} swerve_bringup__action__GoToTag_Result;

// Struct for a sequence of swerve_bringup__action__GoToTag_Result.
typedef struct swerve_bringup__action__GoToTag_Result__Sequence
{
  swerve_bringup__action__GoToTag_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'state'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_Feedback
{
  int32_t current_tag_id;
  int32_t next_tag_id;
  int32_t target_tag_id;
  rosidl_runtime_c__String state;
  float progress;
} swerve_bringup__action__GoToTag_Feedback;

// Struct for a sequence of swerve_bringup__action__GoToTag_Feedback.
typedef struct swerve_bringup__action__GoToTag_Feedback__Sequence
{
  swerve_bringup__action__GoToTag_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "swerve_bringup/action/detail/go_to_tag__struct.h"

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  swerve_bringup__action__GoToTag_Goal goal;
} swerve_bringup__action__GoToTag_SendGoal_Request;

// Struct for a sequence of swerve_bringup__action__GoToTag_SendGoal_Request.
typedef struct swerve_bringup__action__GoToTag_SendGoal_Request__Sequence
{
  swerve_bringup__action__GoToTag_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} swerve_bringup__action__GoToTag_SendGoal_Response;

// Struct for a sequence of swerve_bringup__action__GoToTag_SendGoal_Response.
typedef struct swerve_bringup__action__GoToTag_SendGoal_Response__Sequence
{
  swerve_bringup__action__GoToTag_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} swerve_bringup__action__GoToTag_GetResult_Request;

// Struct for a sequence of swerve_bringup__action__GoToTag_GetResult_Request.
typedef struct swerve_bringup__action__GoToTag_GetResult_Request__Sequence
{
  swerve_bringup__action__GoToTag_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "swerve_bringup/action/detail/go_to_tag__struct.h"

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_GetResult_Response
{
  int8_t status;
  swerve_bringup__action__GoToTag_Result result;
} swerve_bringup__action__GoToTag_GetResult_Response;

// Struct for a sequence of swerve_bringup__action__GoToTag_GetResult_Response.
typedef struct swerve_bringup__action__GoToTag_GetResult_Response__Sequence
{
  swerve_bringup__action__GoToTag_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "swerve_bringup/action/detail/go_to_tag__struct.h"

/// Struct defined in action/GoToTag in the package swerve_bringup.
typedef struct swerve_bringup__action__GoToTag_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  swerve_bringup__action__GoToTag_Feedback feedback;
} swerve_bringup__action__GoToTag_FeedbackMessage;

// Struct for a sequence of swerve_bringup__action__GoToTag_FeedbackMessage.
typedef struct swerve_bringup__action__GoToTag_FeedbackMessage__Sequence
{
  swerve_bringup__action__GoToTag_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swerve_bringup__action__GoToTag_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__STRUCT_H_
