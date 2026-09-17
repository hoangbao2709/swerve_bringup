// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swerve_bringup:action/GoToTag.idl
// generated code does not contain a copyright notice

#ifndef SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__BUILDER_HPP_
#define SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swerve_bringup/action/detail/go_to_tag__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_Goal_target_tag_id
{
public:
  Init_GoToTag_Goal_target_tag_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::swerve_bringup::action::GoToTag_Goal target_tag_id(::swerve_bringup::action::GoToTag_Goal::_target_tag_id_type arg)
  {
    msg_.target_tag_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_Goal>()
{
  return swerve_bringup::action::builder::Init_GoToTag_Goal_target_tag_id();
}

}  // namespace swerve_bringup


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_Result_reason
{
public:
  explicit Init_GoToTag_Result_reason(::swerve_bringup::action::GoToTag_Result & msg)
  : msg_(msg)
  {}
  ::swerve_bringup::action::GoToTag_Result reason(::swerve_bringup::action::GoToTag_Result::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Result msg_;
};

class Init_GoToTag_Result_success
{
public:
  Init_GoToTag_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToTag_Result_reason success(::swerve_bringup::action::GoToTag_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GoToTag_Result_reason(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_Result>()
{
  return swerve_bringup::action::builder::Init_GoToTag_Result_success();
}

}  // namespace swerve_bringup


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_Feedback_progress
{
public:
  explicit Init_GoToTag_Feedback_progress(::swerve_bringup::action::GoToTag_Feedback & msg)
  : msg_(msg)
  {}
  ::swerve_bringup::action::GoToTag_Feedback progress(::swerve_bringup::action::GoToTag_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Feedback msg_;
};

class Init_GoToTag_Feedback_state
{
public:
  explicit Init_GoToTag_Feedback_state(::swerve_bringup::action::GoToTag_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToTag_Feedback_progress state(::swerve_bringup::action::GoToTag_Feedback::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_GoToTag_Feedback_progress(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Feedback msg_;
};

class Init_GoToTag_Feedback_target_tag_id
{
public:
  explicit Init_GoToTag_Feedback_target_tag_id(::swerve_bringup::action::GoToTag_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToTag_Feedback_state target_tag_id(::swerve_bringup::action::GoToTag_Feedback::_target_tag_id_type arg)
  {
    msg_.target_tag_id = std::move(arg);
    return Init_GoToTag_Feedback_state(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Feedback msg_;
};

class Init_GoToTag_Feedback_next_tag_id
{
public:
  explicit Init_GoToTag_Feedback_next_tag_id(::swerve_bringup::action::GoToTag_Feedback & msg)
  : msg_(msg)
  {}
  Init_GoToTag_Feedback_target_tag_id next_tag_id(::swerve_bringup::action::GoToTag_Feedback::_next_tag_id_type arg)
  {
    msg_.next_tag_id = std::move(arg);
    return Init_GoToTag_Feedback_target_tag_id(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Feedback msg_;
};

class Init_GoToTag_Feedback_current_tag_id
{
public:
  Init_GoToTag_Feedback_current_tag_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToTag_Feedback_next_tag_id current_tag_id(::swerve_bringup::action::GoToTag_Feedback::_current_tag_id_type arg)
  {
    msg_.current_tag_id = std::move(arg);
    return Init_GoToTag_Feedback_next_tag_id(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_Feedback>()
{
  return swerve_bringup::action::builder::Init_GoToTag_Feedback_current_tag_id();
}

}  // namespace swerve_bringup


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_SendGoal_Request_goal
{
public:
  explicit Init_GoToTag_SendGoal_Request_goal(::swerve_bringup::action::GoToTag_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::swerve_bringup::action::GoToTag_SendGoal_Request goal(::swerve_bringup::action::GoToTag_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_SendGoal_Request msg_;
};

class Init_GoToTag_SendGoal_Request_goal_id
{
public:
  Init_GoToTag_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToTag_SendGoal_Request_goal goal_id(::swerve_bringup::action::GoToTag_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoToTag_SendGoal_Request_goal(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_SendGoal_Request>()
{
  return swerve_bringup::action::builder::Init_GoToTag_SendGoal_Request_goal_id();
}

}  // namespace swerve_bringup


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_SendGoal_Response_stamp
{
public:
  explicit Init_GoToTag_SendGoal_Response_stamp(::swerve_bringup::action::GoToTag_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::swerve_bringup::action::GoToTag_SendGoal_Response stamp(::swerve_bringup::action::GoToTag_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_SendGoal_Response msg_;
};

class Init_GoToTag_SendGoal_Response_accepted
{
public:
  Init_GoToTag_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToTag_SendGoal_Response_stamp accepted(::swerve_bringup::action::GoToTag_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GoToTag_SendGoal_Response_stamp(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_SendGoal_Response>()
{
  return swerve_bringup::action::builder::Init_GoToTag_SendGoal_Response_accepted();
}

}  // namespace swerve_bringup


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_GetResult_Request_goal_id
{
public:
  Init_GoToTag_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::swerve_bringup::action::GoToTag_GetResult_Request goal_id(::swerve_bringup::action::GoToTag_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_GetResult_Request>()
{
  return swerve_bringup::action::builder::Init_GoToTag_GetResult_Request_goal_id();
}

}  // namespace swerve_bringup


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_GetResult_Response_result
{
public:
  explicit Init_GoToTag_GetResult_Response_result(::swerve_bringup::action::GoToTag_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::swerve_bringup::action::GoToTag_GetResult_Response result(::swerve_bringup::action::GoToTag_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_GetResult_Response msg_;
};

class Init_GoToTag_GetResult_Response_status
{
public:
  Init_GoToTag_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToTag_GetResult_Response_result status(::swerve_bringup::action::GoToTag_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GoToTag_GetResult_Response_result(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_GetResult_Response>()
{
  return swerve_bringup::action::builder::Init_GoToTag_GetResult_Response_status();
}

}  // namespace swerve_bringup


namespace swerve_bringup
{

namespace action
{

namespace builder
{

class Init_GoToTag_FeedbackMessage_feedback
{
public:
  explicit Init_GoToTag_FeedbackMessage_feedback(::swerve_bringup::action::GoToTag_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::swerve_bringup::action::GoToTag_FeedbackMessage feedback(::swerve_bringup::action::GoToTag_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_FeedbackMessage msg_;
};

class Init_GoToTag_FeedbackMessage_goal_id
{
public:
  Init_GoToTag_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoToTag_FeedbackMessage_feedback goal_id(::swerve_bringup::action::GoToTag_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GoToTag_FeedbackMessage_feedback(msg_);
  }

private:
  ::swerve_bringup::action::GoToTag_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::swerve_bringup::action::GoToTag_FeedbackMessage>()
{
  return swerve_bringup::action::builder::Init_GoToTag_FeedbackMessage_goal_id();
}

}  // namespace swerve_bringup

#endif  // SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__BUILDER_HPP_
