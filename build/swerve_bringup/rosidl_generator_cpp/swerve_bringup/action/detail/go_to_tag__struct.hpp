// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swerve_bringup:action/GoToTag.idl
// generated code does not contain a copyright notice

#ifndef SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__STRUCT_HPP_
#define SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_Goal __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_Goal __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_Goal_
{
  using Type = GoToTag_Goal_<ContainerAllocator>;

  explicit GoToTag_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_tag_id = 0l;
    }
  }

  explicit GoToTag_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_tag_id = 0l;
    }
  }

  // field types and members
  using _target_tag_id_type =
    int32_t;
  _target_tag_id_type target_tag_id;

  // setters for named parameter idiom
  Type & set__target_tag_id(
    const int32_t & _arg)
  {
    this->target_tag_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_Goal
    std::shared_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_Goal
    std::shared_ptr<swerve_bringup::action::GoToTag_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_Goal_ & other) const
  {
    if (this->target_tag_id != other.target_tag_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_Goal_

// alias to use template instance with default allocator
using GoToTag_Goal =
  swerve_bringup::action::GoToTag_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup


#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_Result __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_Result __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_Result_
{
  using Type = GoToTag_Result_<ContainerAllocator>;

  explicit GoToTag_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->reason = "";
    }
  }

  explicit GoToTag_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->reason = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_Result
    std::shared_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_Result
    std::shared_ptr<swerve_bringup::action::GoToTag_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_Result_

// alias to use template instance with default allocator
using GoToTag_Result =
  swerve_bringup::action::GoToTag_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup


#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_Feedback __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_Feedback_
{
  using Type = GoToTag_Feedback_<ContainerAllocator>;

  explicit GoToTag_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_tag_id = 0l;
      this->next_tag_id = 0l;
      this->target_tag_id = 0l;
      this->state = "";
      this->progress = 0.0f;
    }
  }

  explicit GoToTag_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_tag_id = 0l;
      this->next_tag_id = 0l;
      this->target_tag_id = 0l;
      this->state = "";
      this->progress = 0.0f;
    }
  }

  // field types and members
  using _current_tag_id_type =
    int32_t;
  _current_tag_id_type current_tag_id;
  using _next_tag_id_type =
    int32_t;
  _next_tag_id_type next_tag_id;
  using _target_tag_id_type =
    int32_t;
  _target_tag_id_type target_tag_id;
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _progress_type =
    float;
  _progress_type progress;

  // setters for named parameter idiom
  Type & set__current_tag_id(
    const int32_t & _arg)
  {
    this->current_tag_id = _arg;
    return *this;
  }
  Type & set__next_tag_id(
    const int32_t & _arg)
  {
    this->next_tag_id = _arg;
    return *this;
  }
  Type & set__target_tag_id(
    const int32_t & _arg)
  {
    this->target_tag_id = _arg;
    return *this;
  }
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__progress(
    const float & _arg)
  {
    this->progress = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_Feedback
    std::shared_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_Feedback
    std::shared_ptr<swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_Feedback_ & other) const
  {
    if (this->current_tag_id != other.current_tag_id) {
      return false;
    }
    if (this->next_tag_id != other.next_tag_id) {
      return false;
    }
    if (this->target_tag_id != other.target_tag_id) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->progress != other.progress) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_Feedback_

// alias to use template instance with default allocator
using GoToTag_Feedback =
  swerve_bringup::action::GoToTag_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "swerve_bringup/action/detail/go_to_tag__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Request __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_SendGoal_Request_
{
  using Type = GoToTag_SendGoal_Request_<ContainerAllocator>;

  explicit GoToTag_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit GoToTag_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    swerve_bringup::action::GoToTag_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const swerve_bringup::action::GoToTag_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Request
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Request
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_SendGoal_Request_

// alias to use template instance with default allocator
using GoToTag_SendGoal_Request =
  swerve_bringup::action::GoToTag_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Response __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_SendGoal_Response_
{
  using Type = GoToTag_SendGoal_Response_<ContainerAllocator>;

  explicit GoToTag_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit GoToTag_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Response
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_SendGoal_Response
    std::shared_ptr<swerve_bringup::action::GoToTag_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_SendGoal_Response_

// alias to use template instance with default allocator
using GoToTag_SendGoal_Response =
  swerve_bringup::action::GoToTag_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup

namespace swerve_bringup
{

namespace action
{

struct GoToTag_SendGoal
{
  using Request = swerve_bringup::action::GoToTag_SendGoal_Request;
  using Response = swerve_bringup::action::GoToTag_SendGoal_Response;
};

}  // namespace action

}  // namespace swerve_bringup


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Request __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_GetResult_Request_
{
  using Type = GoToTag_GetResult_Request_<ContainerAllocator>;

  explicit GoToTag_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit GoToTag_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Request
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Request
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_GetResult_Request_

// alias to use template instance with default allocator
using GoToTag_GetResult_Request =
  swerve_bringup::action::GoToTag_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup


// Include directives for member types
// Member 'result'
// already included above
// #include "swerve_bringup/action/detail/go_to_tag__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Response __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_GetResult_Response_
{
  using Type = GoToTag_GetResult_Response_<ContainerAllocator>;

  explicit GoToTag_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit GoToTag_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    swerve_bringup::action::GoToTag_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const swerve_bringup::action::GoToTag_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Response
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_GetResult_Response
    std::shared_ptr<swerve_bringup::action::GoToTag_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_GetResult_Response_

// alias to use template instance with default allocator
using GoToTag_GetResult_Response =
  swerve_bringup::action::GoToTag_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup

namespace swerve_bringup
{

namespace action
{

struct GoToTag_GetResult
{
  using Request = swerve_bringup::action::GoToTag_GetResult_Request;
  using Response = swerve_bringup::action::GoToTag_GetResult_Response;
};

}  // namespace action

}  // namespace swerve_bringup


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "swerve_bringup/action/detail/go_to_tag__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swerve_bringup__action__GoToTag_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__swerve_bringup__action__GoToTag_FeedbackMessage __declspec(deprecated)
#endif

namespace swerve_bringup
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct GoToTag_FeedbackMessage_
{
  using Type = GoToTag_FeedbackMessage_<ContainerAllocator>;

  explicit GoToTag_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit GoToTag_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const swerve_bringup::action::GoToTag_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swerve_bringup__action__GoToTag_FeedbackMessage
    std::shared_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swerve_bringup__action__GoToTag_FeedbackMessage
    std::shared_ptr<swerve_bringup::action::GoToTag_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GoToTag_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const GoToTag_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GoToTag_FeedbackMessage_

// alias to use template instance with default allocator
using GoToTag_FeedbackMessage =
  swerve_bringup::action::GoToTag_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace swerve_bringup

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace swerve_bringup
{

namespace action
{

struct GoToTag
{
  /// The goal message defined in the action definition.
  using Goal = swerve_bringup::action::GoToTag_Goal;
  /// The result message defined in the action definition.
  using Result = swerve_bringup::action::GoToTag_Result;
  /// The feedback message defined in the action definition.
  using Feedback = swerve_bringup::action::GoToTag_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = swerve_bringup::action::GoToTag_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = swerve_bringup::action::GoToTag_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = swerve_bringup::action::GoToTag_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct GoToTag GoToTag;

}  // namespace action

}  // namespace swerve_bringup

#endif  // SWERVE_BRINGUP__ACTION__DETAIL__GO_TO_TAG__STRUCT_HPP_
