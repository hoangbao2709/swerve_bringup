cmake_minimum_required(VERSION 3.8)
project(swerve_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(gazebo REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
add_library(swerve_gazebo_motor_system SHARED ros_compat/gazebo_motor_system.cpp)
target_compile_features(swerve_gazebo_motor_system PUBLIC cxx_std_17)
target_include_directories(swerve_gazebo_motor_system PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(swerve_gazebo_motor_system ${GAZEBO_LIBRARIES})
ament_target_dependencies(swerve_gazebo_motor_system gazebo_ros2_control hardware_interface pluginlib
  rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(gazebo_ros2_control config/gazebo_motor_plugin.xml)
install(TARGETS swerve_gazebo_motor_system LIBRARY DESTINATION lib)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_constraint tests/test_motor_constraint.cpp)
endif()

# Humble bondcpp 3.0.2 polls its FSM without locking while Nav2's bond
# callbacks transition it. Keep its exact ABI/source and backport synchronized
# wait methods inside this workspace. Newer system versions need no override.
find_package(bondcpp REQUIRED)
if(bondcpp_VERSION VERSION_EQUAL "3.0.2")
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(bond REQUIRED)
  find_package(smclib REQUIRED)
  add_library(bondcpp_humble_compat SHARED
    ros_compat/bondcpp_3_0_2/bond.cpp
    ros_compat/bondcpp_3_0_2/BondSM_sm.cpp)
  target_include_directories(bondcpp_humble_compat PRIVATE ${bondcpp_INCLUDE_DIRS})
  ament_target_dependencies(bondcpp_humble_compat rclcpp rclcpp_lifecycle bond smclib)
  target_link_libraries(bondcpp_humble_compat uuid)
  target_compile_features(bondcpp_humble_compat PUBLIC cxx_std_17)
  set_target_properties(bondcpp_humble_compat PROPERTIES OUTPUT_NAME bondcpp)
  install(TARGETS bondcpp_humble_compat LIBRARY DESTINATION lib)
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_bond_wait tests/test_bond_wait.cpp)
    target_include_directories(test_bond_wait PRIVATE ${bondcpp_INCLUDE_DIRS})
    ament_target_dependencies(test_bond_wait rclcpp rclcpp_lifecycle bond smclib)
    target_link_libraries(test_bond_wait bondcpp_humble_compat)
  endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToTag.action"
)

install(
  DIRECTORY urdf meshes launch rviz worlds config models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY swerve_navigation/config swerve_navigation/maps
  DESTINATION share/${PROJECT_NAME}/swerve_navigation
)

# Keep the requested project layout while exposing the navigation launch file
# through the standard `ros2 launch <package> navigation.launch.py` lookup.
install(
  FILES swerve_navigation/launch/navigation.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  PROGRAMS swerve_controller/swerve_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME swerve_controller_node
)

install(
  PROGRAMS swerve_controller/command_arbiter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME command_arbiter_node
)

install(
  FILES swerve_controller/command_ownership.py
        swerve_controller/steering_alignment.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS swerve_controller/swerve_odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME swerve_odometry_node
)

install(
  PROGRAMS swerve_controller/lidar_preprocessor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_preprocessor_node
)

install(
  PROGRAMS swerve_controller/v30e_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME v30e_sim_node
)

install(
  PROGRAMS swerve_controller/tag_route_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tag_route_planner
)

install(
  FILES swerve_controller/tag_navigation_core.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

# The web full-stack entrypoint reuses these exact safety gates from the install.
install(FILES scripts/ros_stack_supervisor.py scripts/navigation_readiness.py
  DESTINATION share/${PROJECT_NAME}/scripts)

install(
  PROGRAMS test_navigation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_navigation.py
)

install(
  PROGRAMS indoor_localization_navigation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tag_navigation_cli
)

install(
  PROGRAMS accuracy_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME accuracy_benchmark.py
)

install(
  PROGRAMS scripts/navigation_readiness.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_readiness.py
)

install(
  PROGRAMS scripts/controller_spawner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller_spawner
)

install(
  PROGRAMS scripts/run_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_acceptance.py
)

install(
  PROGRAMS scripts/raw_motion.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME raw_motion.py
)

install(
  PROGRAMS scripts/validate_caster_geometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_caster_geometry.py
)

install(
  PROGRAMS scripts/nav_accuracy_case.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav_accuracy_case.py
)

install(
  PROGRAMS scripts/run_full_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME run_full_acceptance.py
)

ament_package()
