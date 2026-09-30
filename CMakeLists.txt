cmake_minimum_required(VERSION 3.8)
project(swerve_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
