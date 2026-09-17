cmake_minimum_required(VERSION 3.8)
project(swerve_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch rviz worlds config
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
  PROGRAMS swerve_controller/swerve_odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME swerve_odometry_node
)

install(
  PROGRAMS swerve_controller/lidar_preprocessor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_preprocessor_node
)

ament_package()
