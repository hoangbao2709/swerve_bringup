# robot-real-sim

## Goal

Create a simulation environment that is as close as possible to the real robot software architecture.

Gazebo replaces only the physical hardware layer.

Required architecture:

Browser
-> Django / WebSocket
-> ROS Bridge
-> ROS 2
-> Command Arbiter / Nav2 / SLAM
-> Robot Interface
-> Gazebo

The future real robot path must remain:

Browser
-> Django / WebSocket
-> ROS Bridge
-> ROS 2
-> Command Arbiter / Nav2 / Localization
-> Hardware Driver
-> Real Robot

## User-facing application

Keep only:

- /login
- /control
- /robots/:robotId/control

Remove unrelated product pages.

## Mandatory behavior

The remaining functions must not be degraded.

Preserve:

- authentication
- robot overview
- robot detail
- manual control
- autonomous control
- Emergency Stop
- command watchdog
- ROS bridge
- real ROS telemetry
- LiDAR 2D
- LiDAR 3D
- odometry
- TF
- SLAM Toolbox
- Nav2
- map identity
- map revision synchronization
- path preview
- NavigateToPose

## Forbidden

Do not:

- fake robot motion in frontend
- fake LiDAR
- fake telemetry
- bypass ROS bridge
- bypass command arbiter
- bypass Nav2
- remove safety checks
- remove watchdog
- weaken tests to make them pass
- hard-code success
- silently use LOCAL_SIM