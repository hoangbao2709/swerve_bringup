# Run the real-path simulation

The simulation branch keeps Login and Robot Control connected to the backend
and ROS runtime. Gazebo supplies the simulated robot and sensors; no frontend
simulation takes over when a service is unavailable.

From the repository root:

```bash
./scripts/setup_full_stack.sh
./scripts/start_stack.sh unified --gui --rviz
```

The unified profile starts Gazebo, ROS 2 control, SLAM Toolbox, Nav2, Django,
the ROS bridge, and the frontend. Follow [SETUP_A_Z.md](../SETUP_A_Z.md) for
machine prerequisites, map publication, readiness probes, and diagnostics.

Application routes are `/login`, `/control`, and
`/robots/:robotId/control`. A missing bridge, runtime value, E-stop
acknowledgement, or matching map revision remains unavailable and blocks the
corresponding motion action.
