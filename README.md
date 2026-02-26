# Cartesian Motion Base Library (CMB)

[![ROS 2 CI (Build and Test)](https://github.com/leledeyuan00/cartesian_motion_base/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/leledeyuan00/cartesian_motion_base/actions/workflows/ros2_ci.yml)

## :bulb: What's this library?

**Cartesian Motion Base** is a lightweight C++ library for robot Cartesian control in ROS 2. 
It provides a flexible framework for motion planning, task sequencing, and real-time control.

![Cartesian Motion Base Architecture](https://github.com/user-attachments/assets/b3149861-9ca2-418f-80c7-6e1c20ea0f6b)


## :robot: Simulation

Since CMB is designed for a general manipulator control architecture, we used a popular UR5 manipulator as an example. You can easily play and test it in the docker environment.

### :whale: Quick play with Docker

```bash
cd .docker
./set_display.bash
docker compose up -d
docker container exec -it cartesian_motion_sim /bin/bash
ros2 launch cartesian_motion_sim cartesian_motion_sim.launch.py
```

Then open a new terminal

```bash
docker container exec -it cartesian_motion_sim /bin/bash
ros2 launch cartesian_motion_test cartesian_motion_test.launch.py
```

### Dual Arm Simulation
```bash
ros2 launch cartesian_motion_sim cartesian_motion_sim.launch.py config_type:=dual_arm
```

```bash
ros2 launch cartesian_motion_test cartesian_motion_dual_arm_test.launch.launch.py 
```

## :bookmark_tabs: Documentation
Please visit our official [Cartesian Motion Base Doc](https://leledeyuan00.github.io/cartesian_motion_base/) for tutorials and examples.

## :octocat: Contributing
We welcome contributions from the community! Please see our [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on how to report bugs, request enhancements, and contribute code.

------------------
**Copyright (c) 2026 Dayuan Chen / [Smart Robots Design Lab, Tohoku University](https://srd.mech.tohoku.ac.jp/en/).**
![Lab Logo](https://srd.mech.tohoku.ac.jp/wordpress/wp-content/uploads/2023/12/Lab_logo_new-1.png)