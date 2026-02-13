# Cartesian Motion Base Library (CMB)

[![ROS 2 CI (Build and Test)](https://github.com/leledeyuan00/cartesian_motion_base/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/leledeyuan00/cartesian_motion_base/actions/workflows/ros2_ci.yml)
## Simulation

### Single Arm Simulation
```bash
cd cartesian_motion_sim
./set_display.bash
docker compose up -d
docker container exec -it cartesian_motion_sim /bin/bash
ros2 launch cartesian_motion_sim cartesian_motion_sim.launch.py config_type:=dual_arm
```

```bash
ros2 launch cartesian_motion_test cartesian_motion_test.launch.py
```

### Dual Arm Simulation
```bash
ros2 launch cartesian_motion_sim cartesian_motion_sim.launch.py config_type:=dual_arm
```

```bash
ros2 launch cartesian_motion_test cartesian_motion_dual_arm_test.launch.launch.py 
```

## Citation
If you use this library in your research, please cite the following Zenodo release:

```
@software{dayuan_2026_18616705,
  author       = {Dayuan},
  title        = {leledeyuan00/cartesian\_motion\_base: Realease
                   v0.1.0},
  month        = feb,
  year         = 2026,
  publisher    = {Zenodo},
  version      = {v0.1.0},
  doi          = {10.5281/zenodo.18616705},
  url          = {https://doi.org/10.5281/zenodo.18616705},
}