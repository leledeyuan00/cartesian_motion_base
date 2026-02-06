# Cartesian Motion Base Library (CMB)


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