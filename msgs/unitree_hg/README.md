# unitree_hg (vendored)

Minimal subset of the Unitree humanoid (G1/H1-2) low-level messages,
vendored so the estimator can subscribe to `unitree_hg/msg/LowState`. Not
authored by the SG-ODOM project.

- **Source:** [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2/tree/1ed0d02d443bd3e1f63ef07cd32e443c57a9ed94/cyclonedds_ws/src/unitree/unitree_hg),
  package `cyclonedds_ws/src/unitree/unitree_hg`
- **Upstream commit:** `1ed0d02d443bd3e1f63ef07cd32e443c57a9ed94`
- **License:** BSD-3-Clause, Copyright (c) 2016-2024 HangZhou YuShu
  TECHNOLOGY CO.,LTD. ("Unitree Robotics"). See [LICENSE](LICENSE).

## Included files

All identical to upstream:

- `msg/IMUState.msg`
- `msg/LowState.msg`
- `msg/MotorState.msg`

## Modifications from upstream

- Only the three messages above are kept (upstream also has command, hand,
  BMS and pressure-sensor messages); `CMakeLists.txt` lists only these.
