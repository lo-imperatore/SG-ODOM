# any_msgs (vendored)

Minimal subset providing `any_msgs/msg/ExtendedJointState`, the joint-state
type nested in `anymal_msgs/msg/AnymalState`. Not authored by the SG-ODOM
project.

- **Source:** [leggedrobotics/holistic_fusion](https://github.com/leggedrobotics/holistic_fusion/tree/9db3c2698292dc256c2a4d148328b33425621c86/ros/holistic_fusion_anymal_msgs/msg),
  file `ros/holistic_fusion_anymal_msgs/msg/ExtendedJointState.msg`
- **Upstream commit:** `9db3c2698292dc256c2a4d148328b33425621c86`
- **License:** BSD-3-Clause, Copyright (c) 2025 Julian Nubert, Robotic
  Systems Lab, ETH Zurich. See [LICENSE](LICENSE).

## Included files

- `msg/ExtendedJointState.msg`: identical to upstream.

## Modifications from upstream

- Placed in its own package named `any_msgs` (upstream ships it inside
  `holistic_fusion_anymal_msgs`), so the full type name matches ROS 2
  ANYmal recordings: `any_msgs/msg/ExtendedJointState`.
- New ROS 2 `CMakeLists.txt` / `package.xml`.
