# anymal_msgs (vendored)

Minimal subset of ANYmal message definitions, vendored so the estimator can
subscribe to `anymal_msgs/msg/AnymalState` from recorded ROS 2 bags. Not
authored by the SG-ODOM project.

- **Source:** [leggedrobotics/holistic_fusion](https://github.com/leggedrobotics/holistic_fusion/tree/9db3c2698292dc256c2a4d148328b33425621c86/ros/holistic_fusion_anymal_msgs/msg),
  package `ros/holistic_fusion_anymal_msgs` (ROS 1 / catkin)
- **Upstream commit:** `9db3c2698292dc256c2a4d148328b33425621c86`
- **License:** BSD-3-Clause, Copyright (c) 2025 Julian Nubert, Robotic
  Systems Lab, ETH Zurich. See [LICENSE](LICENSE).

## Included files

- `msg/AnymalState.msg`
- `msg/Contact.msg`

## Modifications from upstream

Ported from ROS 1 to ROS 2. Field layout and types are unchanged:

- Package renamed from `holistic_fusion_anymal_msgs` to `anymal_msgs`, and
  new ROS 2 `CMakeLists.txt` / `package.xml` written.
- `Header` written as `std_msgs/Header` (ROS 2 syntax).
- `AnymalState.joints` references `any_msgs/ExtendedJointState`, which
  upstream defines in the same package; it is vendored separately in
  [`../any_msgs`](../any_msgs).
- `Contact.msg`: `frictionCoefficient` / `restitutionCoefficient` renamed
  to `friction_coefficient` / `restitution_coefficient` (ROS 2 requires
  snake_case field names; the serialized layout is unaffected).
