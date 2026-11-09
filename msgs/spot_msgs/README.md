# spot_msgs (vendored)

Minimal subset of the Spot ROS 2 driver messages, vendored so the estimator
can decode `spot_msgs/msg/FootStateArray` from recorded ROS 2 bags without
building the full driver (whose `spot_msgs` depends on the large
`bosdyn_msgs` package). Not authored by the SG-ODOM project.

- **Source:** [rai-opensource/spot_ros2](https://github.com/rai-opensource/spot_ros2/tree/30550d726b4798419a05662073822f5aebf2c1b1/spot_msgs/msg),
  package `spot_msgs`
- **Upstream commit:** `30550d726b4798419a05662073822f5aebf2c1b1`
- **License:** MIT, Copyright (c) 2022-2024 MASKOR & Robotics and AI
  Institute LLC dba RAI Institute (the upstream LICENSE file also carries a
  BSD-3-Clause notice from Clearpath Robotics Inc.). See [LICENSE](LICENSE).

## Included files

- `msg/FootState.msg`: identical to upstream except for a comment.
- `msg/FootStateArray.msg`

## Modifications from upstream

- Only the two messages above are kept; new `CMakeLists.txt` /
  `package.xml` without the `bosdyn_msgs` dependency.
- **`FootStateArray.msg` adds a leading `std_msgs/Header header` field that
  upstream does not have.** This changes the serialized layout: messages
  recorded with the upstream driver cannot be decoded with this definition.
  The estimator's `state` contact classifier reads the timestamp from this
  header. The default `stability_gate` classifier uses `/joint_states`
  instead and does not subscribe to this message.
