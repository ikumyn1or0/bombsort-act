#!/bin/bash
# コントローラを起動し直す（起動時の切り替えが時間切れになったとき）
source /opt/ros/humble/setup.bash; source ~/airobot_ws/install/setup.bash
ros2 control list_controllers
for c in joint_state_broadcaster crane_plus_arm_controller crane_plus_gripper_controller; do
  ros2 control set_controller_state $c active 2>&1 | tail -1
done
ros2 control list_controllers
