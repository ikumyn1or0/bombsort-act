#!/bin/bash
# 試作ワールドを起動（docker exec -d で呼ぶ）。引数: ワールド名(bombsort|bombsort_nocam) gui(1|0)
export DISPLAY=:0 XAUTHORITY=/home/ubuntu/.Xauthority
source /opt/ros/humble/setup.bash; source ~/airobot_ws/install/setup.bash
W=${1:-bombsort}; G=${2:-1}; M=${3:-1}  # M: 0 なら MoveIt（rviz2 を含む）を立てない
date '+START %F %T' > /workspace/bombsort/gazebo.log
exec ros2 launch /workspace/bombsort/bombsort.launch.py world:=/workspace/bombsort/$W.sdf gui:=$G moveit:=$M >> /workspace/bombsort/gazebo.log 2>&1
