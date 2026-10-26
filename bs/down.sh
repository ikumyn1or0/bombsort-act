#!/bin/bash
# ワールドと関連プロセスを確実に止める。bombsort/down.sh は SIGTERM で ign gazebo が残ることがあった（2026-10-16、
# 残った Gazebo と次の起動がぶつかり、コントローラ待ちで止まった）ので、残っていれば SIGKILL まで送る。
P='ros2 launch /workspace/bombsort|ign gazebo|parameter_bridge|move_group|robot_state_publisher|ruby.*ign|controller_manager|spawner|ros2 bag record|/workspace/bs/(demo|referee|infer)\.py|pose_relay'
pkill -INT -f 'ros2 launch /workspace/bombsort'; pkill -INT -f 'ros2 bag record'; sleep 3
pkill -f "$P"; sleep 3
pkill -9 -f "$P"; sleep 1
n=$(pgrep -f "$P" | wc -l)
echo "down: left=$n"
