#!/bin/bash
# run.sh の AGENT_CMD から呼ぶ。pixi 環境で推論ノード（infer.py）を回す。
# 引数: checkpoint のディレクトリ  入力(state|image)  trace の出力先
source /opt/ros/humble/setup.bash; source ~/airobot_ws/install/setup.bash
cd ~/pai_training
exec ~/.pixi/bin/pixi run python /workspace/bs/infer.py --model-path "$1" --mode "$2" --device cuda --trace "$3" $INFER_ARGS  # INFER_ARGS: 例 --exec ensemble
