#!/bin/bash
# pai_training の pixi 環境で python を時間計測つきで回す。docker exec -d で呼ぶ。
# 引数: ログ名（/workspace/bs/logs/<名前>.log に追記） python の引数...
NAME=$1; shift
L=/workspace/bs/logs/$NAME.log
source /opt/ros/humble/setup.bash
cd ~/pai_training
export WANDB_MODE=offline
echo "START $(date '+%F %T') UTC args: $*" >> $L
s=$(date +%s)
~/.pixi/bin/pixi run python "$@" >> $L 2>&1
rc=$?
echo "END $(date '+%F %T') UTC rc=$rc elapsed=$(( $(date +%s)-s ))s" >> $L
