#!/bin/bash
# 連続形式の評価（と、お手本の記録）を1回まわす。docker exec -d -u ubuntu pai_ros2_gpu bash /workspace/bs/run.sh ... で呼ぶ。
# 引数: 名前 N 段(static|reflect) 乱数の種 [bag の出力先（記録するときだけ）]
# 動かす側は既定でお手本（demo.py）。方策を評価するときは AGENT_CMD に推論のコマンドを入れる（初期姿勢へ送るのは方策側の仕事）。
# 結果: /workspace/bs/results/<名前>.jsonl（1個1行）と <名前>_summary.json。ログ: /workspace/bs/logs/<名前>_*.log
NAME=$1; N=${2:-20}; MODE=${3:-static}; SEED=${4:-0}; BAG=$5
B=/workspace/bs; L=$B/logs/${NAME}_run.log
mkdir -p $B/logs $B/results
source /opt/ros/humble/setup.bash; source ~/airobot_ws/install/setup.bash
cd $B
echo "START $(date '+%F %T') UTC name=$NAME n=$N mode=$MODE seed=$SEED bag=$BAG" >> $L
if [ -e $B/results/$NAME.jsonl ]; then echo "results/$NAME.jsonl が既にある。名前を変えるか消してから" >> $L; exit 1; fi
bash /workspace/bs/down.sh >> $L 2>&1
setsid bash /workspace/bombsort/up.sh ${WORLD:-bombsort_v5} 0 ${MOVEIT:-0} &  # 既定は MoveIt と rviz2 を立てない（MOVEIT=1 で立てる）。WORLD=bombsort_v6m_view で録画用カメラつき
s=$(date +%s)
for i in $(seq 1 45); do
  sleep 2
  ros2 control list_controllers 2>/dev/null | grep -q crane_plus_gripper_controller && break
done
sleep 5
bash /workspace/bombsort/ctrl.sh >> $L 2>&1
echo "ready after $(( $(date +%s)-s ))s" >> $L
if [ -n "$BAG" ]; then
  setsid ros2 bag record -o $BAG /joint_states /crane_plus_arm_controller/joint_trajectory \
    /crane_plus_gripper_controller/joint_trajectory /bs_camera /bs_obj /bs_status /bs_episode $BAG_EXTRA > $B/logs/${NAME}_bag.log 2>&1 &
  BAGPID=$!
  sleep 4
fi
if [ -n "$AGENT_CMD" ]; then
  setsid bash -c "$AGENT_CMD" > $B/logs/${NAME}_agent.log 2>&1 &
else
  setsid python3 $B/demo.py --mode $MODE $DEMO_ARGS > $B/logs/${NAME}_agent.log 2>&1 &
fi
AGPID=$!
s2=$(date +%s)
python3 $B/referee.py --n $N --mode $MODE --seed $SEED --out $B/results/$NAME.jsonl --tag $NAME $REF_ARGS > $B/logs/${NAME}_referee.log 2>&1  # REF_ARGS: 例 --time-limit 15（1回目で掴めた成功だけを数える基準）
echo "referee rc=$? wall=$(( $(date +%s)-s2 ))s" >> $L
kill -INT -- -$AGPID 2>/dev/null; sleep 1; kill -- -$AGPID 2>/dev/null
if [ -n "$BAG" ]; then kill -INT -- -$BAGPID 2>/dev/null; sleep 5; du -sh $BAG >> $L; fi
bash /workspace/bs/down.sh >> $L 2>&1
cat $B/results/${NAME}_summary.json >> $L 2>/dev/null; echo >> $L
echo "END $(date '+%F %T') UTC" >> $L
