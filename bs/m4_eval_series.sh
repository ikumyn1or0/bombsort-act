#!/bin/bash
# 方策の途中の checkpoint を、同じ種で順に評価する。docker exec -d で呼ぶ。
# 引数: checkpoint の親ディレクトリ  評価名の接頭辞  種  N（組の数）  周数...（例: ... 10 20 30）
# 済んだ評価（結果の jsonl がある）は飛ばす。checkpoint がまだなければ、できるまで待つ（学習と並行して回すため）。
CK=$1; PFX=$2; SEED=$3; N=$4; shift 4
B=/workspace/bs
for E in "$@"; do
  NAME=${PFX}_ep${E}_s${SEED}
  until [ -e $CK/ep$E/model.safetensors ]; do sleep 30; done
  sleep 5
  [ -e $B/results/$NAME.jsonl ] || bash $B/m4_eval.sh $NAME $CK/ep$E $N $SEED
  echo "EVAL_DONE $NAME $(date '+%F %T') UTC" >> $B/logs/${PFX}_series_done.log
done
echo "DONE $(date '+%F %T') UTC" >> $B/logs/${PFX}_series_done.log
