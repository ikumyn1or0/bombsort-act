#!/bin/bash
# 画像1枚の方策を、静止した複数の物体（一括式・1組3個・間隔 7 cm・1個ごとの制限 15 秒）で N 組評価する。docker exec -d で呼ぶ。
# 引数: 名前  checkpoint のディレクトリ  N（組の数）  乱数の種（評価は 100 番台）  [bag の出力先]
NAME=$1; CK=$2; N=${3:-20}; SEED=${4:-100}; BAG=$5
export WORLD=${WORLD:-bombsort_v6m} MOVEIT=${MOVEIT:-0}
export REF_ARGS=${REF_ARGS:---k 3 --time-limit 15}
export AGENT_CMD="bash /workspace/bs/agent.sh $CK image /workspace/bs/logs/${NAME}_trace.jsonl"
bash /workspace/bs/run.sh $NAME $N multi $SEED $BAG
