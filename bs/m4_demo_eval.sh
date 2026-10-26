#!/bin/bash
# お手本のスクリプトを、静止した複数の物体（一括式・1組3個・間隔 7 cm・制限 15 秒）で N 組評価する。docker exec -d で呼ぶ。
# 引数: 名前  組の数（既定 20）  乱数の種  [bag の出力先]
NAME=$1; N=${2:-20}; SEED=${3:-105}; BAG=$4
export WORLD=${WORLD:-bombsort_v6m}
export REF_ARGS=${REF_ARGS:---k 3 --time-limit 15} DEMO_ARGS=${DEMO_ARGS:---carry-z 0.10}
bash /workspace/bs/run.sh $NAME $N multi $SEED $BAG
