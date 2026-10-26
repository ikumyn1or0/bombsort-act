#!/bin/bash
# 画像入力の ACT を GPU・バッチ32で学習する。docker exec -d で呼ぶ。
# 引数: データセット名（~/dataset/lerobot_dataset/ の下）  周数  何周ごとに保存するか  [出力名]
# checkpoint があれば作り直さない。
DS=$1; EP=${2:-100}; SAVE=${3:-10}; OUT=${4:-${DS}_e${EP}}
D=/home/ubuntu/dataset/lerobot_dataset; C=/home/ubuntu/checkpoints; B=/workspace/bs
[ -e $C/$OUT/config.json ] || bash $B/px.sh ${OUT}_train $B/train.py --dataset-root $D/$DS \
  --features $B/features_img.py --device cuda --batch 32 --workers 6 --epochs $EP --save-every $SAVE --out $C/$OUT
echo "DONE $(date '+%F %T') UTC" >> $B/logs/${OUT}_train_done.log
