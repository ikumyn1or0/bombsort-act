#!/bin/bash
# 画像1枚の方策を、録画用カメラつきのワールドで、静止した複数の物体・種 100 の最初の N 組だけ動かし、mp4 にする。docker exec -d で呼ぶ。
# 引数: 名前  checkpoint のディレクトリ  N（組の数、既定 3）  見出し
NAME=$1; CK=$2; N=${3:-3}; LABEL=${4:-image policy, 3 static objects}
export WORLD=bombsort_v6m_view BAG_EXTRA=/bs_camera2
R=/home/ubuntu/dataset/rosbags; B=/workspace/bs
[ -e $B/results/$NAME.jsonl ] || bash $B/m4_eval.sh $NAME $CK $N 100 $R/$NAME
bash $B/px.sh ${NAME}_mp4 $B/bag2mp4.py --bag $R/$NAME --out $B/viz/$NAME.mp4 --label "$LABEL"
ffmpeg -y -loglevel error -i $B/viz/$NAME.mp4 -c:v libx264 -pix_fmt yuv420p $B/viz/${NAME}_h264.mp4  # mp4v は再生できない環境があるので変換する
echo "DONE $(date '+%F %T') UTC" >> $B/logs/${NAME}_video_done.log
