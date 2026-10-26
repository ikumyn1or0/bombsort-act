#!/bin/bash
# 提出の版の最終評価（2026-10-22）。審判は A案（組を出してから 45 秒で打ち切る）・吸い寄せの上限 10 mm・種 110・20 組。
# 128×128 の版（提出の版、10 周）と 256×256 の版（比べるために試した版。checkpoint は公開しない）を評価し、128 の版を録画する。
# docker exec -d -u ubuntu pai_ros2_gpu bash /workspace/bs/t9d_am10.sh で呼ぶ。進み具合は logs/t9d_am10.log。
B=/workspace/bs; C=/home/ubuntu/checkpoints; R=/home/ubuntu/dataset/rosbags
L=$B/logs/t9d_am10.log
RULE="--k 3 --time-limit 15 --placement dir --balance --batch-limit 45 --attach-max 0.010"
log() { echo "$(date '+%F %T') UTC $*" >> $L; }
summ() { python3 -c "import json;s=json.load(open('$B/results/$1_summary.json'));print('組',s['success'],'/',s['n'],'個',s['obj_success'],'/',s['n_obj'])" 2>/dev/null; }
log "start"
n=t9d_old_am10_bl45_s110
[ -e $B/results/${n}_summary.json ] || WORLD=bombsort_v6m REF_ARGS="$RULE" bash $B/m4_eval.sh $n $C/bs_m4b_s41_img_e30/ep10 20 110
log "$n：$(summ $n)"
n=t9d_new_ep20_am10_bl45_s110
[ -e $B/results/${n}_summary.json ] || WORLD=bombsort_v6m256 REF_ARGS="$RULE" bash $B/m4_eval.sh $n $C/bs_t9b_s42_img256_e30/ep20 20 110
log "$n：$(summ $n)"
n=t9dv_old_am10_bl45
if [ ! -e $B/viz/${n}_h264.mp4 ]; then
  [ -e $B/results/$n.jsonl ] || WORLD=bombsort_v6m_view BAG_EXTRA=/bs_camera2 REF_ARGS="$RULE" bash $B/m4_eval.sh $n $C/bs_m4b_s41_img_e30/ep10 3 100 $R/$n
  bash $B/px.sh ${n}_mp4 $B/bag2mp4.py --bag $R/$n --out $B/viz/$n.mp4 --label "image ACT (10 epochs), snap 10 mm, 45 s per set"
  ffmpeg -y -loglevel error -i $B/viz/$n.mp4 -c:v libx264 -pix_fmt yuv420p $B/viz/${n}_h264.mp4
fi
log "$n：$(summ $n) viz/${n}_h264.mp4"
log "end"
