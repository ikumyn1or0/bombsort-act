#!/bin/bash
# 静止した複数の物体の学習を通しで回す（提出の版の checkpoint bs_m4b_s41_img_e30 はこれで作った）。
# docker exec -d -u ubuntu pai_ros2_gpu bash /workspace/bs/m4b_pipeline.sh で呼ぶ。
#  置き方：向きを区別した条件＋手前・奥の形の均等化（審判の --placement dir --balance）。記録・評価・録画のすべてで同じ
#  1. お手本で 100 組を記録する（種 41、制限 15 秒）
#  2. 画像1枚＋関節角に変換する（組が成功し初期姿勢に戻ったエピソードだけ使われる）
#  3. 30 周学習する（GPU・バッチ32、5 周ごとに保存）。並行して 5〜30 周を評価する（種 100、N=20 組）
#  4. 最良の周（組の成功数、同点なら個の成功数、さらに同点なら若い周）を種 101 でも評価する
#  5. 最良の周で、録画用ワールドで種 100 の最初の3組を録画する
# 済んだ段は飛ばすので、落ちたら同じコマンドで続きから回る。ただし記録が途中で落ちた場合は
# results/m4b_rec_s41.jsonl が途中までで残るので、別名に写して空にしてから回し直す。進み具合は logs/m4b_pipeline.log。
B=/workspace/bs; C=/home/ubuntu/checkpoints; D=/home/ubuntu/dataset/lerobot_dataset; R=/home/ubuntu/dataset/rosbags
L=$B/logs/m4b_pipeline.log
DS=bs_m4b_s41_img; CK=$C/${DS}_e30
export REF_ARGS="--k 3 --time-limit 15 --placement dir --balance"
log() { echo "$(date '+%F %T') UTC $*" >> $L; }
EPS="5 10 15 20 25 30"
log "start"

# 1. 記録
[ -e $B/results/m4b_rec_s41.jsonl ] || WORLD=bombsort_v6m DEMO_ARGS="--carry-z 0.10" bash $B/run.sh m4b_rec_s41 100 multi 41 $R/bs_m4b_s41
log "記録：$(python3 -c "import json;s=json.load(open('$B/results/m4b_rec_s41_summary.json'));print('組',s['success'],'/',s['n'],'個',s['obj_success'],'/',s['n_obj'])")"

# 2. 変換
[ -d $D/$DS ] || bash $B/px.sh ${DS}_convert $B/rosbag2lerobot_bs.py --bags $R/bs_m4b_s41 --out $D/$DS --features $B/features_img.py --mode image
log "変換：$(grep -o 'frames=.*' $B/logs/${DS}_convert.log | tail -1)"

# 3. 学習と、並行して評価
bash $B/m4_eval_series.sh $CK m4b 100 20 $EPS &
SER=$!
bash $B/m2_train.sh $DS 30 5
log "学習が終わった"
wait $SER

# 4. 最良の周を種 101 で
read S O N <<< "$(python3 - $EPS <<'PY'
import json, os, sys
b = None
for e in sys.argv[1:]:
    p = f'/workspace/bs/results/m4b_ep{e}_s100_summary.json'
    if os.path.exists(p):
        s = json.load(open(p)); k = (s['success'], s['obj_success'], -int(e))
        b = k if b is None or k > b else b
print(*(b[0], b[1], -b[2]) if b else (0, 0, 0))
PY
)"
log "最良：${N} 周（組 $S/20、個 $O/60、種 100）"
[ "$O" -gt 0 ] && bash $B/m4_eval_series.sh $CK m4b 101 20 $N
log "種 101：$(python3 -c "import json;s=json.load(open('$B/results/m4b_ep${N}_s101_summary.json'));print('組',s['success'],'/ 20','個',s['obj_success'],'/ 60')" 2>/dev/null)"

# 5. 録画
[ "$O" -gt 0 ] && bash $B/m4_video.sh m4bv_ep${N} $CK/ep$N 3 "image ACT ($N epochs), 3 static objects, varied layouts"
log "録画：$(python3 -c "import json;s=json.load(open('$B/results/m4bv_ep${N}_summary.json'));print('組',s['success'],'/',s['n'],'個',s['obj_success'],'/',s['n_obj'])" 2>/dev/null) viz/m4bv_ep${N}_h264.mp4"
log "end"
