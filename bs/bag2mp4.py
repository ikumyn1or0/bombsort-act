#!/usr/bin/env python3
# 録画用カメラ（/bs_camera2）を記録した bag から、目視用の mp4 を作る。sim 時刻の 15 fps で並べるので、再生は sim の実時間どおり。
# 画面の左上に、名前・sim 時刻・何個目か・審判の状態・直前の判定を焼き込む（文字は英語。コンテナに日本語のフォントがないため）。
# 使い方: python bag2mp4.py --bag <bag> --out <mp4> --label <見出し> [--speed 1.0]
import argparse, json
import numpy as np
import cv2
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--label', default=''); ap.add_argument('--fps', type=float, default=15)
    ap.add_argument('--speed', type=float, default=1.0, help='再生の速さ（2 なら sim の 2 倍速）')
    a = ap.parse_args()
    imgs, st, ev = [], [], []
    with Reader(a.bag) as r:
        conns = [c for c in r.connections if c.topic in ('/bs_camera2', '/bs_status', '/bs_episode')]
        for c, ts, raw in r.messages(connections=conns):
            m = deserialize_cdr(raw, c.msgtype)
            if c.topic == '/bs_camera2':
                imgs.append((ts, np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)))
            elif c.topic == '/bs_status':
                st.append((ts, json.loads(m.data)))
            else:
                ev.append(json.loads(m.data))
    w = np.array([t for t, _ in st], dtype=np.float64); s = np.array([d['t'] for _, d in st])
    to_sim = lambda ts: float(np.interp(ts, w, s))
    tim = np.array([to_sim(t) for t, _ in imgs])
    sts = [d for _, d in st]
    outcomes = sorted([e for e in ev if e['ev'] == 'outcome'], key=lambda e: e['t'])
    t0 = next((e['t'] for e in ev if e['ev'] == 'start'), tim[0]) - 1.0
    h, wd = imgs[0][1].shape[:2]
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (wd, h))
    n = 0
    for t in np.arange(t0, tim[-1], a.speed / a.fps):
        i = max(0, np.searchsorted(tim, t, side='right') - 1)
        f = cv2.cvtColor(imgs[i][1], cv2.COLOR_RGB2BGR).copy()
        d = sts[max(0, np.searchsorted(s, t, side='right') - 1)]
        done = [o for o in outcomes if o['t'] <= t]
        ok = sum(o['ok'] for o in done)
        lines = [a.label, f'sim {t:6.1f}s  object #{d.get("ep", -1) + 1}  phase {d.get("phase", "")}',
                 f'result so far: {ok}/{len(done)} success']
        if done and 'k' in done[0]:  # 複数の物体：組ごとの成否と、組の中の何個目か
            ends = [e for e in ev if e['ev'] == 'end' and e['t'] <= t]
            lines[1] = f'sim {t:6.1f}s  set #{d.get("ep", -1) + 1}  objects done {d.get("k_done", 0)}  phase {d.get("phase", "")}'
            lines[2] = f'objects: {ok}/{len(done)} success   sets: {sum(e["ok"] for e in ends)}/{len(ends)} all-3 success'
            if t - done[-1]['t'] < 3.0:
                o = done[-1]
                lines.append(f'set #{o["ep"] + 1} obj {o["k"] + 1} ({o["color"][5:]}): '
                             f'{"SUCCESS" if o["ok"] else o["reason"].upper()}  ({o["t_obj"]:.1f}s)')
        elif done and t - done[-1]['t'] < 3.0:
            o = done[-1]
            lines.append(f'#{o["ep"] + 1}: {"SUCCESS" if o["ok"] else o["reason"].upper()}  ({o["t_obj"]:.1f}s)')
        for k, txt in enumerate(lines):
            y = 22 + 22 * k
            cv2.putText(f, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(f, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        vw.write(f); n += 1
    vw.release()
    print(f'wrote {a.out}: {n} frames ({n / a.fps:.1f}s), images in bag {len(imgs)}')


if __name__ == '__main__':
    main()
