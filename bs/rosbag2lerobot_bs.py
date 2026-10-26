#!/usr/bin/env python3
# 元のファイル: matsuolab/pai_training（コミット b4d6b54） の scripts/rosbag2lerobot.py
# pai_training には LICENSE がないため、このファイルは本リポジトリの MIT ライセンスの対象外。
# 変えた点:
#  - 1つの bag に連続形式の何本ものエピソードが入っている前提。審判の /bs_episode の start〜end で切る
#    （物体が出た瞬間から、初期姿勢に戻り着いた瞬間まで）。最後のフレームで埋めない
#  - 成功し、かつ初期姿勢に戻り着いたエピソードだけを使う
#  - 標本化は sim 時刻の 20 Hz。bag の受信時刻（壁時計）は、/bs_status に入っている sim 時刻で sim 時刻へ直す
#    （お手本の指令は sim 時刻の 20 Hz で出ている。壁時計で切ると実時間比のぶん約 7% ずれる）
#  - 値は直前値保持（元の変換と同じ）。関節は名前で joint1..4, hand にそろえる
#  - --mode image：observation.images.front に /bs_camera ／ --mode state：observation.environment_state に /bs_obj
#    （/bs_obj は着地したあと次を出すまで着地点のまま。審判がそう出している）
# 使い方: python rosbag2lerobot_bs.py --bags <bag のディレクトリ> [...] --out <出力先> --features <features_*.py> --mode image|state
import argparse, importlib.util, json, time
from pathlib import Path
import numpy as np
import torch
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

JOINTS = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4', 'crane_plus_joint_hand']
ARM_T = '/crane_plus_arm_controller/joint_trajectory'
GRIP_T = '/crane_plus_gripper_controller/joint_trajectory'


def load_config_py(path):
    spec = importlib.util.spec_from_file_location('features_cfg', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def hold(times, values, target):
    """直前値保持。target より前に値がなければ最初の値を使う。"""
    idx = np.searchsorted(times, target, side='right') - 1
    return values[np.clip(idx, 0, len(values) - 1)]


def read_bag(bag, mode):
    want = {'/joint_states', ARM_T, GRIP_T, '/bs_status', '/bs_episode', '/bs_camera' if mode == 'image' else '/bs_obj'}
    msgs = {t: [] for t in want}
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic in want]
        for c, ts, raw in r.messages(connections=conns):
            msgs[c.topic].append((ts, deserialize_cdr(raw, c.msgtype)))
    # 壁時計（bag の受信時刻、ns）→ sim 時刻（s）
    st = [(ts, json.loads(m.data)['t']) for ts, m in msgs['/bs_status']]
    w = np.array([s[0] for s in st], dtype=np.float64); s = np.array([s[1] for s in st])
    to_sim = lambda ts: np.interp(np.asarray(ts, dtype=np.float64), w, s)

    out = {}
    js = msgs['/joint_states']
    out['js'] = (to_sim([t for t, _ in js]),
                 np.array([[dict(zip(m.name, m.position)).get(n, 0.0) for n in JOINTS] for _, m in js], dtype=np.float32))
    a = msgs[ARM_T]
    out['arm'] = (to_sim([t for t, _ in a]),
                  np.array([[dict(zip(m.joint_names, m.points[0].positions)).get(n, 0.0) for n in JOINTS[:4]] for _, m in a], dtype=np.float32))
    g = msgs[GRIP_T]
    out['grip'] = (to_sim([t for t, _ in g]), np.array([m.points[0].positions[0] for _, m in g], dtype=np.float32))
    if mode == 'image':
        im = msgs['/bs_camera']
        out['extra'] = (to_sim([t for t, _ in im]),
                        np.stack([np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3) for _, m in im]))
    else:
        ob = msgs['/bs_obj']
        out['extra'] = (to_sim([t for t, _ in ob]), np.array([list(m.data) for _, m in ob], dtype=np.float32))
    ev = [json.loads(m.data) for _, m in msgs['/bs_episode']]
    starts = {e['ep']: e for e in ev if e['ev'] == 'start'}
    ends = {e['ep']: e for e in ev if e['ev'] == 'end'}
    eps = []
    for k in sorted(starts):
        if k not in ends:
            print(f'  ep{k}: end がない（記録の途中で止まった）-> 使わない')
            continue
        e = ends[k]
        if not (e['ok'] and e['home']):
            print(f'  ep{k}: ok={e["ok"]} reason={e["reason"]} home={e["home"]} -> 使わない')
            continue
        eps.append((k, starts[k]['t'], e['t'], e))
    return out, eps


def cut(out, t0, t1, freq, prev_dt=0.0):
    n = int(np.floor((t1 - t0) * freq + 1e-6)) + 1
    tt = t0 + np.arange(n) / freq
    state = hold(*out['js'], tt)
    action = np.hstack([hold(*out['arm'], tt), hold(*out['grip'], tt)[:, None]])
    extra = hold(*out['extra'], tt)
    # prev_dt 秒前の画像（物体の動く向きを方策が読めるように。ACT は観測1回ぶんしか入れられない）。
    # エピソードの始めは、物体が出る前の画像になる（推論でも同じになる）
    prev = hold(*out['extra'], tt - prev_dt) if prev_dt > 0 else None
    return state, action, extra, prev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bags', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--features', required=True)
    ap.add_argument('--mode', choices=['image', 'state'], required=True)
    ap.add_argument('--freq', type=int, default=20)
    ap.add_argument('--prev-dt', type=float, default=0.0, help='[image] >0 なら、この秒数だけ前の画像を observation.images.prev に入れる（提出の版では使わない）')
    a = ap.parse_args()
    cfg = load_config_py(a.features)
    t0 = time.time()
    episodes = []
    for bag in a.bags:
        s = time.time()
        out, eps = read_bag(bag, a.mode)
        for k, ts, te, e in eps:
            episodes.append((bag, k, cut(out, ts, te, a.freq, a.prev_dt if a.mode == 'image' else 0.0)))
        print(f'read {bag}: {len(eps)} episodes used ({time.time() - s:.1f}s)')
    t_read = time.time() - t0
    ds = LeRobotDataset.create(repo_id=cfg.repo_id, fps=a.freq, root=Path(a.out), robot_type='crane_plus',
                               features=cfg.features, use_videos=True)
    t1 = time.time()
    total = 0
    lens = []
    for bag, k, (state, action, extra, prev) in episodes:
        for i in range(len(state)):
            f = {'observation.state': torch.from_numpy(state[i]), 'action': torch.from_numpy(action[i]), 'task': cfg.task_name}
            if a.mode == 'image':
                f['observation.images.front'] = extra[i]
                if prev is not None:
                    f['observation.images.prev'] = prev[i]
            else:
                f['observation.environment_state'] = torch.from_numpy(extra[i])
            ds.add_frame(f)
        ds.save_episode()
        total += len(state); lens.append(len(state))
    print(f'TIMING read={t_read:.1f}s write+encode={time.time() - t1:.1f}s total={time.time() - t0:.1f}s '
          f'frames={total} episodes={len(lens)} len_min={min(lens)} len_max={max(lens)} len_mean={np.mean(lens):.1f}')
    print(f'Dataset saved to: {ds.root}')


if __name__ == '__main__':
    main()
