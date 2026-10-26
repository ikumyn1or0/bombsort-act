#!/usr/bin/env python3
# 元のファイル: matsuolab/pai_training（コミット b4d6b54） の scripts/infer.py
# pai_training には LICENSE がないため、このファイルは本リポジトリの MIT ライセンスの対象外。
# 変えた点:
#  - 指令は sim 時刻の 20 Hz で出す（お手本 demo.py と変換が sim 時刻の 20 Hz なので、そろえる。元は壁時計のタイマー）
#  - 起動したら、まず初期姿勢（bs_common.HOME_Q、手は開く）へ 2 秒かけて補間して送る。
#    審判は腕が初期姿勢に着くまで最初の1個を出さず、評価の始めに腕を初期姿勢へ送るのは方策側の仕事にしてあるため
#  - 初期姿勢に着いたあとは、最初の観測（/bs_obj か /bs_camera）が届くまで初期姿勢を出し続け、届いたら方策に切り替える
#  - 毎周期の観測と指令を --trace（jsonl）に書く（失敗の切り分け用）
# 使い方（pixi 環境で）: python infer.py --model-path <checkpoint> --mode state|image --device cuda [--trace path]
import argparse, json, threading, time
import numpy as np
import torch
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lerobot.common.policies.act.modeling_act import ACTPolicy
from bs_common import ARM, HAND, HOME_Q, G_OPEN

PERIOD = 0.05
HOME_DUR = 2.0


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


class Infer(Node):
    def __init__(self, a, policy, dev):
        # sim 時刻は /clock（use_sim_time）ではなく /joint_states の stamp から読む。/clock は Gazebo の毎ステップ届き、
        # Python の受信処理が推論と GIL を取り合った（最初の評価で推論が中央値 90 ms。Gazebo なしでは 3.8 ms）
        super().__init__('bs_infer', parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.a, self.policy, self.dev = a, policy, dev
        self.arm_pub = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.js = {}
        self.t_js = 0.0
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.extra = None
        self.wrist = 0  # 手首カメラを使わない方策では、手首の画像を待たない
        if a.mode == 'image':
            self.create_subscription(Image, '/bs_camera', self.on_img, 2)
            if 'observation.images.wrist' in policy.config.image_features:  # 手首カメラ（試した版。提出の版では使わない）
                self.wrist = None
                self.create_subscription(Image, '/bs_wrist', lambda m: setattr(
                    self, 'wrist', np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()), 2)
        else:
            self.create_subscription(Float32MultiArray, '/bs_obj', lambda m: setattr(self, 'extra', list(m.data)), 10)
        self.ep = -1
        self.chunks = []
        self.create_subscription(String, '/bs_episode', self.on_ep, 10)
        self.trace = open(a.trace, 'a') if a.trace else None

    def on_js(self, m):
        self.js.update(zip(m.name, m.position))
        self.t_js = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def now(self):
        return self.t_js

    def log(self, s):
        print(f'[sim {self.now():8.3f}] {s}', flush=True)

    def on_img(self, m):
        self.extra = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()
        # 少し前の画像を取り出せるように、受けた時刻（sim）つきで 2 秒ぶん残す（変換は bag の受信時刻で選んでいる）
        self.imgs = [(t, im) for t, im in getattr(self, 'imgs', []) if self.t_js - t <= 2.0] + [(self.t_js, self.extra)]

    def prev_img(self):
        """--prev-dt 秒前までに届いた最後の画像。まだなければ一番古いもの（変換の直前値保持と同じ扱い）。"""
        old = [im for t, im in self.imgs if t <= self.now() - self.a.prev_dt]
        return old[-1] if old else self.imgs[0][1]

    def on_ep(self, m):
        e = json.loads(m.data)
        if e.get('ev') == 'start':
            self.ep = e['ep']
            if 'color' in e:
                self.log(f'ep{self.ep} start {e["color"]} ({e["x"]:.3f},{e["y"]:.3f})')
            else:  # 複数の物体の組の start には color がない（物体は objs の一覧）
                self.log(f'ep{self.ep} start {e.get("mode")} {len(e.get("objs", []))} objs')
        elif e.get('ev') == 'outcome':
            self.log(f'ep{e["ep"]} outcome ok={e["ok"]} reason={e["reason"]} t_obj={e["t_obj"]}')

    def send(self, q, g):
        for pub, names, pos in [(self.arm_pub, ARM, q), (self.grip_pub, [HAND], [g])]:
            m = JointTrajectory(); m.joint_names = names
            p = JointTrajectoryPoint(); p.positions = [float(v) for v in pos]
            p.time_from_start.sec = 0; p.time_from_start.nanosec = int(0.1 * 1e9)
            m.points = [p]; pub.publish(m)

    def act(self):
        state = torch.tensor([[self.js[n] for n in ARM] + [self.js[HAND]]], dtype=torch.float32, device=self.dev)
        obs = {'observation.state': state}
        if self.a.mode == 'image':
            to_t = lambda im: torch.from_numpy(im).permute(2, 0, 1).float().div(255).unsqueeze(0).to(self.dev)
            obs['observation.images.front'] = to_t(self.extra)
            if 'observation.images.wrist' in self.policy.config.image_features:  # 手首カメラ
                obs['observation.images.wrist'] = to_t(self.wrist)
            if 'observation.images.prev' in self.policy.config.image_features:
                obs['observation.images.prev'] = to_t(self.prev_img())
        else:
            obs['observation.environment_state'] = torch.tensor([self.extra], dtype=torch.float32, device=self.dev)
        with torch.no_grad():
            if self.a.exec == 'single':
                return self.policy.select_action(obs).cpu().numpy().flatten(), state.cpu().numpy().flatten()
            return self.ensemble(obs), state.cpu().numpy().flatten()

    def chunk(self, obs):
        """予測の列を丸ごと取り出す（select_action の中身と同じ正規化。(chunk, 5)、k 番目は k*action_dt 秒後の指令）。"""
        p = self.policy
        b = p.normalize_inputs(obs)
        if p.config.image_features:
            b = dict(b); b['observation.images'] = [b[k] for k in p.config.image_features]
        return p.unnormalize_outputs({'action': p.model(b)[0]})['action'][0].cpu().numpy()

    def ensemble(self, obs):
        """ACT の時間方向の重ね合わせ（temporal ensembling）。--replan 秒ごとに予測し直し、いまの時刻に当たる指令を
        有効な列ごとに時刻で補間して、古い列ほど重く（w = exp(-coeff * i)、i=0 が最も古い）平均する。
        LeRobot の ACTTemporalEnsembler は「列の刻み＝制御の刻み」前提だが、ここは列が 0.1 秒刻み・制御が 20 Hz なので自前で持つ。"""
        t = self.now()
        dt, n = self.a.action_dt, self.policy.config.chunk_size
        if not self.chunks or t - self.chunks[-1][0] >= self.a.replan - 1e-6:
            self.chunks.append((t, self.chunk(obs)))
        self.chunks = [(t0, c) for t0, c in self.chunks if t - t0 <= (n - 1) * dt]
        vals = []
        for t0, c in self.chunks:
            u = (t - t0) / dt
            i = min(int(u), n - 2)
            f = min(u - i, 1.0)
            vals.append(c[i] * (1 - f) + c[i + 1] * f)
        w = np.exp(-self.a.ens_coeff * np.arange(len(vals)))
        return (np.array(vals) * w[:, None]).sum(0) / w.sum()

    def run(self):
        while self.now() < 1.0 or any(n not in self.js for n in ARM + [HAND]):
            time.sleep(0.05)
        q0 = [self.js[n] for n in ARM]
        t_home = self.now()
        self.log(f'homing from {np.round(q0, 3).tolist()} to {np.round(HOME_Q, 3).tolist()}')
        t_next = self.now()
        mode, n_steps, slow = 'homing', 0, []
        while rclpy.ok():
            time.sleep(0.001)
            t = self.now()
            if t < t_next:
                continue
            t_next += PERIOD
            if t_next < t:  # sim 時刻が飛んだら追いつかせる（指令をまとめて出さない）
                t_next = t + PERIOD
            if mode == 'homing':
                s = smooth((t - t_home) / HOME_DUR)
                self.send([a + (b - a) * s for a, b in zip(q0, HOME_Q)], G_OPEN)
                if s >= 1.0 and self.extra is not None and self.wrist is not None:
                    mode = 'policy'
                    self.policy.reset()
                    self.chunks = []
                    self.log(f'policy start (exec={self.a.exec})')
                continue
            w = time.perf_counter()
            action, state = self.act()
            dt = time.perf_counter() - w
            self.send(action[:4], action[4])
            n_steps += 1
            if dt > 0.04:
                slow.append(dt)
            if self.trace:
                ex = self.extra if self.a.mode == 'state' else None
                self.trace.write(json.dumps(dict(t=round(t, 3), ep=self.ep, s=np.round(state, 4).tolist(),
                                                 a=np.round(action, 4).tolist(), o=ex, ms=round(dt * 1000, 1))) + '\n')
            if n_steps % 200 == 0:
                self.log(f'steps={n_steps} last_infer={dt*1000:.0f}ms slow(>40ms)={len(slow)} action={np.round(action, 3).tolist()}')
                if self.trace:
                    self.trace.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model-path', required=True)
    ap.add_argument('--mode', choices=['image', 'state'], default='state')
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--trace', default='')
    ap.add_argument('--exec', choices=['single', 'ensemble'], default='ensemble',  # 2026-10-16 に既定を重ね合わせにした
                    help='single: 毎周期、列の先頭だけを実行（元の infer.py の動き）／ensemble: 列を時間方向に重ね合わせて実行')
    ap.add_argument('--action-dt', type=float, default=0.1, help='予測の列の刻み（学習の --action-dt と同じ値）')
    ap.add_argument('--replan', type=float, default=0.1, help='[ensemble] 予測し直す間隔（秒、sim）')
    ap.add_argument('--prev-dt', type=float, default=0.4, help='[image] 方策が observation.images.prev を取るとき、何秒前の画像を渡すか（変換の --prev-dt と同じ値）')
    ap.add_argument('--ens-coeff',type=float, default=0.01, help='[ensemble] 重みの係数（ACT の原論文の既定は 0.01）')
    a = ap.parse_args()
    dev = torch.device(a.device)
    policy = ACTPolicy.from_pretrained(a.model_path)
    policy.eval(); policy.to(dev)
    rclpy.init()
    n = Infer(a, policy, dev)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        n.run()
    except KeyboardInterrupt:
        pass
    finally:
        if n.trace:
            n.trace.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
