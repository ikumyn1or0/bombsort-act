# 審判（物体を出し、成否を判定するノード）。お手本とも方策とも組める。
# 使い方: python3 referee.py --n 20 --mode static --seed 0 --out /workspace/bs/results/<名前>.jsonl
#
# 連続形式：腕が初期姿勢にあるのを見て物体を1個出す → 掴み・運び・離すのを見て着地で判定 → 物体を消す
#   → 腕が初期姿勢に戻り着いたらそのエピソードを閉じ、同じ周期で次を出す。N 個で終わる。
# お手本・方策のどちらとも組めるように、固定（attach）と外し（detach）も審判が受け持つ。
#   固定：閉じる指令中・手が途中で止まっている・手先と物体の中心が 1.5 cm 以内・手先が低い → 物体を手先の真下へ寄せて固定
#   外し：固定中に開く指令が来たら外す（物体は落ちる）
# 出すトピック（20 Hz、sim 時刻）
#   /bs_obj      Float32MultiArray [x, y, 赤=1/黒=0]。着地したあとは、次を出すまで着地点のまま
#   /bs_status   String（JSON）。お手本のスクリプトが読む正しい状態（位置・速度・固定の有無・段階）
#   /bs_episode  String（JSON）。start / outcome / end のときだけ。変換はこれでエピソードを切る
import argparse, json, math, os, random, statistics, subprocess, threading, time
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, String, Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from bs_common import *  # noqa: F403

PERIOD = 0.05
CLOSE_SETTLE = 0.6


class Referee(Node):
    def __init__(self, a):
        super().__init__('bs_referee', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.a = a
        self.rng = random.Random(a.seed)
        self.multi = a.mode == 'multi'
        self.names = MULTI_NAMES if self.multi else sorted(ZONES)
        self.batch = None
        # --layout：JSON の [[ [x, y, "red"|"black"], ... ], ...]（組ごとの物体）。組の数は --n より優先する
        self.layout = json.load(open(a.layout)) if getattr(a, 'layout', None) else None
        if self.layout is not None:
            a.n = len(self.layout)
        self.att ={b: self.create_publisher(Empty, f'/{b}/attach', 10) for b in self.names}
        self.det = {b: self.create_publisher(Empty, f'/{b}/detach', 10) for b in self.names}
        self.obj_pub = self.create_publisher(Float32MultiArray, '/bs_obj', 10)
        self.status_pub = self.create_publisher(String, '/bs_status', 10)
        self.ep_pub = self.create_publisher(String, '/bs_episode', 10)
        self.js = {}
        self.create_subscription(JointState, '/joint_states', lambda m: self.js.update(zip(m.name, m.position)), 10)
        self.gcmd = G_OPEN
        self.t_close = 0.0
        self.miss_logged = False
        self.create_subscription(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', self.on_grip, 10)
        for b in self.names:
            self.create_subscription(String, f'/{b}/attach_state', lambda m, b=b: self.log(f'attach_state {b}: {m.data}'), 10)
        # set_pose は ROS から中継できない（humble の ros_gz_bridge は ControlWorld だけ）ので C++ の中継を使う
        self.pose = {}
        self.relay = subprocess.Popen(['/home/ubuntu/pose_relay'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.wlock = threading.Lock()
        threading.Thread(target=self.read_relay, daemon=True).start()

    # ---- 入出力 ----
    def on_grip(self, m):
        if m.points and m.points[0].positions:
            g = m.points[0].positions[0]
            if g > G_MID >= self.gcmd:
                self.t_close = self.now()
                self.miss_logged = False
            self.gcmd = g

    def read_relay(self):
        for line in self.relay.stdout:
            f = line.split()
            if len(f) >= 5 and f[0] == 'pose':  # 壊れた行は読み飛ばす（読むのが止まると中継が詰まる。v4 で発生）
                try:
                    self.pose[f[1]] = (float(f[2]), float(f[3]), float(f[4]) - BASE_Z)
                except ValueError:
                    pass

    def set_pose(self, name, x, y, z_world):
        with self.wlock:
            self.relay.stdin.write(f'{name} {x:.5f} {y:.5f} {z_world:.5f}\n')
            self.relay.stdin.flush()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, s):
        print(f'[sim {self.now():8.3f}] {s}', flush=True)

    def emit(self, **kw):
        kw['t'] = round(self.now(), 3)
        self.ep_pub.publish(String(data=json.dumps(kw)))
        self.log('EPISODE ' + json.dumps(kw, ensure_ascii=False))

    def endtip(self):
        ex, ey, ez, _ = forward_kinematics([self.js[n] for n in ARM])
        return ex, ey, ez

    # ---- 1個ぶん ----
    def spawn(self, k):
        b = self.rng.choice(sorted(ZONES))
        if self.a.mode == 'static':
            x, y = self.rng.uniform(FIELD[0], FIELD[1]), self.rng.uniform(FIELD[2], FIELD[3])
            v = (0.0, 0.0)
        else:
            # 反射の段：出る位置は範囲の縁のどこか、向きは乱数。手を出す前に壁で反射するのが見えるように
            x, y = self.rng.uniform(FIELD[0], FIELD[1]), self.rng.choice([FIELD[2], FIELD[3]])
            ang = math.radians(self.rng.uniform(20, 70))
            v = (self.a.speed * math.cos(ang) * self.rng.choice([1, -1]), self.a.speed * math.sin(ang) * (1 if y < 0 else -1))
        for o in ZONES:
            if o != b:
                self.set_pose(o, *PARK[o])
        self.set_pose(b, x, y, OBJ_Z)
        self.cur = dict(ep=k, b=b, x=x, y=y, x0=x, y0=y, v0=[round(v[0], 4), round(v[1], 4)], vx=v[0], vy=v[1], moving=self.a.mode != 'static',
                        attached=False, t0=self.now(), snap_tick=None, hand_hist=[], suction=None)
        self.last_obj = (x, y, 1.0 if b == 'bomb_red' else 0.0)
        self.phase = 'active'
        self.emit(ev='start', ep=k, color=b, x=round(x, 4), y=round(y, 4), vx=round(v[0], 4), vy=round(v[1], 4), mode=self.a.mode)

    def finish(self, ok, reason, land=None):
        c = self.cur
        b = c['b']
        el = self.now() - c['t0']
        if ok and el > self.a.time_limit:
            ok, reason = False, 'timeout'
        if c['attached']:
            self.det[b].publish(Empty())
        self.set_pose(b, *PARK[b])  # 着地した周期で消す（倒れて転がる様子は写さない）
        if land is not None:
            self.last_obj = (land[0], land[1], self.last_obj[2])
        self.result = dict(ep=c['ep'], color=b, ok=ok, reason=reason, t_obj=round(el, 3),
                           spawn=[round(c['x0'], 4), round(c['y0'], 4)],
                           land=[round(land[0], 4), round(land[1], 4)] if land else None,
                           suction=c['suction'], t_suction=c.get('t_suction'), v0=c['v0'])
        self.emit(ev='outcome', **self.result)
        self.phase = 'returning'
        self.t_outcome = self.now()

    def step_active(self):
        c = self.cur
        b = c['b']
        if b not in self.pose or any(n not in self.js for n in ARM + [HAND]):
            return
        ex, ey, ez = self.endtip()
        prev = getattr(self, 'prev_tip', (ex, ey, ez))
        ev = math.dist(prev, (ex, ey, ez)) / PERIOD
        self.prev_tip = (ex, ey, ez)
        hand = self.js[HAND]
        c['hand_hist'] = (c['hand_hist'] + [hand])[-4:]
        ox, oy, oz = self.pose[b]

        # 反射の段：20 Hz で位置を書き換え、壁で反射。開いた手先が低く止まっていて近ければ、手先の真下へ吸い寄せて止める
        if c['moving']:
            d = math.hypot(c['x'] - ex, c['y'] - ey)
            if hand <= G_OPEN + 0.15 and ez <= STOP_Z and ev < STILL_V and d <= STOP_DIST:
                self.set_pose(b, ex, ey, OBJ_Z)
                c['moving'] = False; c['x'], c['y'] = ex, ey
                c['suction'] = round(d, 4)
                c['t_suction'] = round(self.now() - c['t0'], 3)
                self.log(f'suction: pulled {d*1000:.1f}mm to ({ex:.3f},{ey:.3f}) endtip_z={ez:.3f} v={ev*100:.2f}cm/s hand={hand:.3f}')
            else:
                c['x'] += c['vx'] * PERIOD; c['y'] += c['vy'] * PERIOD
                if not FIELD[0] <= c['x'] <= FIELD[1]:
                    c['vx'] = -c['vx']; c['x'] = min(max(c['x'], FIELD[0]), FIELD[1])
                if not FIELD[2] <= c['y'] <= FIELD[3]:
                    c['vy'] = -c['vy']; c['y'] = min(max(c['y'], FIELD[2]), FIELD[3])
                self.set_pose(b, c['x'], c['y'], OBJ_Z)
            ox, oy = c['x'], c['y']

        # 固定：寄せる → 0.1 秒後に固定 → 0.3 秒後にもう一度（起動後の最初の固定が効かないことがあった）
        if c['snap_tick'] is not None:
            c['snap_tick'] += 1
            if c['snap_tick'] in (2, 6):
                self.att[b].publish(Empty())
            if c['snap_tick'] == 6:
                c['attached'] = True
                self.log(f'attached {b}')
        # 閉じる指令の直後は手がまだ動いておらず「止まっている」と読めてしまうので、指令から CLOSE_SETTLE 秒は判定しない
        elif (not c['attached'] and not c['moving'] and self.gcmd > G_MID and len(c['hand_hist']) == 4
              and self.now() - self.t_close >= CLOSE_SETTLE):
            still = max(c['hand_hist']) - min(c['hand_hist']) < 0.01
            gap = math.hypot(ox - ex, oy - ey)
            if still and hand <= G_CLOSE - 0.3 and gap <= ATTACH_MAX and ez <= ATTACH_EZ and oz < 0.03:
                self.set_pose(b, ex, ey, oz + BASE_Z)
                c['snap_tick'] = 0
                self.log(f'grasp: gap={gap*1000:.1f}mm hand={hand:.3f} endtip=({ex:.3f},{ey:.3f},{ez:.3f}) -> snap & attach')

        if c['attached'] and self.gcmd <= G_MID:
            self.det[b].publish(Empty())
            c['attached'] = False
            self.phase = 'falling'
            self.t_rel = self.now()
            self.log(f'release at ({ox:.3f},{oy:.3f},{oz:.3f})')
            return

        if not c['attached'] and c['snap_tick'] is None:
            if not (WALL[0] - OUT_MARGIN <= ox <= WALL[1] + OUT_MARGIN and WALL[2] - OUT_MARGIN <= oy <= WALL[3] + OUT_MARGIN):
                return self.finish(False, 'out_of_field', (ox, oy))
        if self.now() - c['t0'] > self.a.time_limit:
            return self.finish(False, 'timeout')
        self.last_obj = (ox, oy, self.last_obj[2])

    def step_falling(self):
        b = self.cur['b']
        ox, oy, oz = self.pose[b]
        self.last_obj = (ox, oy, self.last_obj[2])
        if oz <= LAND_Z or self.now() - self.t_rel > 1.0:
            zx, zy = ZONES[b]
            other = [o for o in ZONES if o != b][0]
            if abs(ox - zx) <= ZONE_HALF and abs(oy - zy) <= ZONE_HALF:
                ok, reason = True, 'success'
            elif abs(ox - ZONES[other][0]) <= ZONE_HALF and abs(oy - ZONES[other][1]) <= ZONE_HALF:
                ok, reason = False, 'wrong_zone'
            else:
                ok, reason = False, 'dropped'
            self.log(f'landed {self.now() - self.t_rel:.3f}s after release at ({ox:.3f},{oy:.3f},{oz:.3f}) -> {reason}')
            self.finish(ok, reason, (ox, oy))

    # ---- 静止した複数の物体（一括式） ----
    # k 個を同時に出し、k 個とも片付く（成功か失敗）までを1エピソードにする。組の中では腕が初期姿勢に戻るのを待たない。
    # 時間制限は1個ごと：前の個が片付いた時刻（1個目は組を出した時刻）から、次の個の着地まで。
    # 時間切れのときは、掴んでいる物体（なければ手先に一番近い物体）を失敗として消し、組の残りを続ける。
    # 置いてある物体が場外へ出たら、その物体を失敗として消す。組の成功＝k 個とも成功。
    # --batch-limit を与えたとき（A案）は1個ごとの制限をやめ、組を出してからその秒数で残りをすべて時間切れにする。
    # 1個ごとの制限では、時間切れの時刻から次の個を数え始めるので方策の動きの周期とずれ、1個の失敗が次の個の時間切れを招いていた
    # （種 110 の評価で、失敗はほぼ必ず2個続けて出た）。外した物体は場に残し、時間内なら掴み直せる。
    def spawn_batch(self, k):
        a = self.a
        if self.layout is not None:  # 決めた配置で出す（当たる範囲の測定用。--layout）
            pts = [(p[0], p[1]) for p in self.layout[k]]
            cols = [p[2] for p in self.layout[k]]
        elif a.placement == 'dir' or a.balance:  # 2026-10-20 の見直し（向きを区別した条件・手前と奥の形を均等に）
            pts = sample_layout(self.rng, k=a.k, placement=a.placement, balance=a.balance, min_dist=a.min_dist, min_dy=a.min_dy)
            cols = None
        else:
            for _ in range(10000):
                pts = [(self.rng.uniform(FIELD[0], FIELD[1]), self.rng.uniform(FIELD[2], FIELD[3])) for _ in range(a.k)]
                if all(math.dist(p, q) >= a.min_dist and abs(p[1] - q[1]) >= a.min_dy for i, p in enumerate(pts) for q in pts[i + 1:]):
                    break
            else:
                raise RuntimeError('置き方の条件を満たす配置が見つからない')
            cols = None
        used = {'red': 0, 'black': 0}
        objs = []
        for i, (x, y) in enumerate(pts):
            c = cols[i] if cols else self.rng.choice(['red', 'black'])
            objs.append(dict(name=f'bomb_{c}_{used[c]}', b=f'bomb_{c}', x0=x, y0=y, state='field', moved=0.0, attached=False))
            used[c] += 1
        live = {o['name'] for o in objs}
        for n in self.names:
            if n not in live:
                self.set_pose(n, *park_of(n))
        for o in objs:
            self.set_pose(o['name'], o['x0'], o['y0'], OBJ_Z)
        self.batch = dict(ep=k, objs=objs, t0=self.now(), results=[])
        self.cur, self.snap, self.hand_hist = None, None, []
        self.t_pick = self.now()
        self.phase = 'active'
        self.emit(ev='start', ep=k, mode='multi',
                  objs=[dict(name=o['name'], color=o['b'], x=round(o['x0'], 4), y=round(o['y0'], 4)) for o in objs])

    def resolve(self, o, ok, reason, land=None):
        bt = self.batch
        el = self.now() - self.t_pick
        if ok and (self.now() - bt['t0'] > self.a.batch_limit if self.a.batch_limit else el > self.a.time_limit):
            ok, reason = False, 'timeout'
        if o['attached']:
            self.det[o['name']].publish(Empty())
            o['attached'] = False
        self.set_pose(o['name'], *park_of(o['name']))
        o['state'] = 'done'
        r = dict(ep=bt['ep'], k=len(bt['results']), name=o['name'], color=o['b'], ok=ok, reason=reason, t_obj=round(el, 3),
                 t_batch=round(self.now() - bt['t0'], 3), spawn=[round(o['x0'], 4), round(o['y0'], 4)],
                 land=[round(land[0], 4), round(land[1], 4)] if land else None,
                 rule_ok=o.get('rule_ok'), moved_mm=round(o['moved'] * 1000, 1))
        bt['results'].append(r)
        self.emit(ev='outcome', **r)
        # 片付けたのが掴んでいる（寄せている）物体のときだけ、掴んでいる記録を消して次の個の時間を始める。
        # 掴んで運んでいる最中に隣が場外へ出たとき、掴んでいる記録まで消していて、その物体が判定されないまま
        # 審判が止まり続けた（測定 m4_probe1、4.7 時間）。隣が場外へ出ても、いま取っている個の時間は続ける
        # 時間切れは、いま取っている個の時間が尽きたということなので、消した物体がどれでも次の個の時間を始める
        # （これを抜いていたため、掴んでいないときの時間切れで残りの物体が同じ周期に続けて時間切れになった。記録 m4b_rec_s41）
        mine = self.cur is o or (self.snap is not None and self.snap[0] is o) or reason == 'timeout'
        if mine:
            self.cur, self.snap = None, None
            self.t_pick = self.now()
        if all(x['state'] == 'done' for x in bt['objs']):
            self.phase = 'returning'
            self.t_outcome = self.now()
        else:
            self.phase = 'falling' if self.cur is not None and self.cur['state'] == 'falling' else 'active'

    def end_batch(self, home):
        bt = self.batch
        rs = bt['results']
        bad = [r for r in rs if not r['ok']]
        e = dict(ep=bt['ep'], ok=not bad, reason=bad[0]['reason'] if bad else 'success', n_ok=len(rs) - len(bad),
                 home=home, t_ep=round(self.now() - bt['t0'], 3))
        self.emit(ev='end', **e)
        with open(self.a.out, 'a') as f:
            for r in rs:
                f.write(json.dumps(dict(r, batch_ok=e['ok'], home=home, run=self.a.tag, mode='multi', seed=self.a.seed)) + '\n')
        self.results.append(e)
        self.batch = None

    def step_multi(self):
        bt = self.batch
        live = [o for o in bt['objs'] if o['state'] != 'done']
        if any(n not in self.js for n in ARM + [HAND]) or any(o['name'] not in self.pose for o in live):
            return
        ex, ey, ez = self.endtip()
        hand = self.js[HAND]
        self.hand_hist = (self.hand_hist + [hand])[-4:]
        # 出した直後の数周期は、中継の位置がまだ待機位置を指す（試運転 m4d_try1 で、お手本が待機位置の物体を取りに行った）。
        # 出した位置の近くに一度来るまでは、その物体を場外・動いた量・お手本への公開の対象にしない
        for o in live:
            if not o.get('seen'):
                ox, oy, _ = self.pose[o['name']]
                o['seen'] = math.hypot(ox - o['x0'], oy - o['y0']) < 0.005
        field = [o for o in live if o['state'] == 'field' and o.get('seen')]
        for o in field:  # 置いてある物体が動いた量（隣に当たったかを見るため）
            if self.snap is not None and self.snap[0] is o:
                continue  # 固定の直前に手先の真下へ寄せた分は数えない（m4i_ep10_s100 では、これが moved_mm のほぼ全部だった）
            ox, oy, _ = self.pose[o['name']]
            o['moved'] = max(o['moved'], math.hypot(ox - o['x0'], oy - o['y0']))
        c = self.cur
        if self.snap is not None:  # 固定：単体のときと同じく、寄せる → 0.1 秒後に固定 → 0.3 秒後にもう一度
            o, tick = self.snap
            tick += 1
            self.snap = (o, tick)
            if tick in (2, 6):
                self.att[o['name']].publish(Empty())
            if tick == 6:
                o['attached'], o['state'] = True, 'held'
                self.cur, self.snap = o, None
                self.log(f'attached {o["name"]}')
        elif (c is None and field and self.gcmd > G_MID and len(self.hand_hist) == 4
              and self.now() - self.t_close >= CLOSE_SETTLE):
            o = min(field, key=lambda q: math.hypot(self.pose[q['name']][0] - ex, self.pose[q['name']][1] - ey))
            ox, oy, oz = self.pose[o['name']]
            still = max(self.hand_hist) - min(self.hand_hist) < 0.01
            gap = math.hypot(ox - ex, oy - ey)
            if still and hand <= G_CLOSE - 0.3 and gap > self.a.attach_max and ez <= ATTACH_EZ and oz < 0.03 and not self.miss_logged:
                self.miss_logged = True  # 上限を厳しくしたときに、外れた掴みの距離を数えるため
                self.log(f'grasp miss {o["name"]}: gap={gap*1000:.1f}mm > {self.a.attach_max*1000:.0f}mm')
            if still and hand <= G_CLOSE - 0.3 and gap <= self.a.attach_max and ez <= ATTACH_EZ and oz < 0.03:
                o['rule_ok'] = o is (max if self.a.rule == 'ymax' else min)(field, key=lambda q: self.pose[q['name']][1])  # お手本の規則どおりか
                self.set_pose(o['name'], ex, ey, oz + BASE_Z)
                self.snap = (o, 0)
                self.log(f'grasp {o["name"]}: gap={gap*1000:.1f}mm hand={hand:.3f} endtip=({ex:.3f},{ey:.3f},{ez:.3f}) -> snap & attach')

        if c is not None and c['state'] == 'held' and self.gcmd <= G_MID:
            self.det[c['name']].publish(Empty())
            c['attached'], c['state'] = False, 'falling'
            self.phase = 'falling'
            self.t_rel = self.now()
            ox, oy, oz = self.pose[c['name']]
            self.log(f'release {c["name"]} at ({ox:.3f},{oy:.3f},{oz:.3f})')
            return

        for o in field:
            if self.snap is not None and self.snap[0] is o:
                continue
            ox, oy, _ = self.pose[o['name']]
            if not (WALL[0] - OUT_MARGIN <= ox <= WALL[1] + OUT_MARGIN and WALL[2] - OUT_MARGIN <= oy <= WALL[3] + OUT_MARGIN):
                self.log(f'{o["name"]} が場外へ出た ({ox:.3f},{oy:.3f})')
                return self.resolve(o, False, 'out_of_field', (ox, oy))
        if (self.now() - bt['t0'] > self.a.batch_limit) if self.a.batch_limit else (self.now() - self.t_pick > self.a.time_limit):
            if c is not None:
                o = c
            elif self.snap is not None:
                o = self.snap[0]
            elif not field:  # 念のため：どの状態にも属さない物体が残ったら、それを時間切れにして先へ進む（止まり続けない）
                o = next(iter(live), None)
                if o is None:
                    return
            else:
                o = min(field, key=lambda q: math.hypot(self.pose[q['name']][0] - ex, self.pose[q['name']][1] - ey))
            self.log(f'時間切れ -> {o["name"]} を失敗として消す')
            return self.resolve(o, False, 'timeout')

    def step_multi_falling(self):
        c = self.cur
        ox, oy, oz = self.pose[c['name']]
        if oz <= LAND_Z or self.now() - self.t_rel > 1.0:
            zx, zy = ZONES[c['b']]
            other = [o for o in ZONES if o != c['b']][0]
            if abs(ox - zx) <= ZONE_HALF and abs(oy - zy) <= ZONE_HALF:
                ok, reason = True, 'success'
            elif abs(ox - ZONES[other][0]) <= ZONE_HALF and abs(oy - ZONES[other][1]) <= ZONE_HALF:
                ok, reason = False, 'wrong_zone'
            else:
                ok, reason = False, 'dropped'
            self.log(f'landed {c["name"]} {self.now() - self.t_rel:.3f}s after release at ({ox:.3f},{oy:.3f},{oz:.3f}) -> {reason}')
            self.resolve(c, ok, reason, (ox, oy))

    def publish_multi(self):
        st = dict(t=round(self.now(), 4), phase=self.phase, n_done=len(self.results))
        bt = self.batch
        if bt is not None:
            objs = []
            for o in bt['objs']:
                if o['state'] != 'done' and o.get('seen') and o['name'] in self.pose:
                    px, py, pz = self.pose[o['name']]
                    objs.append(dict(name=o['name'], color=o['b'], x=round(px, 4), y=round(py, 4), z=round(pz, 4), state=o['state']))
            st.update(ep=bt['ep'], objs=objs, k_done=len(bt['results']), attached=self.cur is not None and self.cur['state'] == 'held')
            if self.cur is not None and self.cur['name'] in self.pose:  # お手本が陣地で落とす位置を補正するのに使う
                px, py, pz = self.pose[self.cur['name']]
                st.update(held=self.cur['name'], color=self.cur['b'], x=px, y=py, z=pz)
        self.status_pub.publish(String(data=json.dumps(st)))

    def publish(self):
        if self.multi:
            return self.publish_multi()
        if getattr(self, 'last_obj', None) is not None:
            self.obj_pub.publish(Float32MultiArray(data=[float(v) for v in self.last_obj]))
        c = getattr(self, 'cur', None)
        # t（sim 時刻）は変換で bag の受信時刻（壁時計）を sim 時刻へ直すのに使う
        st = dict(t=round(self.now(), 4), phase=self.phase, n_done=len(self.results))
        if c is not None:
            st.update(ep=c['ep'], color=c['b'], x=self.last_obj[0], y=self.last_obj[1],
                      z=self.pose.get(c['b'], (0, 0, 0))[2], vx=c['vx'], vy=c['vy'],
                      moving=c['moving'], attached=c['attached'])
            if c['b'] in self.pose:  # Gazebo 上の実際の位置（確認用。x, y は審判が書き込んだ値）
                st.update(ax=round(self.pose[c['b']][0], 4), ay=round(self.pose[c['b']][1], 4))
        self.status_pub.publish(String(data=json.dumps(st)))

    def run(self):
        a = self.a
        while self.now() < 1.0:  # /clock が届くまで待つ
            time.sleep(0.1)
        t = self.now()
        while self.now() - t < 1.0:
            time.sleep(0.01)
        # 起動直後の DetachableJoint の固定が残っていると、set_pose で腕ごと引っぱられる（v4 で発生）ので外してから始める
        for b in self.names:
            self.det[b].publish(Empty())
        time.sleep(0.5)
        for b in self.names:
            self.set_pose(b, *park_of(b))
        self.phase, self.results, self.cur, self.last_obj = 'wait_home', [], None, None
        self.t_outcome = None
        t_wait = self.now()
        self.log(f'referee start: n={a.n} mode={a.mode} seed={a.seed} home_q={[round(v, 3) for v in HOME_Q]}')
        t_next = self.now()
        while rclpy.ok():
            time.sleep(0.001)
            if self.now() < t_next:
                continue
            t_next += PERIOD
            if self.phase == 'active':
                self.step_multi() if self.multi else self.step_active()
            elif self.phase == 'falling':
                self.step_multi_falling() if self.multi else self.step_falling()
            elif self.phase in ('wait_home', 'returning'):
                home = at_home(self.js)
                waited = self.now() - (self.t_outcome if self.t_outcome is not None else t_wait)
                limit = a.home_timeout if self.t_outcome is not None else a.first_wait
                if (home or waited > limit) and self.multi:
                    if self.batch is not None:
                        self.end_batch(home)
                    elif not home:
                        self.log('初期姿勢に戻らないまま待ち時間を過ぎたので、そのまま出す')
                    if len(self.results) >= a.n:
                        break
                    self.spawn_batch(len(self.results))
                elif home or waited > limit:
                    if self.cur is not None:
                        r = dict(self.result, home=home, t_ep=round(self.now() - self.cur['t0'], 3))
                        self.results.append(r)
                        self.emit(ev='end', **r)
                        with open(a.out, 'a') as f:
                            f.write(json.dumps(dict(r, run=a.tag, mode=a.mode, seed=a.seed)) + '\n')
                    elif not home:
                        self.log('初期姿勢に戻らないまま待ち時間を過ぎたので、そのまま出す')
                    if len(self.results) >= a.n:
                        break
                    self.spawn(len(self.results))
            self.publish()
        self.summary()

    def summary(self):
        if self.multi:
            return self.summary_multi()
        rs = self.results
        ok = [r for r in rs if r['ok']]
        reasons = {}
        for r in rs:
            reasons[r['reason']] = reasons.get(r['reason'], 0) + 1
        med = statistics.median([r['t_obj'] for r in ok]) if ok else float('nan')
        s = dict(run=self.a.tag, mode=self.a.mode, seed=self.a.seed, n=len(rs), success=len(ok),
                 rate=round(len(ok) / max(len(rs), 1), 3), median_t_obj=round(med, 2), reasons=reasons,
                 not_home=sum(1 for r in rs if not r['home']))
        self.log('SUMMARY ' + json.dumps(s, ensure_ascii=False))
        with open(self.a.out.replace('.jsonl', '_summary.json'), 'w') as f:
            json.dump(s, f, ensure_ascii=False, indent=1)


    def summary_multi(self):
        objs = []
        with open(self.a.out, 'a+') as f:  # 1組も終わらずに止めたときは、ファイルがまだない
            f.seek(0)
            for line in f:
                r = json.loads(line)
                if r.get('run') == self.a.tag:
                    objs.append(r)
        ok = [r for r in objs if r['ok']]
        reasons = {}
        for r in objs:
            reasons[r['reason']] = reasons.get(r['reason'], 0) + 1
        bs = self.results
        med = statistics.median([e['t_ep'] for e in bs if e['ok']]) if any(e['ok'] for e in bs) else float('nan')
        s = dict(run=self.a.tag, mode='multi', k=self.a.k, seed=self.a.seed, n=len(bs), success=sum(e['ok'] for e in bs),
                 rate=round(sum(e['ok'] for e in bs) / max(len(bs), 1), 3), median_t_ep=round(med, 2),
                 n_obj=len(objs), obj_success=len(ok), obj_rate=round(len(ok) / max(len(objs), 1), 3),
                 median_t_obj=round(statistics.median([r['t_obj'] for r in ok]), 2) if ok else float('nan'),
                 reasons=reasons, rule_ok=sum(1 for r in objs if r.get('rule_ok')),
                 moved_over_3mm=sum(1 for r in objs if r['moved_mm'] > 3.0), not_home=sum(1 for e in bs if not e['home']))
        self.log('SUMMARY ' + json.dumps(s, ensure_ascii=False))
        with open(self.a.out.replace('.jsonl', '_summary.json'), 'w') as f:
            json.dump(s, f, ensure_ascii=False, indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=20, help='multi では組の数')
    ap.add_argument('--mode', choices=['static', 'reflect', 'multi'], default='static')
    ap.add_argument('--k', type=int, default=3, help='[multi] 1組の物体の数（色ごとに3個まで）')
    ap.add_argument('--min-dist', type=float, default=0.07, help='[multi] 物体の中心どうしの最小間隔（m）。5 cm では開いた手が隣に当たった（2026-10-20 の試運転）')
    ap.add_argument('--min-dy', type=float, default=0.03, help='[multi] 物体どうしの y の最小差（m）。y の小さい側から取る規則が迷わないように')
    ap.add_argument('--rule', choices=['ymin', 'ymax', 'order'], default='ymin', help='[multi] rule_ok を数えるときのお手本の規則')
    ap.add_argument('--layout', default=None, help='[multi] 決めた配置の JSON（当たる範囲の測定用）')
    ap.add_argument('--placement', choices=['iso', 'dir'], default='iso', help='[multi] 置き方の条件。iso＝中心どうし --min-dist 以上、dir＝向きを区別（bs_common.layout_ok_dir）')
    ap.add_argument('--balance', action='store_true', help='[multi] 手前・奥の形を等しい確率で選んでから置く')
    ap.add_argument('--batch-limit', type=float, default=0.0, help='[multi] >0 なら1個ごとの制限（--time-limit）の代わりに、組を出してからこの秒数（sim）で組を打ち切る')
    ap.add_argument('--attach-max', type=float, default=ATTACH_MAX, help='[multi] 固定する（物体を手先の真下へ吸い寄せる）水平距離の上限（m）。提出の版の評価は 0.010')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--speed', type=float, default=0.04)
    ap.add_argument('--time-limit', type=float, default=TIME_LIMIT)
    ap.add_argument('--home-timeout', type=float, default=5.0, help='判定のあと、初期姿勢に戻らなくても次を出すまでの秒（sim）')
    ap.add_argument('--first-wait', type=float, default=60.0, help='最初の1個を出す前に初期姿勢を待つ上限（sim 秒）')
    ap.add_argument('--out', required=True)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    rclpy.init()
    n = Referee(a)
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        n.run()
    finally:
        n.relay.kill()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
