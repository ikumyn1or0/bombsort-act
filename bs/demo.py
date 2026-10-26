# お手本のスクリプト。審判（referee.py）と組んで連続形式で動く。
# 使い方: python3 demo.py --mode static   （反射の段は --mode reflect --delay 4）
#
# 指令の形：補間した関節の指令を 20 Hz（sim 時刻）で出し続け、
# 各指令の time_from_start は 0.1 秒（テレオペ physicalai_teleop と同じ）。腕とグリッパーの両方を毎周期出す。
# 手順は毎回同じ（初期姿勢 → 物体の上 → 真下に下ろす → 閉じる → 持ち上げる → 陣地の上 → 落とす高さへ → 開く → 初期姿勢）。
# 正しい状態（物体の位置・速度・固定の有無）は審判の /bs_status から読む。
import argparse, json, math, random, threading, time
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from bs_common import *  # noqa: F403

PERIOD = 0.05


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


class Demo(Node):
    def __init__(self, a):
        super().__init__('bs_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.a = a
        self.arm_pub = self.create_publisher(JointTrajectory, '/crane_plus_arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/crane_plus_gripper_controller/joint_trajectory', 10)
        self.js = {}
        self.create_subscription(JointState, '/joint_states', lambda m: self.js.update(zip(m.name, m.position)), 10)
        self.st = {}
        self.create_subscription(String, '/bs_status', lambda m: setattr(self, 'st', json.loads(m.data)), 10)
        self.label_pub = self.create_publisher(JointTrajectory, '/bs_label_arm', 10)
        self.lock = threading.Lock()
        self.seg = None   # (q_from, q_to, t0, dur)。実際に出す指令
        self.seg_c = None  # ずらさない正しい指令（/bs_label_arm に出す。ずらしていないときは seg と同じ）
        self.grip = G_OPEN
        self.rng = random.Random(a.noise_seed)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, s):
        print(f'[sim {self.now():8.3f}] {s}', flush=True)

    def wait(self, sec):
        t0 = self.now()
        while self.now() - t0 < sec:
            time.sleep(0.002)

    def wait_until(self, cond, timeout):
        t0 = self.now()
        while self.now() - t0 < timeout:
            if cond():
                return True
            time.sleep(0.005)
        return False

    # ---- 20 Hz の指令 ----
    def cmd_q(self, t, seg=None):
        q0, q1, t0, dur = seg or self.seg
        s = smooth((t - t0) / dur) if dur > 0 else 1.0
        return [a + (b - a) * s for a, b in zip(q0, q1)]

    def publisher(self):
        t_next = self.now()
        while rclpy.ok():
            time.sleep(0.001)
            t = self.now()
            if t < t_next or self.seg is None:
                continue
            t_next += PERIOD
            if t_next < t:  # sim 時刻が飛んだら追いつかせる（指令をまとめて出さない）
                t_next = t + PERIOD
            with self.lock:
                q, g, qc = self.cmd_q(t), self.grip, self.cmd_q(t, self.seg_c)
            for pub, names, pos in [(self.arm_pub, ARM, q), (self.grip_pub, [HAND], [g]), (self.label_pub, ARM, qc)]:
                m = JointTrajectory(); m.joint_names = names
                p = JointTrajectoryPoint(); p.positions = [float(v) for v in pos]
                p.time_from_start.sec = 0; p.time_from_start.nanosec = int(0.1 * 1e9)
                m.points = [p]; pub.publish(m)

    def move(self, x, y, z, pitch, dur, settle=0.0, off=(0.0, 0.0)):
        """off：実際に出す指令だけ水平に off（m）ずらす。/bs_label_arm にはずらさない指令を出す。"""
        q, pd = ik(x, y, z, pitch)
        qx = ik(x + off[0], y + off[1], z, pitch)[0] if any(off) else q
        if q is None or qx is None:
            self.log(f'move ({x:.3f},{y:.3f},{z:.3f}) off=({off[0]:.3f},{off[1]:.3f}) 解なし')
            return False
        with self.lock:
            t = self.now()
            self.seg_c = (self.cmd_q(t, self.seg_c), q, t, dur)
            self.seg = (self.cmd_q(t), qx, t, dur)
        self.wait(dur + settle)
        return True

    def draw_off(self):
        """掴む前のずらし（試した版で使った。提出の版では使わない）。確率 --noise-p で、向きは一様、大きさは --noise-min〜--noise-max（m）の一様。"""
        a = self.a
        if a.noise_p <= 0 or self.rng.random() >= a.noise_p:
            return (0.0, 0.0)
        r, th = self.rng.uniform(a.noise_min, a.noise_max), self.rng.uniform(0, 2 * math.pi)
        return (r * math.cos(th), r * math.sin(th))

    def set_grip(self, g, wait=0.0):
        with self.lock:
            self.grip = g
        self.wait(wait)

    def endtip(self):
        ex, ey, ez, _ = forward_kinematics([self.js[n] for n in ARM])
        return ex, ey, ez

    # ---- 1個ぶん ----
    def predict(self, T):
        s = self.st
        x, y, vx, vy = s['x'], s['y'], s['vx'], s['vy']
        for _ in range(int(T / PERIOD)):
            x += vx * PERIOD; y += vy * PERIOD
            if not FIELD[0] <= x <= FIELD[1]:
                vx = -vx; x = min(max(x, FIELD[0]), FIELD[1])
            if not FIELD[2] <= y <= FIELD[3]:
                vy = -vy; y = min(max(y, FIELD[2]), FIELD[3])
        return x, y

    def go(self, x, y, z, pitch, dur):
        """move と同じだが待たない（指令の区間を差し替えるだけ）。"""
        q, pd = ik(x, y, z, pitch)
        if q is None:
            self.log(f'go ({x:.3f},{y:.3f},{z:.3f}) 解なし')
            return False
        with self.lock:
            t = self.now()
            self.seg = (self.cmd_q(t), q, t, dur)
            self.seg_c = (self.cmd_q(t, self.seg_c), q, t, dur)
        return True

    def one_flow(self):
        """静止の段で、腕の指令が止まる区間を作らない手順（2026-10-16）。
        元の手順は「閉じる前 0.8 秒＋固定待ち」と「陣地の上で 0.5・0.4 秒」腕が止まり、学習した方策がその姿勢で止まり続けた。
        ここでは、下ろし切る直前に閉じ始め、閉じたらすぐゆっくり持ち上げる（固定は手先 6 cm 以下で効くので、持ち上げの途中で効く）。
        陣地では、運んでいる途中に物体と手先の差を1回測り、落とす位置へ直行して、着く直前に開く。"""
        s = dict(self.st)
        ep, b = s['ep'], s['color']
        zx, zy = ZONES[b]
        ox, oy = s['x'], s['y']
        a = self.a
        self.move(ox, oy, ABOVE_Z, -90, 1.5)
        self.go(ox, oy, GRASP_Z + a.down_dz, -90, a.down)  # 関節の遅れで手先が目標より高く残るので、指令は低めに出す
        self.wait(a.down + a.close_lead)          # close_lead < 0 なら下ろし切る前に閉じ始める
        ex, ey, ez = self.endtip()
        self.log(f'ep{ep}: close at endtip=({ex:.3f},{ey:.3f},{ez:.3f}) target=({ox:.3f},{oy:.3f},{GRASP_Z:.3f})')
        self.set_grip(G_CLOSE)
        self.wait(a.lift_delay)
        self.go(ox, oy, ABOVE_Z, -90, a.lift)
        if not self.wait_until(lambda: self.st.get('attached'), a.lift):
            self.log(f'ep{ep}: 固定されなかった -> 開いて初期姿勢へ')
            self.set_grip(G_OPEN)
            self.move(*HOME_XYZ, HOME_PITCH, 1.5)
            return
        t_att = self.now()
        self.wait_until(lambda: self.now() - t_att > 0 and self.seg_done(), a.lift)
        # 陣地の上へ向かう途中（手首の向きが ZONE_PITCH に近づいたところ）で、物体と手先の差を測る
        self.go(zx, zy, ABOVE_Z, ZONE_PITCH, a.carry)
        self.wait(a.carry * 0.8)
        ex, ey, ez = self.endtip()
        o = self.st
        tx, ty, tz = ex + zx - o['x'], ey + zy - o['y'], ez + DROP_Z - o['z']
        self.log(f'ep{ep}: offset tip-obj=({ex - o["x"]:.3f},{ey - o["y"]:.3f},{ez - o["z"]:.3f}) -> drop target tip=({tx:.3f},{ty:.3f},{tz:.3f})')
        self.wait(a.carry * 0.2)
        self.go(tx, ty, tz, ZONE_PITCH, a.drop)
        self.wait(a.drop + a.open_lead)
        o = self.st
        self.log(f'ep{ep}: open at obj=({o["x"]:.3f},{o["y"]:.3f},{o["z"]:.3f}) err={math.hypot(o["x"] - zx, o["y"] - zy)*1000:.1f}mm')
        self.set_grip(G_OPEN)
        self.go(*HOME_XYZ, HOME_PITCH, a.home)
        self.wait(a.home)

    def seg_done(self):
        q0, q1, t0, dur = self.seg
        return self.now() - t0 >= dur

    def one(self):
        if self.a.flow and self.a.mode == 'static':
            return self.one_flow()
        s = dict(self.st)
        ep, b = s['ep'], s['color']
        zx, zy = ZONES[b]
        if self.a.mode == 'static':
            ox, oy = s['x'], s['y']
            self.move(ox, oy, ABOVE_Z, -90, 1.5)
            self.move(ox, oy, GRASP_Z, -90, 1.0, settle=0.8)  # 関節は時定数約 0.4 秒で遅れて追従する（track_test.py で実測）
        else:
            self.wait(self.a.delay)  # 手を出さずに反射を見せる
            for k in range(6):
                px, py = self.predict(2.0)
                self.log(f'ep{ep}: reach try{k} -> predicted ({px:.3f},{py:.3f})')
                self.move(px, py, GRASP_Z, -90, 1.5)
                if self.wait_until(lambda: not self.st.get('moving', True), 1.0):
                    break
            if self.st.get('moving', True):
                self.log(f'ep{ep}: 吸い寄せられなかった')
                self.move(*HOME_XYZ, HOME_PITCH, 1.5)
                return
            ox, oy = self.st['x'], self.st['y']
            if self.a.hold > 0:
                # 吸い寄せは腕が止まり切る前（2 cm/s 未満）に効くので、そのまま閉じると閉じる間に手先が流れて空を掴む
                # （反射の段の試運転で 3 個中 1 個、18 mm 流れた）。物体の位置へ据え直し、静止の段と同じく待ってから閉じる
                self.move(ox, oy, GRASP_Z, -90, self.a.hold, settle=0.8)
        ex, ey, ez = self.endtip()
        self.log(f'ep{ep}: close at endtip=({ex:.3f},{ey:.3f},{ez:.3f}) target=({ox:.3f},{oy:.3f},{GRASP_Z:.3f})')
        self.set_grip(G_CLOSE)
        if not self.wait_until(lambda: self.st.get('attached'), 2.0):
            self.log(f'ep{ep}: 固定されなかった -> 開いて初期姿勢へ')
            self.set_grip(G_OPEN)
            self.move(ox, oy, ABOVE_Z, -90, 1.0)
            self.move(*HOME_XYZ, HOME_PITCH, 1.5)
            return
        self.move(ox, oy, ABOVE_Z, -90, 1.0)
        self.move(zx, zy, ABOVE_Z, ZONE_PITCH, 1.5, settle=0.5)
        # 物体は手首に固定されているので、手先との差を測って落とす位置を補正する（2回まで）
        ex, ey, ez = self.endtip()
        o = self.st
        tx, ty, tz = ex + zx - o['x'], ey + zy - o['y'], ez + DROP_Z - o['z']
        for it in range(2):
            self.move(tx, ty, tz, ZONE_PITCH, 1.0 if it == 0 else 0.5, settle=0.4)
            o = self.st
            err = math.hypot(o['x'] - zx, o['y'] - zy)
            self.log(f'ep{ep}: place iter{it} obj=({o["x"]:.3f},{o["y"]:.3f},{o["z"]:.3f}) err={err*1000:.1f}mm')
            if err < 0.01 and abs(o['z'] - DROP_Z) < 0.005:
                break
            tx += zx - o['x']; ty += zy - o['y']; tz += DROP_Z - o['z']
        self.set_grip(G_OPEN)
        self.wait_until(lambda: self.st.get('phase') not in ('active', 'falling'), 1.5)
        self.move(*HOME_XYZ, HOME_PITCH, 1.5)

    def place(self, ep, zx, zy, z_above):
        """陣地の上へ運んで落とす（one() と同じ。物体は手首に固定されているので、手先との差を測って落とす位置を補正する）。"""
        self.move(zx, zy, z_above, ZONE_PITCH, 1.5, settle=0.5)
        ex, ey, ez = self.endtip()
        o = self.st
        if 'x' not in o:  # 運んでいる間に時間切れで消された
            self.log(f'ep{ep}: 運んでいる物体がない -> 開く')
            self.set_grip(G_OPEN)
            return
        tx, ty, tz = ex + zx - o['x'], ey + zy - o['y'], ez + DROP_Z - o['z']
        for it in range(2):
            self.move(tx, ty, tz, ZONE_PITCH, 1.0 if it == 0 else 0.5, settle=0.4)
            o = self.st
            if 'x' not in o:
                break
            err = math.hypot(o['x'] - zx, o['y'] - zy)
            self.log(f'ep{ep}: place iter{it} obj=({o["x"]:.3f},{o["y"]:.3f},{o["z"]:.3f}) err={err*1000:.1f}mm')
            if err < 0.01 and abs(o['z'] - DROP_Z) < 0.005:
                break
            tx += zx - o['x']; ty += zy - o['y']; tz += DROP_Z - o['z']
        self.set_grip(G_OPEN)

    def one_multi(self):
        """静止した複数の物体の1組。場の物体を規則（いったん y の小さい側から）で1個ずつ選び、
        置いたら初期姿勢へ戻らずに次の物体へ向かう。組を運び切ったら初期姿勢へ戻る。
        手順は静止の段の one() と同じで、運ぶ高さだけ --carry-z に上げる
        （手先 0.08 m で運ぶと、掴んだ物体の底と置いてある物体の上面がどちらも約 0.042 m で、隙間がない）。"""
        ep, cz = self.st['ep'], self.a.carry_z
        inside = lambda o: WALL[0] <= o['x'] <= WALL[1] and WALL[2] <= o['y'] <= WALL[3]
        # 審判は、出した物体の位置が中継で確かめられてから公開する。組の全部がそろうまで少し待つ
        self.wait_until(lambda: sum(inside(o) for o in self.st.get('objs', [])) >= 3, 1.0)
        for attempt in range(3 * 3):  # 掴み損ねたら同じ規則で選び直す（審判の時間切れで消えることもある）
            s = dict(self.st)
            field = [o for o in s.get('objs', []) if o['state'] == 'field' and inside(o)]
            if s.get('ep') != ep or s.get('phase') != 'active' or not field:
                break
            o = field[0] if self.a.rule == 'order' else (max if self.a.rule == 'ymax' else min)(field, key=lambda q: q['y'])  # 規則：y の小さい側から（ymin）・大きい側から（ymax）・出した順（order）
            zx, zy = ZONES[o['color']]
            ox, oy = o['x'], o['y']
            off = self.draw_off()
            self.log(f'ep{ep}: pick {o["name"]} at ({ox:.3f},{oy:.3f}) of {len(field)} noise=({off[0]*1000:.1f},{off[1]*1000:.1f})mm')
            self.move(ox, oy, cz, -90, 1.5, off=off)
            # carry_z では場の手前側で手先を起こさないと解がない（-80〜-60°）。起こしたまま下ろすと指が物体に当たったので
            # （試運転 m4d_try1：手先 0.073 m で止まり、物体が 7 mm 動いた）、静止の段と同じ高さで真下向きに直してから下ろす
            self.move(ox, oy, ABOVE_Z, -90, 0.5, off=off)
            if any(off):
                # ずらしたまま掴む高さまで下ろし、そこで物体の真上へ直す（方策が外すのは掴む高さで数 mm ずれた所なので、その画像を作る）
                self.move(ox, oy, GRASP_Z, -90, 1.0, off=off)
                self.move(ox, oy, GRASP_Z, -90, self.a.noise_fix, settle=0.8)
            else:
                self.move(ox, oy, GRASP_Z, -90, 1.0, settle=0.8)
            ex, ey, ez = self.endtip()
            self.log(f'ep{ep}: close at endtip=({ex:.3f},{ey:.3f},{ez:.3f}) target=({ox:.3f},{oy:.3f},{GRASP_Z:.3f})')
            self.set_grip(G_CLOSE)
            if not self.wait_until(lambda: self.st.get('attached'), 2.0):
                self.log(f'ep{ep}: 固定されなかった -> 開いて選び直す')
                self.set_grip(G_OPEN)
                self.move(ox, oy, cz, -90, 1.0)
                continue
            self.move(ox, oy, cz, -90, 1.0)
            self.place(ep, zx, zy, cz)
            kd = s.get('k_done', 0)
            self.wait_until(lambda: self.st.get('phase') != 'falling' and self.st.get('k_done', 0) > kd, 1.5)
            # 落とす高さ（手先 0.06 m 前後）から次の物体の上へ直行すると、開いた手が場の縁を低く横切って物体に当たった
            # （試運転 m4d_try4_d7：手先 0.073 m で、次に取る物体を 44 mm 押した）。陣地の上で carry_z まで上げてから向かう
            self.move(zx, zy, cz, ZONE_PITCH, 0.5)
        self.move(*HOME_XYZ, HOME_PITCH, 1.5)

    def run(self):
        while self.now() < 1.0 or any(n not in self.js for n in ARM):
            time.sleep(0.1)
        with self.lock:
            q = [self.js[n] for n in ARM]
            self.seg = self.seg_c = (q, q, self.now(), 0.0)
        threading.Thread(target=self.publisher, daemon=True).start()
        self.set_grip(G_OPEN)
        self.move(*HOME_XYZ, HOME_PITCH, 2.0)
        self.log('demo ready (at home)')
        done = -1
        while rclpy.ok():
            s = self.st
            if s.get('phase') == 'active' and s.get('ep', -1) > done:
                done = s['ep']
                t0 = self.now()
                self.one_multi() if self.a.mode == 'multi' else self.one()
                self.log(f'ep{done}: demo finished in {self.now() - t0:.1f}s')
            time.sleep(0.005)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['static', 'reflect', 'multi'], default='static')
    ap.add_argument('--carry-z', type=float, default=0.10, help='[multi] 物体の上を通る・運ぶ高さ（手先、m）')
    ap.add_argument('--rule', choices=['ymin', 'ymax', 'order'], default='ymin', help='[multi] どの物体から取るか（y の小さい側から／大きい側から／審判が出した順＝決めた配置の順）')
    ap.add_argument('--delay', type=float, default=4.0, help='反射の段で、手を出すまで待つ秒（sim）')
    ap.add_argument('--hold', type=float, default=0.5, help='反射の段で、止まったあと物体の位置へ据え直す秒（0 で据え直さずすぐ閉じる＝初めの版の動き）')
    ap.add_argument('--flow',action='store_true', help='静止の段で、腕が止まる区間を作らない手順（one_flow）')
    ap.add_argument('--down', type=float, default=1.5, help='[flow] 物体の上から掴む高さまで下ろす秒')
    ap.add_argument('--down-dz', type=float, default=-0.01, help='[flow] 下ろす指令の高さを GRASP_Z からいくつずらすか（m）')
    ap.add_argument('--close-lead', type=float, default=0.0, help='[flow] 下ろし切る何秒前に閉じ始めるか（負で前）')
    ap.add_argument('--lift-delay', type=float, default=0.0, help='[flow] 閉じてから持ち上げ始めるまでの秒')
    ap.add_argument('--lift', type=float, default=2.5, help='[flow] 持ち上げる秒（ゆっくり始まる補間）')
    ap.add_argument('--carry', type=float, default=2.0, help='[flow] 陣地の上へ運ぶ秒')
    ap.add_argument('--drop', type=float, default=1.0, help='[flow] 落とす位置へ下ろす秒')
    ap.add_argument('--open-lead', type=float, default=0.0, help='[flow] 落とす位置に着く何秒後に開くか（負で前）')
    ap.add_argument('--home', type=float, default=2.0, help='[flow] 初期姿勢へ戻る秒')
    ap.add_argument('--noise-p', type=float, default=0.0, help='[multi] 1個ごとに掴む前の動きをずらす確率（0 でずらさない）')
    ap.add_argument('--noise-min', type=float, default=0.004, help='[multi] ずらす量の下限（m）')
    ap.add_argument('--noise-max', type=float, default=0.012, help='[multi] ずらす量の上限（m）')
    ap.add_argument('--noise-fix', type=float, default=0.5, help='[multi] 掴む高さで物体の真上へ直す秒')
    ap.add_argument('--noise-seed', type=int, default=0, help='[multi] ずらしの乱数の種')
    a = ap.parse_args()
    rclpy.init()
    n = Demo(a)
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        n.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
