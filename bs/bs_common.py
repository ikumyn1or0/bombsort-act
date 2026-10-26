# 審判・お手本の共通定数と小道具。値は開発の初めに試作で測って決めた。
# 逆運動学は書籍のサンプル AI-Robot-Book-Humble/chapter6（Apache License 2.0）の kinematics.py を import する（複製しない）。
import math, sys
sys.path.insert(0, '/home/ubuntu/airobot_ws/src/chapter6/crane_plus_commander/crane_plus_commander')
from kinematics import inverse_kinematics, forward_kinematics, joint_in_range  # noqa: E402

BASE_Z = 1.015                       # 台座の世界座標の高さ（set_pose と中継の座標は世界座標）
FIELD = (0.10, 0.16, -0.08, 0.08)    # 円柱の中心が出る・動く範囲（真上から掴める範囲）
WALL = (0.0875, 0.1725, -0.0925, 0.0925)  # 壁の内側（見た目だけ）。場外の判定に使う
OUT_MARGIN = 0.01                    # 中心が壁の内側からこれ以上出たら場外
ZONES = {'bomb_red': (0.05, 0.18), 'bomb_black': (0.05, -0.18)}
ZONE_HALF = 0.045                    # 陣地 9 cm 四方
PARK = {'bomb_red': (3.0, -3.0, 0.02), 'bomb_black': (3.0, 3.0, 0.02)}
OBJ_Z = BASE_Z + 0.022               # 出すときの円柱の中心の高さ（世界座標）

ATTACH_MAX = 0.015   # 閉じた時点で手先と物体の中心の水平距離がこれ以内なら固定する
ATTACH_EZ = 0.06     # 固定するのは手先がこの高さ以下のときだけ
STOP_DIST = 0.02     # 吸い寄せ：開いた手先からこれ以内（水平）
STOP_Z = 0.05        #          かつ手先がこの高さ以下
STILL_V = 0.02       #          かつ手先の速さがこれ未満
TIME_LIMIT = 30.0    # 1個あたりの制限（sim 時刻、物体が出てから着地まで）
DROP_Z = 0.045       # 離すときの円柱の中心の高さ（底が机から 2.5 cm 上）
LAND_Z = 0.023       # 中心がこれ以下になったら着地（静止時は約 0.020）

G_OPEN, G_CLOSE = -0.5236, 0.1745    # テレオペと同じ値
G_MID = (G_OPEN + G_CLOSE) / 2       # 指令がこれより大きければ「閉じる」と読む（方策の出力は連続値のため）
ARM = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
HAND = 'crane_plus_joint_hand'

HOME_XYZ = (0.13, 0.0, 0.10)         # 初期姿勢：フィールドの中央の上、手先 10 cm、真下向き、手は開く
HOME_PITCH = -90
HOME_TOL = 0.03                      # 腕の4関節がすべて初期姿勢からこれ以内（rad）なら「戻り着いた」
GRASP_Z = 0.04                       # 掴む高さ（手先）
ABOVE_Z = 0.08                       # 物体・陣地の上で通る高さ（手先）
ZONE_PITCH = -60                     # 陣地の上での手先の向き


def ik(x, y, z, pitch_deg):
    """真下向きから順に 5° ずつ起こして、可動範囲に収まる最初の解を返す（試作のときと同じ探し方）。"""
    for pd in [pitch_deg] + [pitch_deg + k * 5 for k in range(1, 12)]:
        q = inverse_kinematics([x, y, z, math.radians(pd)], True)
        if q is not None and all(joint_in_range(q)):
            return list(q), pd
    return None, None


HOME_Q, _ = ik(*HOME_XYZ, HOME_PITCH)


def at_home(js):
    if any(n not in js for n in ARM):
        return False
    return max(abs(js[n] - h) for n, h in zip(ARM, HOME_Q)) <= HOME_TOL and js.get(HAND, 0.0) <= G_OPEN + 0.15


# ---- 静止した複数の物体（ワールド bombsort_v6m。モデル名は bomb_red_0..2・bomb_black_0..2） ----
MULTI_NAMES = [f'bomb_{c}_{i}' for c in ('red', 'black') for i in range(3)]


def color_of(name):
    """モデル名から陣地のキー（ZONES のキー）を返す。bomb_red_1 → bomb_red。v5 の bomb_red はそのまま。"""
    return 'bomb_red' if name.startswith('bomb_red') else 'bomb_black'


def park_of(name):
    """場外の待機位置。v6m の i 番目は x を 0.1 ずつずらす（make_v6m.py と同じ）。"""
    if name in PARK:
        return PARK[name]
    x, y, z = PARK[color_of(name)]
    return (x + 0.1 * int(name.rsplit('_', 1)[1]), y, z)


# ---- 置き方の条件（2026-10-20 に、開いた手が当たる範囲を測って決めた） ----
def local_ab(p, q):
    """p から見た q の位置を、根元から p への向きを基準に (接線 a, 半径 b) で返す（m）。a>0 は反時計回り（y が増える側）、b>0 は根元から遠い側。"""
    th = math.atan2(p[1], p[0])
    dx, dy = q[0] - p[0], q[1] - p[1]
    return -math.sin(th) * dx + math.cos(th) * dy, math.cos(th) * dx + math.sin(th) * dy


def layout_ok_dir(pts, min_dy=0.03):
    """y の小さい順に取るとき、後で取る物体が、先に取る物体から見て「a ≥ 6 cm」か「a ≥ 3 cm かつ b ≤ −2.5 cm」にあるか。
    測定では、同じ奥行き（b=0）は a=5 cm まで、根元から遠い側（b=+2.5）は a=4 cm まで当たり、根元寄り（b=−2.5）は a=3 cm から当たらないか軽く触れる程度だった。"""
    s = sorted(pts, key=lambda p: p[1])
    for i in range(len(s)):
        for j in range(i + 1, len(s)):
            a, b = local_ab(s[i], s[j])
            if s[j][1] - s[i][1] < min_dy or not (a >= 0.06 or (a >= 0.03 and b <= -0.025)):
                return False
    return True


def depth_pattern(pts, x_mid=0.13):
    """y の小さい順に、手前（カメラ寄り、x > x_mid）を N、奥を F で並べた形。"""
    return ''.join('N' if p[0] > x_mid else 'F' for p in sorted(pts, key=lambda p: p[1]))


def sample_layout(rng, k=3, placement='dir', balance=True, min_dist=0.07, min_dy=0.03, x_mid=0.13):
    """置き方の条件を満たす k 個の位置を出す。balance なら、先に手前・奥の形（2**k 通り）を等しい確率で選び、その形で置く。"""
    want = ''.join(rng.choice('NF') for _ in range(k)) if balance else None
    for _ in range(200000):
        pts = []
        for i in range(k):
            pts.append((rng.uniform(FIELD[0], FIELD[1]), rng.uniform(FIELD[2], FIELD[3])))
        if want is not None:
            # 形を先に決めるときは、y の順に並べ替えてから奥行きを形に合わせて引き直す
            pts.sort(key=lambda p: p[1])
            pts = [(rng.uniform(x_mid, FIELD[1]) if c == 'N' else rng.uniform(FIELD[0], x_mid), y) for c, (x, y) in zip(want, pts)]
        if placement == 'dir':
            if layout_ok_dir(pts, min_dy):
                return pts
        elif all(math.dist(p, q) >= min_dist and abs(p[1] - q[1]) >= min_dy for i, p in enumerate(pts) for q in pts[i + 1:]):
            return pts
    raise RuntimeError('置き方の条件を満たす配置が見つからない')
