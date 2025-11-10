import math
from utils import wrap_deg180

async def drive_go_to_goal(
    x, y, angle,
    xg=350,
    yg=200,
    vmax=70,  # 直進の最大係数(0..100)
    wmax=60,  # 旋回の最大係数(0..100)
    k_r=0.5,  # 距離ゲイン → v
    k_a=1.2,  # 角度ゲイン → ω
    stop_dist=20):
    if x is None or y is None or angle is None:
        return 0, 0  # まだ位置が来てない

    dx, dy = (xg - x), (yg - y)
    dist = math.hypot(dx, dy)
    if dist < stop_dist:
        return 0, 0

    # 目標の絶対方位[deg]
    target_heading = math.degrees(math.atan2(dy, dx))
    # 現在方位[deg]は toio からそのまま（0..359）
    # 相対方位誤差[-180..180]
    a_err = -wrap_deg180(target_heading - angle)

    # 直進と旋回の係数（比例）。距離が大きいほど速く、方位誤差が大きいほど回頭
    v = k_r * dist  # 距離→直進
    w = k_a * a_err  # 角誤差→旋回（符号つき）

    # スケーリング & クリップ（toio係数）
    v = max(-vmax, min(vmax, v))
    w = max(-wmax, min(wmax, w))

    print (f"drive_go_to_goal: dist={dist:.2f}, a_err={a_err:.2f}, v={v:.2f}, w={w:.2f}")

    # 差動合成：左= v - w/2, 右= v + w/2（係数バランスは機体で微調整）
    left = int(max(-100, min(100, v - 0.5 * w)))
    right = int(max(-100, min(100, v + 0.5 * w)))

    return left, right
