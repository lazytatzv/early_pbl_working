"""
verify_static_stability.py - クロール歩行における静的安定マージンの最適化探査
"""

import math
import sys
import numpy as np

L1 = 68.56
L2 = 126.0
STANCE_OFFSET_X = 48.5
STANCE_OFFSET_Y = 48.5
DEFAULT_Z = 126.0
STEP_HEIGHT = 50.4
STEP_LENGTH = 51.4
SWING_RATIO = 0.20
HEADING_ANGLE = 0.0

OFFSETS = [
    (49.75, 49.75),    # 0: FR (右前)
    (-49.75, -49.75),  # 1: BL (左後)
    (49.75, -49.75),   # 2: FL (左前)
    (-49.75, 49.75),   # 3: BR (右後)
]

def get_home_pos(leg_id: int):
    ox, oy = OFFSETS[leg_id]
    hx = ox + (STANCE_OFFSET_X if ox > 0 else -STANCE_OFFSET_X)
    hy = oy + (STANCE_OFFSET_Y if oy > 0 else -STANCE_OFFSET_Y)
    return hx, hy

def distance_point_to_line_segment(px, py, ax, ay, bx, by):
    l2 = (bx - ax)**2 + (by - ay)**2
    if l2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / l2))
    proj_x = ax + t * (bx - ax)
    proj_y = ay + t * (by - ay)
    return math.hypot(px - proj_x, py - proj_y)

def point_in_triangle(px, py, tri_pts):
    (x1, y1), (x2, y2), (x3, y3) = tri_pts
    d1 = (px - x2) * (y1 - y2) - (x1 - x2) * (py - y2)
    d2 = (px - x3) * (y2 - y3) - (x2 - x3) * (py - y3)
    d3 = (px - x1) * (y3 - y1) - (x3 - x1) * (py - y1)

    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

    is_inside = not (has_neg and has_pos)

    d_e1 = distance_point_to_line_segment(px, py, x1, y1, x2, y2)
    d_e2 = distance_point_to_line_segment(px, py, x2, y2, x3, y3)
    d_e3 = distance_point_to_line_segment(px, py, x3, y3, x1, y1)
    min_dist = min(d_e1, d_e2, d_e3)

    margin = min_dist if is_inside else -min_dist
    return is_inside, margin

def analyze_gait_stability(lead=0.125, shift_x=24.0, shift_y=24.0, stance_offset=48.5, vx=6.0, vy=0.0, vtheta=0.0):
    max_cmd = max(abs(vx), abs(vy), abs(vtheta))
    move_speed = 6.0
    speed_ratio = max(0.0, min(1.0, max_cmd / move_speed))

    heading_rad = math.radians(HEADING_ANGLE)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    rot_vx = vx * cos_h - vy * sin_h
    rot_vy = vx * sin_h + vy * cos_h

    stride_x = rot_vx * (STEP_LENGTH / 5.0)
    stride_y = rot_vy * (STEP_LENGTH / 5.0)
    stride_rot = vtheta * (math.pi / 30.0)

    min_margin = float('inf')

    for step in range(500):
        phase = step / 500.0

        shift_phase = (phase + lead) % 1.0
        body_x = -math.sin(shift_phase * 4.0 * math.pi) * (shift_x * speed_ratio)
        body_y = -math.cos(shift_phase * 2.0 * math.pi) * (shift_y * speed_ratio)

        feet_pos = []
        swings = []

        for i in range(4):
            leg_phase = (phase - i * 0.25 + 1.0) % 1.0
            ox, oy = OFFSETS[i]
            hx = ox + (stance_offset if ox > 0 else -stance_offset)
            hy = oy + (stance_offset if oy > 0 else -stance_offset)

            is_swing = (leg_phase < SWING_RATIO)
            swings.append(is_swing)

            if is_swing:
                s = leg_phase / SWING_RATIO
            else:
                s = (leg_phase - SWING_RATIO) / (1.0 - SWING_RATIO)

            sign = 1.0 if is_swing else -1.0
            s_center = sign * (s - 0.5)

            foot_x = hx + stride_x * s_center - hy * stride_rot * s_center
            foot_y = hy + stride_y * s_center + hx * stride_rot * s_center
            feet_pos.append((foot_x, foot_y))

        stance_feet = [feet_pos[i] for i in range(4) if not swings[i]]

        if len(stance_feet) == 3:
            is_inside, margin = point_in_triangle(body_x, body_y, stance_feet)
            if margin < min_margin:
                min_margin = margin

    return min_margin


if __name__ == "__main__":
    print("=" * 70)
    print(" 📐 スタンス幅・重心シフト量の極限最適化 ")
    print("=" * 70)

    best_m = -999.0
    best_cfg = None

    for stance in [48.5, 55.0, 60.0]:
        for lead in [0.0, 0.0625, 0.125, 0.1875]:
            for sx in [15.0, 20.0, 25.0, 30.0]:
                for sy in [15.0, 20.0, 25.0, 30.0]:
                    m1 = analyze_gait_stability(lead, sx, sy, stance, 6.0, 0.0, 0.0)
                    m2 = analyze_gait_stability(lead, sx, sy, stance, -6.0, 0.0, 0.0)
                    m3 = analyze_gait_stability(lead, sx, sy, stance, 0.0, 0.0, 1.0)
                    worst = min(m1, m2, m3)
                    if worst > best_m:
                        best_m = worst
                        best_cfg = (stance, lead, sx, sy, m1, m2, m3)

    stance, lead, sx, sy, m1, m2, m3 = best_cfg
    print(f"最優スタンス幅 STANCE_OFFSET: {stance:.1f} mm")
    print(f"最優リード位相 lead:          {lead:.4f} サイクル ({lead*360:.1f}度)")
    print(f"最優 BODY_SHIFT_X:            {sx:.1f} mm")
    print(f"最優 BODY_SHIFT_Y:            {sy:.1f} mm")
    print(f"前進マージン:                 {m1:+6.2f} mm")
    print(f"後退マージン:                 {m2:+6.2f} mm")
    print(f"旋回マージン:                 {m3:+6.2f} mm")
    print(f"最悪最小マージン:             {best_m:+6.2f} mm")
    print("=" * 70)
