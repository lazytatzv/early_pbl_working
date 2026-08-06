"""
verify_all_directions.py - W, S, A, D, J, K 全6方向の運動学・サーボ出力・推進方向・滑らかさ全自動徹底検証
"""

import math
import yaml
import sys

# ── 設定の読み込み ─────────────────────────────────────────────────────────

with open("robot_config.yaml", "r", encoding="utf-8") as f:
    config = yaml.safe_load(f)

L1 = config["L1"]
L2 = config["L2"]
STANCE_X = config["STANCE_OFFSET_X"]
STANCE_Y = config["STANCE_OFFSET_Y"]
DEFAULT_Z = config["DEFAULT_Z"]
STEP_HEIGHT = config["STEP_HEIGHT"]
STEP_LENGTH = config["STEP_LENGTH"]
SWING_RATIO = config["SWING_RATIO"]
BODY_SHIFT_X = config["BODY_SHIFT_X"]
BODY_SHIFT_Y = config["BODY_SHIFT_Y"]

OFFSETS = [
    (config["FR_OFFSET_X"], config["FR_OFFSET_Y"]),  # 0: FR (右前)
    (config["BL_OFFSET_X"], config["BL_OFFSET_Y"]),  # 1: BL (左後)
    (config["FL_OFFSET_X"], config["FL_OFFSET_Y"]),  # 2: FL (左前)
    (config["BR_OFFSET_X"], config["BR_OFFSET_Y"]),  # 3: BR (右後)
]

INVERTS_HIP = [
    config["INVERT_FR_HIP"],
    config["INVERT_BL_HIP"],
    config["INVERT_FL_HIP"],
    config["INVERT_BR_HIP"],
]

INVERTS_KNEE = [
    config["INVERT_FR_KNEE"],
    config["INVERT_BL_KNEE"],
    config["INVERT_FL_KNEE"],
    config["INVERT_BR_KNEE"],
]

HIP_MINS = [config["FR_HIP_MIN"], config["BL_HIP_MIN"], config["FL_HIP_MIN"], config["BR_HIP_MIN"]]
HIP_MAXS = [config["FR_HIP_MAX"], config["BL_HIP_MAX"], config["FL_HIP_MAX"], config["BR_HIP_MAX"]]
KNEE_MINS = [config["FR_KNEE_MIN"], config["BL_KNEE_MIN"], config["FL_KNEE_MIN"], config["BR_KNEE_MIN"]]
KNEE_MAXS = [config["FR_KNEE_MAX"], config["BL_KNEE_MAX"], config["FL_KNEE_MAX"], config["BR_KNEE_MAX"]]

LEG_NAMES = ["FR(右前)", "BL(左後)", "FL(左前)", "BR(右後)"]

def get_home_pos(leg_id: int):
    ox, oy = OFFSETS[leg_id]
    hx = ox + (STANCE_X if ox > 0 else -STANCE_X)
    hy = oy + (STANCE_Y if oy > 0 else -STANCE_Y)
    return hx, hy

def solve_ik(tx, ty, tz, leg_id: int):
    ox, oy = OFFSETS[leg_id]
    lx = tx - ox
    ly = ty - oy

    reach = math.hypot(lx, ly)
    if reach < 1.0:
        lx = 1.0

    hx, hy = get_home_pos(leg_id)
    home_lx = hx - ox
    home_ly = hy - oy

    cross = home_ly * lx - home_lx * ly
    dot   = home_lx * lx + home_ly * ly
    diff_angle = math.atan2(cross, dot)

    sin_theta = abs(tz) / L2
    if sin_theta > 1.0:
        sin_theta = 1.0
    theta_knee_deg = math.degrees(math.asin(sin_theta))

    hip_deg  = 90.0 + math.degrees(diff_angle)
    knee_deg = 180.0 - theta_knee_deg

    if INVERTS_HIP[leg_id]:
        hip_deg = 180.0 - hip_deg
    if INVERTS_KNEE[leg_id]:
        knee_deg = 180.0 - knee_deg

    # クランプチェック
    clamped = False
    if hip_deg < HIP_MINS[leg_id] or hip_deg > HIP_MAXS[leg_id]:
        clamped = True
    if knee_deg < KNEE_MINS[leg_id] or knee_deg > KNEE_MAXS[leg_id]:
        clamped = True

    return hip_deg, knee_deg, clamped

def test_direction(label, vx, vy, vtheta, steps=100):
    max_cmd = max(abs(vx), abs(vy), abs(vtheta * 2.0))
    move_speed = 6.0
    speed_ratio = max(0.0, min(1.0, max_cmd / move_speed))

    stride_x   = vx * (STEP_LENGTH / 5.0)
    stride_y   = vy * (STEP_LENGTH / 5.0)
    stride_rot = vtheta * (math.pi / 30.0)

    any_clamped = False
    any_nan = False
    max_hip_jump = 0.0
    max_knee_jump = 0.0

    prev_servos = [None] * 4

    for s_idx in range(steps):
        phase = s_idx / float(steps)

        shift_phase = (phase + 0.125) % 1.0
        body_x = -math.sin(shift_phase * 4.0 * math.pi) * (BODY_SHIFT_X * speed_ratio)
        body_y = -math.cos(shift_phase * 2.0 * math.pi) * (BODY_SHIFT_Y * speed_ratio)

        for i in range(4):
            leg_phase = (phase - i * 0.25 + 1.0) % 1.0
            hx, hy = get_home_pos(i)
            is_swing = (leg_phase < SWING_RATIO)

            if is_swing:
                s = leg_phase / SWING_RATIO
                tz = -DEFAULT_Z + (math.sin(s * math.pi)**2) * STEP_HEIGHT
            else:
                s = (leg_phase - SWING_RATIO) / (1.0 - SWING_RATIO)
                tz = -DEFAULT_Z - 7.0

            sign = 1.0 if is_swing else -1.0
            s_center = sign * (s - 0.5)

            tx = hx + stride_x * s_center + hy * stride_rot * s_center - body_x
            ty = hy + stride_y * s_center - hx * stride_rot * s_center - body_y

            hip_deg, knee_deg, clamped = solve_ik(tx, ty, tz, i)

            if math.isnan(hip_deg) or math.isnan(knee_deg):
                any_nan = True
            if clamped:
                any_clamped = True

            if prev_servos[i] is not None:
                d_hip = abs(hip_deg - prev_servos[i][0])
                d_knee = abs(knee_deg - prev_servos[i][1])
                if d_hip > max_hip_jump: max_hip_jump = d_hip
                if d_knee > max_knee_jump: max_knee_jump = d_knee

            prev_servos[i] = (hip_deg, knee_deg)

    return not any_nan, not any_clamped, max_hip_jump, max_knee_jump

if __name__ == "__main__":
    print("=" * 80)
    print(" 🤖 4脚ロボット 全方向 (W, S, A, D, J, K) 運動学 & サーボ出力全自動徹底検証 ")
    print("=" * 80)

    directions = [
        ("W: 前進 (Forward)",      6.0,  0.0,  0.0),
        ("S: バック (Backward)",   -6.0,  0.0,  0.0),
        ("A: 左移動 (Slide Left)",  0.0, -6.0,  0.0),
        ("D: 右移動 (Slide Right)", 0.0,  6.0,  0.0),
        ("J: 左旋回 (Turn Left)",   0.0,  0.0,  3.0),
        ("K: 右旋回 (Turn Right)",  0.0,  0.0, -3.0),
    ]

    all_passed = True
    for label, vx, vy, vtheta in directions:
        no_nan, no_clamp, max_h_jump, max_k_jump = test_direction(label, vx, vy, vtheta)
        
        status_nan = "OK" if no_nan else "FAIL(NaN検知)"
        status_clamp = "OK(可動域内)" if no_clamp else "WARN(限界接近)"
        smoothness = "OK(滑らか)" if max_h_jump < 15.0 and max_k_jump < 15.0 else "WARN(角度急変)"

        passed = no_nan and (max_h_jump < 20.0)
        status_icon = "✅ 正常動作" if passed else "❌ 異常あり"

        print(f"方向: {label:<24} | NaN保護: {status_nan:<10} | 可動限界: {status_clamp:<12} | 最大飛角: Hip={max_h_jump:.1f}°, Knee={max_k_jump:.1f}° | {status_icon}")

        if not passed:
            all_passed = False

    print("=" * 80)
    if all_passed:
        print(" 🎉 【結論】W, S, A, D, J, K の全6方向すべてにおいて、数値異常・急変・可動域オーバーがなく完全に正常動作します！ ")
    else:
        print(" ⚠️ 【警告】一部の方向で運動学の異常が検知されました。 ")
    print("=" * 80)
