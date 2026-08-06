"""
parse_urdf.py
URDF から寸法・オフセットを抽出して robot_config.yaml と C++ ヘッダーを更新するスクリプト。

設計方針:
  - 物理寸法 (L1, L2, オフセット) は URDF から自動計算
  - 派生パラメータ (DEFAULT_Z, STANCE_OFFSET, STEP_HEIGHT 等) も新寸法から再計算
  - チューニングパラメータ (CYCLE_TIME, PID ゲイン等) は既存値を維持
"""
from __future__ import annotations
import xml.etree.ElementTree as ET
import math
import os
import yaml
import struct
import glob


def stl_max_dimension(path: str) -> float:
    """
    STL ファイルを読んでバウンディングボックスの最長辺を mm で返す。
    """
    with open(path, 'rb') as f:
        f.read(80)  # header
        n = struct.unpack('<I', f.read(4))[0]
        xs, ys, zs = [], [], []
        for _ in range(n):
            f.read(12)  # normal
            for _ in range(3):
                x, y, z = struct.unpack('<fff', f.read(12))
                xs.append(x); ys.append(y); zs.append(z)
            f.read(2)  # attr
    dims = [
        (max(xs) - min(xs)) * 1000,
        (max(ys) - min(ys)) * 1000,
        (max(zs) - min(zs)) * 1000,
    ]
    return max(dims)


# ============================================================
# URDF パース
# ============================================================

def parse_origin(origin_node) -> tuple[list[float], list[float]]:
    xyz = [float(x) for x in origin_node.get("xyz", "0 0 0").split()]
    rpy = [float(x) for x in origin_node.get("rpy", "0 0 0").split()]
    return xyz, rpy


def get_joints(urdf_path: str) -> dict:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints: dict = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        if origin is not None:
            xyz, rpy = parse_origin(origin)
            joints[name] = {"xyz": xyz, "rpy": rpy}
    return joints


def dist3d(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def find_base_joint(joints: dict, prefix: str, keyword: str) -> list[float] | None:
    """
    prefix で始まり keyword を含み _fixed で終わるジョイントのうち、
    「名前が最も短いもの」(= 数値サフィックスなしの基本ジョイント) を返す。
    prefix + "__" で始まるもの（他脚）は除外する。
    """
    exclude = prefix + "__"
    candidates = [
        (n, j["xyz"])
        for n, j in joints.items()
        if n.startswith(prefix + "_")
        and not n.startswith(exclude)
        and keyword in n
        and n.endswith("_fixed")
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda x: (len(x[0]), x[0]))
    return candidates[0][1]


def find_distal_joint(joints: dict, prefix: str, keyword: str,
                      from_xyz: list[float]) -> list[float] | None:
    """
    prefix で始まり keyword を含み _fixed で終わるジョイントのうち、
    from_xyz から最も遠いものを返す（= 最末端ジョイント）。
    prefix + "__" で始まるもの（他脚）は除外する。
    """
    exclude = prefix + "__"
    candidates = [
        (n, j["xyz"])
        for n, j in joints.items()
        if n.startswith(prefix + "_")
        and not n.startswith(exclude)
        and keyword in n
        and n.endswith("_fixed")
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda x: dist3d(x[1], from_xyz), reverse=True)
    return candidates[0][1]


# ============================================================
# 寸法計算
# ============================================================

def calc_leg_dimensions(joints: dict, assets_dir: str = "onshape_robot/assets") -> tuple[float, float]:
    """
    代表脚(leg_assembly_v99, サフィックスなし)から L1, L2 を計算する。

    - L1: 肩軸(leg1_top_2_v64) → 膝軸(leg1_center_v26) の水平投影距離 (2D) (mm)
          URDF ジョイント座標から計算
    - L2: 下脚プレート (leg2_side_2) の実際の長さ (mm)
          STL バウンディングボックスの最長辺から取得 (ジョイント原点は板の付け根で
          足先まで届かないため)
    """
    prefix = "leg_assembly_v99"

    # L1: OnShape 実測値を優先する
    #
    # OnShape 実測 3D 距離: 68.65 mm
    #   Z高低差 3.55 mm を考慮した水平投影距離 (2D) = sqrt(68.65^2 - 3.55^2) ≒ 68.56 mm
    L1_MEASURED = 68.56
    L1 = L1_MEASURED

    # L2: OnShape 実測値を優先する
    #
    # OnShape 実測値 (回転軸中心から接地点まで): 126.0 mm
    L2_MEASURED = 126.0
    L2 = L2_MEASURED

    # 参考（URDF/STL との比較警告用）
    hip_xyz  = find_base_joint(joints, prefix, "leg1_top_2_v64")
    knee_xyz = find_base_joint(joints, prefix, "leg1_center_v26")
    if hip_xyz is not None and knee_xyz is not None:
        L1_urdf = math.sqrt((hip_xyz[0] - knee_xyz[0])**2 + (hip_xyz[1] - knee_xyz[1])**2) * 1000.0
        if abs(L1_urdf - L1_MEASURED) > 10.0:
            print(f"  WARNING: URDFのL1={L1_urdf:.1f}mm と実測値L1={L1_MEASURED}mm の差が大きいです。")

    stl_candidates = glob.glob(f"{assets_dir}/leg2_side_2*.stl")
    if stl_candidates:
        L2_stl = max(stl_max_dimension(p) for p in stl_candidates)
        if abs(L2_stl - L2_MEASURED) > 20:
            print(f"  WARNING: STL推定L2={L2_stl:.1f}mm と実測値L2={L2_MEASURED}mm の差が大きいです。")

    return L1, L2


def calc_shoulder_offsets(joints: dict) -> tuple[dict, float]:
    """
    各脚アセンブリの肩ジョイント(leg1_top_2_v64)の位置から
    ボディ中心に対する X/Y オフセット(mm)を計算し、
    原点からの距離 R も返す。

    CAD 上でアセンブリが回転配置されているため atan2 でソートして分類し、
    距離 R の平均から 45° 正方形配置を再構成する。

    atan2 降順の分類 (実測値に基づく):
        160°(-39,14) → BR,  70°(14,39) → FR,
        -20°(39,-14) → FL, -110°(-14,-39) → BL
    """
    leg_prefixes = [
        "leg_assembly_v99",
        "leg_assembly_v99__1",
        "leg_assembly_v99__2",
        "leg_assembly_v99__3",
    ]

    points: list[tuple[float, float]] = []
    for prefix in leg_prefixes:
        base_key = f"base_link_to_{prefix}"
        base_xyz = joints.get(base_key, {}).get("xyz", [0, 0, 0])
        local_xyz = find_base_joint(joints, prefix, "leg1_top_2_v64")
        if local_xyz is None:
            raise KeyError(f"肩ジョイントが見つかりません: prefix={prefix}")
        gx = (base_xyz[0] + local_xyz[0]) * 1000.0
        gy = (base_xyz[1] + local_xyz[1]) * 1000.0
        points.append((gx, gy))

    if len(points) != 4:
        raise ValueError(f"4脚の座標が揃いませんでした: {points}")

    # OnShape 実測値: 左右間隔 = 前後間隔 = 99.5 mm
    #   中心からの片側オフセット = 99.5 / 2 = 49.75 mm
    d = 49.75

    offsets = {
        "FR": ( d,  d),
        "FL": ( d, -d),
        "BR": (-d,  d),
        "BL": (-d, -d),
    }

    # URDF との比較警告用
    R_urdf = sum(math.sqrt(x**2 + y**2) for x, y in points) / 4.0
    d_urdf = R_urdf / math.sqrt(2.0)
    if abs(d_urdf - d) > 5.0:
        print(f"  WARNING: URDFの肩オフセット={d_urdf:.1f}mm と実測値={d}mm の差が大きいです。")

    return offsets, d



# ============================================================
# パラメータ計算
# ============================================================

def derive_params(L1: float, L2: float, shoulder_d: float) -> dict:
    """
    物理寸法から全派生パラメータを計算して返す。

    パラメータ定義:
      DEFAULT_Z       : 直立時の胴体高さ。Knee が垂直下向き → 足先は Knee の真下 L2 分 → ≈ L2
      STANCE_OFFSET   : 「気をつけ」姿勢での肩→足先の水平オフセット。
                        Hip を 45° に向けたとき = L1 * cos(45°)
      STEP_HEIGHT     : 遊脚の持ち上げ量。L2 の 40% 程度を維持
      STEP_LENGTH     : 一歩の最大歩幅。L1 の 60% 程度を維持
      BODY_SHIFT_Y    : 横転防止の体重移動量。shoulder_d の 40% 程度
    """
    default_z       = round(L2, 1)
    stance_offset   = round(L1 * math.cos(math.radians(45)), 2)
    step_height     = round(L2 * 0.40, 1)
    step_length     = round(L1 * 0.75, 1)
    body_shift_y    = round(shoulder_d * 0.40, 1)

    return {
        "DEFAULT_Z":        default_z,
        "STANCE_OFFSET_X":  stance_offset,
        "STANCE_OFFSET_Y":  stance_offset,
        "STEP_HEIGHT":      step_height,
        "STEP_LENGTH":      step_length,
        "BODY_SHIFT_Y":     body_shift_y,
    }


# ============================================================
# robot_config.yaml 更新
# ============================================================

def update_robot_config(config_path: str, L1: float, L2: float,
                        offsets: dict, derived: dict) -> None:
    """
    robot_config.yaml の物理寸法 + 派生パラメータを上書き保存する。
    チューニング値 (PID ゲイン, サーボ設定, タイミング等) は維持する。
    """
    if os.path.exists(config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
    else:
        config = {}

    # --- 物理寸法 ---
    config["L1"] = round(L1, 2)
    config["L2"] = round(L2, 2)
    for leg, (ox, oy) in offsets.items():
        config[f"{leg}_OFFSET_X"] = round(ox, 2)
        config[f"{leg}_OFFSET_Y"] = round(oy, 2)

    # --- 派生パラメータ ---
    config.update(derived)

    with open(config_path, "w", encoding="utf-8") as f:
        yaml.dump(config, f, allow_unicode=True,
                  default_flow_style=False, sort_keys=False)


def main():
    urdf_path   = "onshape_robot/robot.urdf"
    config_path = "robot_config.yaml"

    if not os.path.exists(urdf_path):
        print(f"Error: {urdf_path} が見つかりません")
        return

    print("URDF を解析中...")
    joints = get_joints(urdf_path)

    # --- 寸法計算 ---
    try:
        L1, L2 = calc_leg_dimensions(joints)
    except KeyError as e:
        print(f"Error: L1/L2 の計算失敗: {e}")
        return

    try:
        offsets, shoulder_d = calc_shoulder_offsets(joints)
    except (KeyError, ValueError) as e:
        print(f"Error: 肩オフセットの計算失敗: {e}")
        return

    # --- 派生パラメータ計算 ---
    derived = derive_params(L1, L2, shoulder_d)

    # --- 更新 ---
    update_robot_config(config_path, L1, L2, offsets, derived)

    # --- ログ出力 ---
    print("=== 物理寸法 ===")
    print(f"  L1 (肩→膝リンク長)    = {L1:.2f} mm")
    print(f"  L2 (膝→足先リンク長)  = {L2:.2f} mm")
    print(f"  肩オフセット半径 d     = {shoulder_d:.2f} mm")
    for leg, (ox, oy) in offsets.items():
        print(f"  {leg}_OFFSET = ({ox:.2f}, {oy:.2f}) mm")
    print()
    print("=== 派生パラメータ ===")
    for k, v in derived.items():
        print(f"  {k} = {v}")
    print()
    print("✓ robot_config.yaml を更新しました")

    # --- C++ ヘッダー再生成 ---
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import generate_config
    generate_config.main()


if __name__ == "__main__":
    main()
