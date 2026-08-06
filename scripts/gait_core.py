from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import yaml

@dataclass(frozen=True)
class RobotConfig:
    L1: float
    L2: float
    FR_X: float
    FR_Y: float
    FL_X: float
    FL_Y: float
    BR_X: float
    BR_Y: float
    BL_X: float
    BL_Y: float
    STANCE_OFFSET_X: float
    STANCE_OFFSET_Y: float

    DEFAULT_Z:      float
    STEP_HEIGHT:    float
    STEP_LENGTH:    float
    CYCLE_TIME:     int
    BODY_SHIFT_X:   float
    BODY_SHIFT_Y:   float
    SWING_RATIO:    float
    HEADING_ANGLE:  float
    STRIDE_X_DIV:   float
    STRIDE_Y_DIV:   float
    STRIDE_ROT_DIV: float

def load_config(path: Optional[str] = None) -> RobotConfig:
    if path is None:
        path = os.path.join(os.path.dirname(__file__), "..", "robot_config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        c = yaml.safe_load(f)
    return RobotConfig(
        L1=float(c["L1"]),
        L2=float(c["L2"]),
        FR_X=float(c["FR_OFFSET_X"]),
        FR_Y=float(c["FR_OFFSET_Y"]),
        FL_X=float(c["FL_OFFSET_X"]),
        FL_Y=float(c["FL_OFFSET_Y"]),
        BR_X=float(c["BR_OFFSET_X"]),
        BR_Y=float(c["BR_OFFSET_Y"]),
        BL_X=float(c["BL_OFFSET_X"]),
        BL_Y=float(c["BL_OFFSET_Y"]),
        STANCE_OFFSET_X=float(c["STANCE_OFFSET_X"]),
        STANCE_OFFSET_Y=float(c["STANCE_OFFSET_Y"]),
        DEFAULT_Z=float(c["DEFAULT_Z"]),
        STEP_HEIGHT=float(c["STEP_HEIGHT"]),
        STEP_LENGTH=float(c["STEP_LENGTH"]),
        CYCLE_TIME=int(c["CYCLE_TIME"]),
        BODY_SHIFT_X=float(c["BODY_SHIFT_X"]),
        BODY_SHIFT_Y=float(c["BODY_SHIFT_Y"]),
        SWING_RATIO=float(c["SWING_RATIO"]),
        HEADING_ANGLE=float(c["HEADING_ANGLE"]),
        STRIDE_X_DIV=float(c["STRIDE_X_DIV"]),
        STRIDE_Y_DIV=float(c["STRIDE_Y_DIV"]),
        STRIDE_ROT_DIV=float(c["STRIDE_ROT_DIV"]),
    )

def _home_pos(ox: float, oy: float, cfg: RobotConfig) -> Tuple[float, float]:
    hx = ox + (cfg.STANCE_OFFSET_X if ox > 0 else -cfg.STANCE_OFFSET_X)
    hy = oy + (cfg.STANCE_OFFSET_Y if oy > 0 else -cfg.STANCE_OFFSET_Y)
    return hx, hy

_cfg = load_config()

leg_names = ["FR", "BL", "FL", "BR"]

offsets: list[Tuple[float, float]] = [
    (_cfg.FR_X, _cfg.FR_Y),
    (_cfg.BL_X, _cfg.BL_Y),
    (_cfg.FL_X, _cfg.FL_Y),
    (_cfg.BR_X, _cfg.BR_Y),
]

base_angles: list[float] = []
for _ox, _oy in offsets:
    _hx, _hy = _home_pos(_ox, _oy, _cfg)
    base_angles.append(np.arctan2(_hy - _oy, _hx - _ox))

L1             = _cfg.L1
L2             = _cfg.L2
FR_X           = _cfg.FR_X
FR_Y           = _cfg.FR_Y
FL_X           = _cfg.FL_X
FL_Y           = _cfg.FL_Y
BR_X           = _cfg.BR_X
BR_Y           = _cfg.BR_Y
BL_X           = _cfg.BL_X
BL_Y           = _cfg.BL_Y
STANCE_OFFSET_X = _cfg.STANCE_OFFSET_X
STANCE_OFFSET_Y = _cfg.STANCE_OFFSET_Y
DEFAULT_Z      = _cfg.DEFAULT_Z
STEP_HEIGHT    = _cfg.STEP_HEIGHT
STEP_LENGTH    = _cfg.STEP_LENGTH
CYCLE_TIME     = _cfg.CYCLE_TIME
BODY_SHIFT_X   = _cfg.BODY_SHIFT_X
BODY_SHIFT_Y   = _cfg.BODY_SHIFT_Y
SWING_RATIO    = _cfg.SWING_RATIO
HEADING_ANGLE  = _cfg.HEADING_ANGLE
STRIDE_X_DIV   = _cfg.STRIDE_X_DIV
STRIDE_Y_DIV   = _cfg.STRIDE_Y_DIV
STRIDE_ROT_DIV = _cfg.STRIDE_ROT_DIV

def calculate_leg_pos(
    phase: float,
    leg_id: int,
    vx: float,
    vy: float,
    vtheta: float,
    cfg: Optional[RobotConfig] = None,
) -> Tuple[float, float, float, bool, float, float]:
    if cfg is None:
        cfg = _cfg

    ox, oy = offsets[leg_id]
    home_x, home_y = _home_pos(ox, oy, cfg)

    heading_rad = np.radians(cfg.HEADING_ANGLE)
    cos_h = np.cos(heading_rad)
    sin_h = np.sin(heading_rad)
    rot_vx = vx * cos_h - vy * sin_h
    rot_vy = vx * sin_h + vy * cos_h

    stride_x   = rot_vx * (cfg.STEP_LENGTH / cfg.STRIDE_X_DIV)
    stride_y   = rot_vy * (cfg.STEP_LENGTH / cfg.STRIDE_Y_DIV)
    stride_rot = vtheta * (np.pi / cfg.STRIDE_ROT_DIV)

    leg_phase = (phase - leg_id * 0.25 + 1.0) % 1.0

    body_offset_x = -np.sin(phase * 4.0 * np.pi) * cfg.BODY_SHIFT_X
    body_offset_y = -np.cos(phase * 2.0 * np.pi) * cfg.BODY_SHIFT_Y

    is_swing = (leg_phase < cfg.SWING_RATIO)

    if is_swing:
        s = leg_phase / cfg.SWING_RATIO
        h = np.sin(s * np.pi)
        target_z = -cfg.DEFAULT_Z + h * h * cfg.STEP_HEIGHT
    else:
        s = (leg_phase - cfg.SWING_RATIO) / (1.0 - cfg.SWING_RATIO)
        target_z = -cfg.DEFAULT_Z

    sign    = 1.0 if is_swing else -1.0
    s_center = sign * (s - 0.5)

    target_x = home_x + stride_x * s_center - home_y * stride_rot * s_center - body_offset_x
    target_y = home_y + stride_y * s_center + home_x * stride_rot * s_center - body_offset_y

    return target_x, target_y, target_z, is_swing, body_offset_x, body_offset_y
