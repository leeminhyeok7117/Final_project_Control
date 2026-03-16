"""Core motor definitions"""
from dataclasses import dataclass
from enum import Enum


class MotorNormMode(str, Enum):
    """모터 정규화 모드"""
    RANGE_0_100 = "range_0_100"        # 0~100
    RANGE_M100_100 = "range_m100_100"  # -100~100
    DEGREES = "degrees"                 # 각도 (도)


@dataclass
class Motor:
    """모터 정의"""
    id: int             # 모터 ID (1-254)
    model: str          # 모터 모델명 (예: "ax-12a", "xl430-w250")
    norm_mode: MotorNormMode  # 정규화 모드


@dataclass
class MotorCalibration:
    """모터 캘리브레이션 데이터"""
    id: int             # 모터 ID
    drive_mode: int     # Drive mode (0=정상, 1=반전)
    homing_offset: int  # 원점 오프셋
    range_min: int      # 최소 위치
    range_max: int      # 최대 위치