# bus.py
import logging
import math
import struct
try:
    from dynamixel_sdk import *
except ImportError:
    raise ImportError(
        "dynamixel_sdk가 필요합니다. 설치: pip install dynamixel_sdk"
    )

from config import *
from motor_core import Motor, MotorCalibration, MotorNormMode

# 로깅 설정 (터미널에 깔끔하게 출력하기 위함)
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

class DynamixelBus:
    """
    다이나믹셀 하드웨어 통신을 전담하는 Core 클래스 (Hardware Abstraction Layer)
    ROS 2와 무관한 순수 파이썬 로직만 포함합니다.
    """
    def __init__(self, port=DXL_PORT, baudrate=DXL_BAUDRATE, calibration: dict[str, MotorCalibration] | None = None):
        self.port_name = port
        self.baudrate = baudrate
        self.is_connected = False
        
        self.calibration = calibration or {}
        self.is_calibrated = len(self.calibration) > 0

        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        # SyncWrite 핸들러 (목표 위치 동시 전송용)
        self.sync_writer = GroupSyncWrite(
            self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, 4
        )
        
        # 로봇 영점(Origin) 펄스 저장 딕셔너리
        self.origins = {}

    # ========================================
    # 1. Connection (연결 관리)
    # ========================================
    def connect(self) -> bool:
        """포트를 열고 보드레이트를 설정합니다."""
        if not self.port_handler.openPort():
            logger.error(f"❌ 포트 열기 실패: {self.port_name}")
            return False
        if not self.port_handler.setBaudRate(self.baudrate):
            logger.error(f"❌ 보드레이트 설정 실패: {self.baudrate}")
            return False
            
        self.is_connected = True
        logger.info(f"✅ Dynamixel Bus 연결 성공 ({self.port_name} @ {self.baudrate}bps)")
        return True

    def disconnect(self, disable_torque=True):
        """안전하게 토크를 끄고 포트를 닫습니다."""
        if self.is_connected:
            if disable_torque:
                self.set_torque(LEADER_IDS + FOLLOWER_IDS, False)
            self.port_handler.closePort()
            self.is_connected = False
            logger.info("🔌 연결 해제됨.")

    def __del__(self):
        """객체가 소멸될 때 안전하게 연결을 끊습니다."""
        self.disconnect()

    # ========================================
    # 2. Hardware Configuration (하드웨어 설정)
    # ========================================
    def set_torque(self, motor_ids: list, enable: bool):
        """특정 모터들의 토크(힘)를 켜거나 끕니다."""
        val = TORQUE_ENABLE if enable else TORQUE_DISABLE
        for dxl_id in motor_ids:
            res, err = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, val)
            if res != COMM_SUCCESS:
                logger.warning(f"[{dxl_id}번] 토크 제어 실패: {self.packet_handler.getTxRxResult(res)}")

    def set_operating_mode(self, motor_ids: list, mode: int):
        """구동 모드를 설정합니다. (반드시 토크가 꺼진 상태에서 해야 함)"""
        for dxl_id in motor_ids:
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_OPERATING_MODE, mode)

    def set_profile_velocity(self, motor_ids: list, velocity: int):
        """모터의 최대 이동 속도를 설정합니다."""
        for dxl_id in motor_ids:
            self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, ADDR_PROFILE_VELOCITY, velocity)

    def reboot(self, motor_ids: list):
        """모터를 재부팅합니다. (오류 해제 및 멀티턴 초기화용)"""
        for dxl_id in motor_ids:
            self.packet_handler.reboot(self.port_handler, dxl_id)
            logger.info(f"[{dxl_id}번] 재부팅 명령 전송됨.")

    # ========================================
    # 3. Read/Write Raw (저수준 펄스 통신)
    # ========================================
    def read_raw_positions(self, motor_ids: list) -> dict:
        """모터의 현재 펄스(Raw) 값을 읽어옵니다. (오버플로우 처리 포함)"""
        positions = {}
        for dxl_id in motor_ids:
            pos, res, err = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
            if res == COMM_SUCCESS:
                # 32bit 음수(2의 보수) 처리 로직 (회원님 기존 코드 반영)
                if pos > 2147483647:
                    pos -= 4294967296
                positions[dxl_id] = pos
            else:
                logger.error(f"[{dxl_id}번] 위치 읽기 실패")
                positions[dxl_id] = 0 # 실패 시 임시값
        return positions

    def write_raw_position(self, motor_id: int, pulse: int):
        """단일 모터에 펄스(Raw) 값을 바로 전송합니다. (Jog 이동 등에 사용)"""
        self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_POSITION, pulse)

    # ========================================
    # 4. Calibration & Kinematics (영점 및 각도 변환)
    # ========================================
    def capture_origins(self, motor_ids: list):
        """현재 상태를 읽어 '초기 영점(Origin)'으로 기록합니다."""
        current_pulses = self.read_raw_positions(motor_ids)
        self.origins.update(current_pulses)
        logger.info(f"📍 영점 설정 완료: {self.origins}")

    def get_angles(self, motor_ids: list, is_follower: bool = True) -> dict:
        """
        [Normalize 역할] 현재 모터 위치를 읽어 영점 대비 변화량을 계산한 후, 
        기어비와 방향을 반영하여 각도(Degree)로 변환해 반환합니다.
        """
        raw_positions = self.read_raw_positions(motor_ids)
        angles_deg = {}
        
        for dxl_id in motor_ids:
            current_pulse = raw_positions[dxl_id]
            origin_pulse = self.origins.get(dxl_id, 2048) # 영점이 없으면 기본 중앙값 사용
            
            delta_pulse = current_pulse - origin_pulse
            
            # 리더는 기어비/방향 무시, 팔로워는 적용
            ratio = GEAR_RATIOS.get(dxl_id, 1) if is_follower else 1
            direction = DIRECTION_MAP.get(dxl_id, 1) if is_follower else 1
            
            # 펄스 -> 각도 변환: (변화량 * 360) / (4096 * 기어비 * 방향)
            angle = (delta_pulse * 360.0) / (4096.0 * ratio * direction)
            angles_deg[dxl_id] = angle
            
        return angles_deg

    def set_angles_sync(self, target_angles_dict: dict, is_follower: bool = True):
        """
        [Denormalize 역할] 목표 각도(Degree) 딕셔너리를 받아 
        기어비와 방향을 반영해 목표 펄스로 변환하고 SyncWrite로 동시 전송합니다.
        예: target_angles_dict = {1: 90.0, 2: -45.0, ...}
        """
        self.sync_writer.clearParam()
        
        for dxl_id, target_angle_deg in target_angles_dict.items():
            origin_pulse = self.origins.get(dxl_id, 2048)
            
            ratio = GEAR_RATIOS.get(dxl_id, 1) if is_follower else 1
            direction = DIRECTION_MAP.get(dxl_id, 1) if is_follower else 1
            
            # 각도 -> 펄스 변환: 각도 * 기어비 * (4096 / 360) * 방향
            pulse_change = int(target_angle_deg * ratio * (4096.0 / 360.0) * direction)
            
            # 최종 목표 펄스 = 영점 + 변화량
            goal_pulse = (origin_pulse + pulse_change) & 0xFFFFFFFF
            
            # 4바이트(32bit) 파라미터 쪼개기
            param = [
                DXL_LOBYTE(DXL_LOWORD(goal_pulse)), DXL_HIBYTE(DXL_LOWORD(goal_pulse)), 
                DXL_LOBYTE(DXL_HIWORD(goal_pulse)), DXL_HIBYTE(DXL_HIWORD(goal_pulse))
            ]
            self.sync_writer.addParam(dxl_id, param)

        # 버스에 물려있는 모터들 동시 출발!
        self.sync_writer.txPacket()