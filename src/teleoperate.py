#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from dynamixel_sdk import *
import math
import time

# 원점 정렬 프로그램 (Follower 원점을 잡아주고 포트 반환)
import calibrate_origin_keyboard as calib

# --- 하드웨어 ID 설정 ---
LEADER_IDS   = [21, 22, 23, 24, 25, 26, 27]  # 마스터 로봇 (XL430) — 7개
FOLLOWER_IDS = [1,  2,  3,  4,  5,  6,  7]   # 슬레이브 로봇 — 7개

# --- 팔로워(1~7번) 기어비 및 방향 설정 (action.py와 동일) ---
GEAR_RATIOS   = {1: 15, 2: 15, 3: 5, 4: 5, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: -1, 2: -1, 3: -1, 4: -1, 5: 1, 6: -1, 7: -1}

# Profile Acceleration (주소 108)
PROFILE_ACCEL = 20
# Profile Velocity (주소 112): 0 = 무제한 (실시간 추종)
PROFILE_VEL   = 0

# Leader 홈 복귀 설정
# XL430 Position Control Mode: 중앙(0deg) = 2048 펄스
LEADER_HOME_PULSE    = 2048
LEADER_HOME_VELOCITY = 100   # 홈 복귀 속도 (느리게)
LEADER_HOME_TIMEOUT  = 5.0   # 타임아웃 (초)
LEADER_HOME_TOL      = 30    # 도달 판정 허용 오차 (펄스)

PULSES_PER_REV = 4096
DEG_PER_PULSE  = 360.0 / PULSES_PER_REV

# 다이나믹셀 주소
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VEL   = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POS   = 132


class TeleopNode(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('teleop_node')

        self.portHandler   = port_handler
        self.packetHandler = packet_handler

        # Follower 위치 명령용 SyncWrite (Goal Position 주소 116, 4바이트)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, 4
        )

        self.leader_initial_pulses   = {}
        self.follower_initial_pulses = {}

        # ROS 2 Pub/Sub
        self.publisher_   = self.create_publisher(
            Float64MultiArray, '/teleop_angles', 10
        )
        self.subscription = self.create_subscription(
            Float64MultiArray, '/teleop_angles', self.follower_callback, 10
        )

        # 초기화 (Leader 홈 복귀 → Follower 토크 ON)
        self.initialize_robots()

        # 20 Hz 주기로 Leader 각도 퍼블리시
        self.timer = self.create_timer(0.05, self.publish_leader_angles)

        self.get_logger().info(
            '✅ 원격 조종(Teleop) 준비 완료! Leader(21~27)를 손으로 움직여보세요.'
        )

    # ------------------------------------------------------------------
    def _home_leaders(self):
        """
        Leader(21~27)를 중앙(2048 = 0deg)으로 복귀 후 토크 OFF.
        순서: 토크 ON → 저속 설정 → 목표 2048 전송 → 도달 대기 → 토크 OFF
        """
        self.get_logger().info('🏠 Leader 원점 복귀 중...')
        ph = self.portHandler
        pk = self.packetHandler

        # ① 토크 ON + 저속 설정
        for l_id in LEADER_IDS:
            pk.write1ByteTxRx(ph, l_id, ADDR_TORQUE_ENABLE, 1)
            pk.write4ByteTxRx(ph, l_id, ADDR_PROFILE_VEL, LEADER_HOME_VELOCITY)

        # ② 목표 위치 2048 전송
        for l_id in LEADER_IDS:
            goal = LEADER_HOME_PULSE & 0xFFFFFFFF
            param = [
                DXL_LOBYTE(DXL_LOWORD(goal)),
                DXL_HIBYTE(DXL_LOWORD(goal)),
                DXL_LOBYTE(DXL_HIWORD(goal)),
                DXL_HIBYTE(DXL_HIWORD(goal)),
            ]
            pk.writeTxRx(ph, l_id, ADDR_GOAL_POSITION, 4, param)

        # ③ 도달 대기
        t_start = time.time()
        while True:
            all_done = True
            for l_id in LEADER_IDS:
                pos, result, _ = pk.read4ByteTxRx(ph, l_id, ADDR_PRESENT_POS)
                if result != COMM_SUCCESS:
                    continue
                if pos > 2147483647:
                    pos -= 4294967296
                if abs(pos - LEADER_HOME_PULSE) > LEADER_HOME_TOL:
                    all_done = False
                    break

            if all_done:
                self.get_logger().info('✅ Leader 원점 복귀 완료!')
                break
            if (time.time() - t_start) > LEADER_HOME_TIMEOUT:
                self.get_logger().warn('⚠️ Leader 홈 복귀 타임아웃 (일부 미도달)')
                break

            time.sleep(0.02)

        # ④ 토크 OFF → 손으로 자유롭게 이동 가능
        for l_id in LEADER_IDS:
            pk.write1ByteTxRx(ph, l_id, ADDR_TORQUE_ENABLE, 0)

    # ------------------------------------------------------------------
    def initialize_robots(self):
        """Leader 홈 복귀 → 영점 기록 → Follower 초기화"""

        # 1. Leader 원점 복귀 후 토크 OFF
        self._home_leaders()

        # 2. Leader 현재 위치를 영점으로 기록
        for l_id in LEADER_IDS:
            pos, _, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, l_id, ADDR_PRESENT_POS
            )
            if pos > 2147483647:
                pos -= 4294967296
            self.leader_initial_pulses[l_id] = pos

        # 3. Follower (1~7) 초기화
        for f_id in FOLLOWER_IDS:
            self.packetHandler.write4ByteTxRx(
                self.portHandler, f_id, ADDR_PROFILE_ACCEL, PROFILE_ACCEL
            )
            self.packetHandler.write4ByteTxRx(
                self.portHandler, f_id, ADDR_PROFILE_VEL, PROFILE_VEL
            )
            self.packetHandler.write1ByteTxRx(
                self.portHandler, f_id, ADDR_TORQUE_ENABLE, 1
            )
            pos, _, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, f_id, ADDR_PRESENT_POS
            )
            if pos > 2147483647:
                pos -= 4294967296
            self.follower_initial_pulses[f_id] = pos

    # ------------------------------------------------------------------
    def publish_leader_angles(self):
        """Leader(21~27) 위치 변화량 → 각도(deg) → 토픽 퍼블리시"""
        msg = Float64MultiArray()
        angles_deg = []

        for l_id in LEADER_IDS:
            pos, comm_result, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, l_id, ADDR_PRESENT_POS
            )
            if comm_result == COMM_SUCCESS:
                if pos > 2147483647:
                    pos -= 4294967296
                delta_pulse = pos - self.leader_initial_pulses[l_id]
                angles_deg.append(delta_pulse * DEG_PER_PULSE)
            else:
                angles_deg.append(0.0)

        msg.data = angles_deg
        self.publisher_.publish(msg)

    # ------------------------------------------------------------------
    # def follower_callback(self, msg):
    #     """토픽으로 들어온 7개 각도(deg) → Follower(1~7) 위치 명령"""
    #     target_angles = msg.data

    #     if len(target_angles) != len(FOLLOWER_IDS):
    #         self.get_logger().warn(
    #             f'각도 데이터 길이 불일치: '
    #             f'수신 {len(target_angles)}개 / 예상 {len(FOLLOWER_IDS)}개'
    #         )
    #         return

    #     for i, target_angle_deg in enumerate(target_angles):
    #         f_id = FOLLOWER_IDS[i]

    #         pulse_change = int(
    #             target_angle_deg
    #             * GEAR_RATIOS[f_id]
    #             * (PULSES_PER_REV / 360.0)
    #             * DIRECTION_MAP[f_id]
    #         )
    #         goal = (self.follower_initial_pulses[f_id] + pulse_change) & 0xFFFFFFFF

    #         param = [
    #             DXL_LOBYTE(DXL_LOWORD(goal)),
    #             DXL_HIBYTE(DXL_LOWORD(goal)),
    #             DXL_LOBYTE(DXL_HIWORD(goal)),
    #             DXL_HIBYTE(DXL_HIWORD(goal)),
    #         ]
    #         self.groupSyncWrite.addParam(f_id, param)

    #     # 7개 모터 동시 전송
    #     self.groupSyncWrite.txPacket()
    #     self.groupSyncWrite.clearParam()
    def follower_callback(self, msg):
        """토픽으로 들어온 7개 각도(deg) → Follower(1~7) 위치 명령"""
        target_angles = msg.data

        if len(target_angles) != len(FOLLOWER_IDS):
            self.get_logger().warn(
                f'각도 데이터 길이 불일치: '
                f'수신 {len(target_angles)}개 / 예상 {len(FOLLOWER_IDS)}개'
            )
            return

        for i, target_angle_deg in enumerate(target_angles):
            f_id = FOLLOWER_IDS[i]

            pulse_change = int(
                target_angle_deg
                * GEAR_RATIOS[f_id]
                * (PULSES_PER_REV / 360.0)
                * DIRECTION_MAP[f_id]
            )
            
            # 목표 펄스 계산
            goal_pulse = self.follower_initial_pulses[f_id] + pulse_change

            # --- [임시 추가] 7번 모터 하한선 제한 (2048 미만 무시/고정) ---
            if f_id == 7:
                if goal_pulse < 2500:
                    goal_pulse = 2500
                elif goal_pulse > 4000:
                    goal_pulse = 4000
            # ----------------------------------------------------------

            goal = goal_pulse & 0xFFFFFFFF

            param = [
                DXL_LOBYTE(DXL_LOWORD(goal)),
                DXL_HIBYTE(DXL_LOWORD(goal)),
                DXL_LOBYTE(DXL_HIWORD(goal)),
                DXL_HIBYTE(DXL_HIWORD(goal)),
            ]
            self.groupSyncWrite.addParam(f_id, param)

        # 7개 모터 동시 전송
        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()


# ----------------------------------------------------------------------
def main(args=None):
    print("\n[알림] Follower 원점 정렬을 진행합니다.")
    port_h, packet_h = calib.calibrate_origin()

    print("\n[알림] 정렬 완료! Teleop 노드를 시작합니다.")
    rclpy.init(args=args)

    node = TeleopNode(port_h, packet_h)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for f_id in FOLLOWER_IDS:
            packet_h.write1ByteTxRx(port_h, f_id, ADDR_TORQUE_ENABLE, 0)
        if port_h.is_open:
            port_h.closePort()
        print("\n프로그램 종료.")


if __name__ == '__main__':
    main()