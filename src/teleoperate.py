#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from dynamixel_sdk import *
import math

# 회원님의 정렬 프로그램 (1~7번 모터의 원점을 잡아주고 포트 반환)
import calibrate_origin_keyboard as calib

# --- 하드웨어 ID 설정 (수정됨) ---
LEADER_IDS = [21, 22, 23, 24, 25, 26, 27]   # 마스터 로봇 (XL430, 조종기)
FOLLOWER_IDS = [1, 2, 3, 4, 5, 6, 7]        # 슬레이브 로봇 (기어 박스 장착)

# --- 팔로워(1~7번) 기어비 및 방향 설정 ---
GEAR_RATIOS = {1: 25, 2: 25, 3: 1, 4: 15, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: -1, 2: -1, 3: 1, 4: -1, 5: 1, 6: -1, 7: -1}
BASE_MAX_VELOCITY = 32767

class TeleopNode(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('teleop_node')
        
        # 외부에서 넘겨받은 포트/패킷 핸들러 (U2D2 1개로 통신)
        self.portHandler = port_handler
        self.packetHandler = packet_handler
        
        # Follower 위치 명령용 SyncWrite (목표 위치 주소 116, 4바이트)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)
        
        self.leader_initial_pulses = {}
        self.follower_initial_pulses = {}
        
        # 🌟 ROS 2 Pub/Sub 설정 (토픽명: /teleop_angles)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/teleop_angles', 10)
        self.subscription = self.create_subscription(Float64MultiArray, '/teleop_angles', self.follower_callback, 10)
        
        # 로봇 초기 상태 읽기 및 토크 설정
        self.initialize_robots()
        
        # 0.05초(20Hz)마다 Leader(21~27)의 각도를 읽어서 퍼블리시
        self.timer = self.create_timer(0.05, self.publish_leader_angles)
        
        self.get_logger().info('✅ 원격 조종(Teleop) 준비 완료! Leader(21~27)를 손으로 움직여보세요.')

    def initialize_robots(self):
        """리더와 팔로워의 초기 0점을 기록하고 토크를 알맞게 설정합니다."""
        
        # 1. Leader (21~27): 사람이 움직여야 하므로 토크 OFF (Position 모드여도 토크 끄면 Free Run 가능!)
        for l_id in LEADER_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, l_id, 64, 0) # Torque OFF
            
            # 현재 초기 위치 읽어서 영점으로 저장
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, l_id, 132)
            if pos > 2147483647: pos -= 4294967296
            self.leader_initial_pulses[l_id] = pos
            
        # 2. Follower (1~7): 동작해야 하므로 토크 ON!
        max_ratio = max(GEAR_RATIOS.values())
        for f_id in FOLLOWER_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, f_id, 64, 1) # Torque ON
            
            # 현재 초기 위치 읽어서 영점으로 저장
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, f_id, 132)
            if pos > 2147483647: pos -= 4294967296
            self.follower_initial_pulses[f_id] = pos
            
            # 속도 동기화 (기어비가 큰 모터가 더 빨리 돌도록 Profile Velocity 설정)
            vel = max(1, int(BASE_MAX_VELOCITY * (GEAR_RATIOS[f_id] / max_ratio)))
            self.packetHandler.write4ByteTxRx(self.portHandler, f_id, 112, vel)

    def publish_leader_angles(self):
        """Leader의 위치 변화량을 읽어서 각도(Degree)로 변환 후 Topic으로 쏩니다."""
        msg = Float64MultiArray()
        angles_deg = []
        
        for l_id in LEADER_IDS:
            pos, comm_result, _ = self.packetHandler.read4ByteTxRx(self.portHandler, l_id, 132)
            if comm_result == COMM_SUCCESS:
                if pos > 2147483647: pos -= 4294967296
                
                # 🌟 기준점(초기 위치)으로부터 얼마나 변했는지 계산
                delta_pulse = pos - self.leader_initial_pulses[l_id]
                
                # 펄스를 각도로 변환 (1바퀴 360도 = 4096펄스 기준)
                angle = delta_pulse * (360.0 / 4096.0)
                angles_deg.append(angle)
            else:
                angles_deg.append(0.0) # 통신 에러 시 기본값
                
        msg.data = angles_deg
        self.publisher_.publish(msg)

    def follower_callback(self, msg):
        """Topic으로 들어온 7개의 각도를 받아 Follower(1~7)에 기어비를 곱해 명령을 내립니다."""
        target_angles = msg.data
        
        if len(target_angles) != 7: return

        # target_angles 배열 안에는 Leader(21~27)의 각도가 순서대로 들어있음
        for i, target_angle_deg in enumerate(target_angles):
            f_id = FOLLOWER_IDS[i]
            
            # 🌟 [핵심] 리더가 움직인 각도 * 팔로워 기어비 * 방향 = 팔로워 목표 펄스 변화량
            pulse_change = int(target_angle_deg * GEAR_RATIOS[f_id] * (4096.0 / 360.0) * DIRECTION_MAP[f_id])
            
            # 팔로워 초기 위치에 계산된 변화량을 더함
            goal = (self.follower_initial_pulses[f_id] + pulse_change) & 0xFFFFFFFF
            
            # SyncWrite 파라미터 구성 (4바이트 쪼개기)
            param = [DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)), 
                     DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))]
            self.groupSyncWrite.addParam(f_id, param)

        # 7개 모터로 한 번에 전송! (동기화)
        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()


def main(args=None):
    # 1. 원점 정렬 먼저 진행 (이때 calibrate_origin_keyboard.py는 1~7번 모터용이어야 합니다)
    print("\n[알림] 원점 정렬 프로그램을 먼저 실행합니다. (Follower 로봇 자세 교정)")
    port_h, packet_h = calib.calibrate_origin()
    
    # 2. 정렬 완료 후 Teleop 진입
    print("\n[알림] 정렬 완료! ROS 2 원격 조종(Teleop) 노드를 시작합니다.")
    rclpy.init(args=args)
    
    # 생성된 핸들러를 넘겨주어 포트 충돌 방지
    node = TeleopNode(port_h, packet_h)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 안전하게 포트 닫기
        if port_h.is_open:
            port_h.closePort()
        print("\n프로그램 종료.")

if __name__ == '__main__':
    main()