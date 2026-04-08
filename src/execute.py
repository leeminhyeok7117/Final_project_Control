#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import math
import time
from dynamixel_sdk import *

# (주의) 만약 ros2 run으로 실행 시 import 에러가 나면 
# from final_project import calibrate_origin_keyboard as calib 로 변경하세요.
import calibrate_origin_keyboard as calib

# --- 하드웨어 설정 (회원님 기존 설정과 동일) ---
JOINT_NAME_TO_ID = {
    '회전-30': 1, '회전-22': 2, '회전-23': 3,
    '회전-24': 4, '회전-25': 5, '회전-26': 6
}
GEAR_RATIOS = {1: 15, 2: 15, 3: 5, 4: 5, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: 1, 2: 1, 3: -1, 4: 1, 5: -1, 6: -1, 7: 1}
BASE_MAX_VELOCITY = 1000
SPEED_FACTOR = 2.0
class TrajectoryExecutor(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('trajectory_executor')
        
        # 외부에서 생성된 핸들러를 그대로 이어받음 (포트 중복 오픈 방지)
        self.portHandler = port_handler
        self.packetHandler = packet_handler
        
        # SyncWrite 설정 (Goal Position 주소 116, 데이터 길이 4바이트)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)
        
        self.trajectory_data = []
        self.initial_csv_angles = {}
        self.initial_motor_pulses = {}
        
        # 실행 준비
        self.load_csv('waving_trajectory.csv')
        self.capture_current_state_as_origin()
        
        self.get_logger().info('✅ 준비 완료! 2초 뒤에 CSV 궤적대로 움직입니다.')
        self.timer = self.create_timer(2.0, self.execute_trajectory)
        self.started = False

    def capture_current_state_as_origin(self):
        """정렬이 끝난 '현재 상태'를 물리적 0점으로 인식합니다."""
        target_ids = list(JOINT_NAME_TO_ID.values())
        max_ratio = max([GEAR_RATIOS[i] for i in target_ids])
        
        for dxl_id in target_ids:
            # 현재 위치 읽기
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 108, 20)
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132) # ADDR_PRESENT_POSITION
            if pos > 2147483647: pos -= 4294967296
            self.initial_motor_pulses[dxl_id] = pos
            
            # 속도 동기화 설정 (Profile Velocity)
            vel = max(1, int(BASE_MAX_VELOCITY * (GEAR_RATIOS[dxl_id] / max_ratio)))
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, vel)

    def load_csv(self, filename):
        try:
            with open(filename, mode='r') as f:
                reader = csv.reader(f)
                header = next(reader)
                self.joint_names = header[1:]
                for i, row in enumerate(reader):
                    if i == 0:
                        for name, rad in zip(self.joint_names, [float(x) for x in row[1:]]):
                            self.initial_csv_angles[name] = rad
                    self.trajectory_data.append((float(row[0]), [float(x) for x in row[1:]]))
        except Exception as e:
            self.get_logger().error(f"CSV 로드 실패: {e}")
            quit()

    def execute_trajectory(self):
        if self.started: return
        self.started = True
        
        start_time = time.time()
        self.get_logger().info("🚀 손인사 시작! (정상 속도 재생)")

        for t_target, angles in self.trajectory_data:
            
            while (time.time() - start_time) < (t_target/SPEED_FACTOR):
                time.sleep(0.001)

            for name, rad in zip(self.joint_names, angles):
                if name in JOINT_NAME_TO_ID:
                    dxl_id = JOINT_NAME_TO_ID[name]
                    
                    # 상대적 변화량 계산: (현재CSV각도 - 시작CSV각도)
                    delta_rad = rad - self.initial_csv_angles[name]
                    delta_deg = math.degrees(delta_rad)
                    
                    # 펄스 변화량 계산
                    pulse_change = int(delta_deg * GEAR_RATIOS[dxl_id] * (4096.0/360.0) * DIRECTION_MAP[dxl_id])
                    goal = (self.initial_motor_pulses[dxl_id] + pulse_change) & 0xFFFFFFFF
                    
                    # SyncWrite 파라미터 구성
                    param = [DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)), 
                             DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))]
                    self.groupSyncWrite.addParam(dxl_id, param)

            self.groupSyncWrite.txPacket()
            self.groupSyncWrite.clearParam()

        self.get_logger().info("🏁 모든 동작이 완료되었습니다.")
        # 동작 완료 후에도 로봇이 주저앉지 않게 토크를 유지한 채 노드만 종료 대기
        self.create_timer(1.0, rclpy.shutdown)

def main(args=None):
    # STEP 1: 회원님의 기존 원점 정렬 함수 실행
    print("\n[알림] 원점 정렬 프로그램을 먼저 실행합니다.")
    port_h, packet_h = calib.calibrate_origin()
    
    # STEP 2: 정렬이 끝나면(q를 누르면) 이어서 ROS 2 실행
    print("\n[알림] 정렬 완료! ROS 2 노드를 초기화합니다.")
    rclpy.init(args=args)
    node = TrajectoryExecutor(port_h, packet_h)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 모든 게 끝날 때 포트를 닫아줍니다.
        if port_h.is_open:
            port_h.closePort()
        print("\n프로그램 종료.")

if __name__ == '__main__':
    main()