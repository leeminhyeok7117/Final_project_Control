#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import math
import time
from dynamixel_sdk import *
import calibrate_origin_keyboard as calib
import subprocess

# --- 하드웨어 설정 ---
JOINT_NAME_TO_ID = {
    '회전-30': 1, '회전-22': 2, '회전-23': 3,
    '회전-24': 4, '회전-25': 5, '회전-26': 6, '회전-28': 7
}
GEAR_RATIOS = {1: 15, 2: 15, 3: 5, 4: 5, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: 1, 2: 1, 3: -1, 4: 1, 5: -1, 6: -1, 7: 1}

class DynamixelActionServer(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('dynamixel_action_server')
        
        # 가짜 로봇(방해꾼) 차단
        subprocess.run(['ros2', 'control', 'set_controller_state', 'joint_state_broadcaster', 'inactive'], capture_output=True)
        subprocess.run(['ros2', 'control', 'set_controller_state', 'joint_trajectory_controller', 'inactive'], capture_output=True)
        
        self.portHandler = port_handler
        self.packetHandler = packet_handler
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)
        self.initial_motor_pulses = {}
        self.target_joints = list(JOINT_NAME_TO_ID.keys())
        
        # 원점 펄스 캡처 및 속도 제한 해제
        self.capture_current_state_as_origin()
        
        # RViz 동기화용 퍼블리셔 및 현재 각도 저장 변수
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.current_angles = [0.0] * len(self.target_joints)
        
        # 로봇 위치 지속 방송 (0.05초)
        self.state_timer = self.create_timer(0.05, self.publish_current_state)
        
        # 액션 서버 오픈
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info('🤖 다이나믹셀 액션 서버 가동 완료! 명령을 기다립니다...')

    def capture_current_state_as_origin(self):
        target_ids = list(JOINT_NAME_TO_ID.values())
        for dxl_id in target_ids:
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 108, 20)
            # 1. 현재 원점 펄스 읽기
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132) 
            if pos > 2147483647: pos -= 4294967296
            self.initial_motor_pulses[dxl_id] = pos
            
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, 0)

    def publish_current_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.target_joints
        msg.position = self.current_angles
        self.joint_pub.publish(msg)


    def execute_callback(self, goal_handle):
            self.get_logger().info('📥 궤적 명령 수신! 모터 구동을 시작합니다.')
            
            trajectory = goal_handle.request.trajectory
            points = trajectory.points
            joint_names = trajectory.joint_names
            
            if not points:
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            start_time = time.time()
            last_goal_pulses = {} # 마지막 목표 위치 저장용

            for point in points:
                t_target = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
                
                # MoveIt이 지시한 시간이 될 때까지 대기
                while (time.time() - start_time) < t_target:
                    time.sleep(0.001)

                angles = []
                for i, name in enumerate(self.target_joints):       
                    if name in joint_names:
                        # MoveIt이나 클라이언트가 각도를 보내줬으면 그 각도를 따름
                        idx = joint_names.index(name)
                        rad = point.positions[idx]
                    else:
                        # 명단에 없으면(예: RViz에서 팔만 움직일 때) 현재 각도를 그대로 유지!
                        rad = self.current_angles[i]
                        
                    angles.append(rad)
                    
                    dxl_id = JOINT_NAME_TO_ID[name]
                    
                    delta_deg = math.degrees(rad)
                    pulse_change = int(delta_deg * GEAR_RATIOS[dxl_id] * (4096.0/360.0) * DIRECTION_MAP[dxl_id])
                    goal = (self.initial_motor_pulses[dxl_id] + pulse_change)
                    
                    # 마지막 포인트의 목표 펄스 저장
                    last_goal_pulses[dxl_id] = goal
                    
                    # 4바이트 패킷 처리
                    goal_unsigned = goal & 0xFFFFFFFF
                    param = [DXL_LOBYTE(DXL_LOWORD(goal_unsigned)), DXL_HIBYTE(DXL_LOWORD(goal_unsigned)), 
                            DXL_LOBYTE(DXL_HIWORD(goal_unsigned)), DXL_HIBYTE(DXL_HIWORD(goal_unsigned))]
                    self.groupSyncWrite.addParam(dxl_id, param)

                self.groupSyncWrite.txPacket()
                self.groupSyncWrite.clearParam()
                self.current_angles = angles

            # -----------------------------------------------------------------
            # 🌟 추가된 부분: 모든 모터가 목표 지점에 도달할 때까지 기다림
            # -----------------------------------------------------------------
            self.get_logger().info('⏳ 마지막 위치 도달 대기 중...')
            
            reaching_goal = False
            timeout_start = time.time()
            
            while not reaching_goal:
                all_done = True
                for name in self.target_joints:
                    dxl_id = JOINT_NAME_TO_ID[name]
                    # 현재 위치 읽기 (Present Position: 132)
                    cur_pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132)
                    if cur_pos > 2147483647: cur_pos -= 4294967296
                    
                    # 목표 위치와 현재 위치의 오차가 허용 범위(예: 20펄스) 이내인지 확인
                    if abs(cur_pos - last_goal_pulses[dxl_id]) > 25: # 약 2도 이내 오차
                        all_done = False
                        break
                
                if all_done:
                    reaching_goal = True
                
                # 무한 루프 방지용 타임아웃 (예: 2초)
                if (time.time() - timeout_start) > 5.0:
                    self.get_logger().warn('⚠️ 도달 타임아웃 발생 (일부 모터가 목표치에 미달)')
                    break
                    
                time.sleep(0.01)
            # -----------------------------------------------------------------

            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('✅ 구동 및 위치 도달 완료! 성공 보고를 올렸습니다.')
            
            return result

def main(args=None):
    print("\n[액션 서버] 원점 정렬을 진행합니다. 정렬 후 q를 눌러주세요.")
    port_h, packet_h = calib.calibrate_origin()
    
    rclpy.init(args=args)
    node = DynamixelActionServer(port_h, packet_h)
    
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if port_h.is_open:
            port_h.closePort()
        print("\n[액션 서버] 종료되었습니다.")

if __name__ == '__main__':
    main()