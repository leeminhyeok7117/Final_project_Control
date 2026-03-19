#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import math
import time
from dynamixel_sdk import *
import calibrate_origin_keyboard as calib # 기존 원점 정렬 코드

# --- 하드웨어 설정 ---
JOINT_NAME_TO_ID = {
    '회전-30': 1, '회전-22': 2, '회전-23': 3,
    '회전-24': 4, '회전-25': 5, '회전-26': 6
}
GEAR_RATIOS = {1: 25, 2: 25, 3: 1, 4: 15, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: 1, 2: 1, 3: -1, 4: 1, 5: 1, 6: -1, 7: 1}
BASE_MAX_VELOCITY = 1000

class DynamixelActionServer(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('dynamixel_action_server')
        
        # 1. 다이나믹셀 하드웨어 세팅 (기존과 동일)
        self.portHandler = port_handler
        self.packetHandler = packet_handler
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)
        self.initial_motor_pulses = {}
        self.target_joints = list(JOINT_NAME_TO_ID.keys())
        self.capture_current_state_as_origin()
        
        # 2. RViz 동기화를 위한 JointState 퍼블리셔 (현재 상태 보고용)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 🌟 3. 액션 서버 생성 (여기가 핵심입니다!)
        # MoveIt은 기본적으로 이 이름('/joint_trajectory_controller/follow_joint_trajectory')으로 액션을 쏩니다.
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info('🤖 다이나믹셀 액션 서버가 켜졌습니다! RViz에서 명령을 내려주세요.')

    def capture_current_state_as_origin(self):
        target_ids = list(JOINT_NAME_TO_ID.values())
        max_ratio = max([GEAR_RATIOS[i] for i in target_ids])
        for dxl_id in target_ids:
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132) 
            if pos > 2147483647: pos -= 4294967296
            self.initial_motor_pulses[dxl_id] = pos
            vel = max(1, int(BASE_MAX_VELOCITY * (GEAR_RATIOS[dxl_id] / max_ratio)))
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, vel)

    # 🌟 4. RViz에서 'Plan & Execute'를 누르면 이 함수가 실행됩니다!
    def execute_callback(self, goal_handle):
        self.get_logger().info('📥 RViz로부터 궤적 명령을 수신했습니다! 이동을 시작합니다.')
        
        # MoveIt이 던져준 궤적 데이터 빼오기
        trajectory = goal_handle.request.trajectory
        points = trajectory.points
        joint_names = trajectory.joint_names
        
        # 시작 지점의 각도를 0초 기준으로 삼기 위해 저장
        initial_planned_angles = {name: points[0].positions[joint_names.index(name)] for name in self.target_joints}
        
        start_time = time.time()
        
        # 받은 궤적을 하나씩 돌면서 모터에 전송 (기존 Phase 2 로직)
        for point in points:
            t_target = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
            
            while (time.time() - start_time) < t_target:
                time.sleep(0.001)

            # 모터 제어 명령 쏘기
            angles = []
            for name in self.target_joints:
                rad = point.positions[joint_names.index(name)]
                angles.append(rad)
                
                dxl_id = JOINT_NAME_TO_ID[name]
                delta_rad = rad - initial_planned_angles[name]
                delta_deg = math.degrees(delta_rad)
                pulse_change = int(delta_deg * GEAR_RATIOS[dxl_id] * (4096.0/360.0) * DIRECTION_MAP[dxl_id])
                goal = (self.initial_motor_pulses[dxl_id] + pulse_change) & 0xFFFFFFFF
                
                param = [DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)), 
                         DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))]
                self.groupSyncWrite.addParam(dxl_id, param)

            self.groupSyncWrite.txPacket()
            self.groupSyncWrite.clearParam()

            # RViz에 현재 위치 방송
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.target_joints
            msg.position = angles
            self.joint_pub.publish(msg)
            
        # 🌟 5. 모두 도착했으면 MoveIt에게 "도착 성공!" 보고하기
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('✅ 궤적 이동 완료 및 결과 보고 완료!')
        
        return result

def main(args=None):
    print("\n[알림] 원점 정렬 프로그램을 먼저 실행합니다. 정렬 후 q를 눌러주세요.")
    port_h, packet_h = calib.calibrate_origin()
    
    rclpy.init(args=args)
    node = DynamixelActionServer(port_h, packet_h)
    
    try:
        # 노드를 끄지 않고 계속 살려두어 언제든 명령을 받을 수 있게 합니다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if port_h.is_open:
            port_h.closePort()
        print("\n프로그램 종료.")

if __name__ == '__main__':
    main()