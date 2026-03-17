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

from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState  # 🌟 [추가됨] RViz2 동기화를 위한 JointState 메시지

# --- 하드웨어 설정 ---
JOINT_NAME_TO_ID = {
    '회전-30': 1, '회전-22': 2, '회전-23': 3,
    '회전-24': 4, '회전-25': 5, '회전-26': 6
}
GEAR_RATIOS = {1: 25, 2: 25, 3: 1, 4: 15, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: 1, 2: 1, 3: -1, 4: 1, 5: 1, 6: -1, 7: 1}
BASE_MAX_VELOCITY = 1000

# --- 관절 제한 (이전 디버깅을 통해 얻은 최적의 안전 범위) ---
JOINT_LIMITS_DEG = {
    '회전-30': (-120.0, 120.0),    # 1번 축(허리): 좌우 90도
    '회전-22': (-30.0, 90.0),    # 2번 축(어깨): 뒤로 30도, 앞으로 90도
    '회전-23': (-90.0, 90.0),    # 3번 축: +- 90도
    '회전-24': (-115.0, 10.0),  # 4번 축: +- 115도
    '회전-25': (-90.0, 90.0),    # 5번 축: +- 90도
    '회전-26': (-180.0, 180.0)   # 6번 축(손목): 자유롭게 1바퀴
}

class RealtimeWavingExecutor(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('realtime_waving_executor')
        
        # 1. 다이나믹셀 하드웨어 세팅
        self.portHandler = port_handler
        self.packetHandler = packet_handler
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)
        self.initial_motor_pulses = {}
        self.initial_planned_angles = {}
        self.capture_current_state_as_origin()
        
        # 2. MoveIt 서비스 클라이언트 세팅
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # 🌟 [추가됨] 3. RViz2 화면 업데이트를 위한 JointState 퍼블리셔
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt Planning 서비스를 기다리는 중...')

        # 손인사 타겟 좌표들
        self.targets = [
            {'x': 0.223, 'y': 0.129, 'z': 0.572, 'qx': 0.344, 'qy': -0.618, 'qz': -0.344, 'qw': 0.618},
            {'x': 0.223, 'y': 0.384, 'z': 1.026, 'qx': -0.693, 'qy': 0.141, 'qz': 0.693, 'qw': -0.141},
            {'x': 0.227, 'y': 0.331, 'z': 1.183, 'qx': -0.704, 'qy': 0.068, 'qz': 0.704, 'qw': -0.068},
            {'x': 0.380, 'y': 0.298, 'z': 1.042, 'qx': -0.606, 'qy': 0.241, 'qz': 0.748, 'qw': 0.124},
            {'x': 0.227, 'y': 0.331, 'z': 1.183, 'qx': -0.704, 'qy': 0.068, 'qz': 0.704, 'qw': -0.068},
            {'x': -0.003, 'y': 0.341, 'z': 1.044, 'qx': 0.738, 'qy': 0.165, 'qz': -0.639, 'qw': 0.141},
            {'x': 0.227, 'y': 0.331, 'z': 1.183, 'qx': -0.704, 'qy': 0.068, 'qz': 0.704, 'qw': -0.068},
            {'x': 0.380, 'y': 0.298, 'z': 1.042, 'qx': -0.606, 'qy': 0.241, 'qz': 0.748, 'qw': 0.124},
            {'x': 0.223, 'y': 0.384, 'z': 1.026, 'qx': -0.693, 'qy': 0.141, 'qz': 0.693, 'qw': -0.141},
            {'x': 0.223, 'y': 0.129, 'z': 0.572, 'qx': 0.344, 'qy': -0.618, 'qz': -0.344, 'qw': 0.618}
        ]
        
        self.target_joints = ['회전-30', '회전-22', '회전-23', '회전-24', '회전-25', '회전-26']
        
        # 궤적 저장 변수
        self.current_target_index = 0
        self.total_time_offset = 0.0
        self.all_trajectory_points = []
        self.previous_end_joint_state = None 
        self.pending_target_joint_state = None

        self.timer = self.create_timer(1.0, self.start_planning)
        self.started_planning = False
        self.started_execution = False

    # ==========================================================
    # Phase 0: 하드웨어 원점 캡처
    # ==========================================================
    def capture_current_state_as_origin(self):
        target_ids = list(JOINT_NAME_TO_ID.values())
        max_ratio = max([GEAR_RATIOS[i] for i in target_ids])
        for dxl_id in target_ids:
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132) 
            if pos > 2147483647: pos -= 4294967296
            self.initial_motor_pulses[dxl_id] = pos
            vel = max(1, int(BASE_MAX_VELOCITY * (GEAR_RATIOS[dxl_id] / max_ratio)))
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, vel)

    # ==========================================================
    # Phase 1: MoveIt 궤적 계산
    # ==========================================================
    def start_planning(self):
        if self.started_planning: return
        self.started_planning = True
        self.get_logger().info('🧠 1단계: MoveIt을 통한 궤적 계산을 시작합니다...')
        self.process_next_target()

    def create_joint_constraints(self):
        constraints = Constraints()
        for joint_name, (min_deg, max_deg) in JOINT_LIMITS_DEG.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            min_rad = math.radians(min_deg)
            max_rad = math.radians(max_deg)
            mid = (max_rad + min_rad) / 2.0
            tol = (max_rad - min_rad) / 2.0
            jc.position = mid
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def process_next_target(self):
        if self.current_target_index >= len(self.targets):
            self.get_logger().info('✅ 궤적 계산 완료! 데이터를 저장하고 2초 뒤에 실제 실행을 시작합니다.')
            self.save_to_csv() # 백업용으로 CSV 저장
            
            # 계산된 첫 번째 각도를 초기 각도로 저장 (상대적 움직임 계산용)
            first_angles = self.all_trajectory_points[0][1]
            for name, rad in zip(self.target_joints, first_angles):
                self.initial_planned_angles[name] = rad
                
            # 2초 뒤에 Phase 2 실행
            self.create_timer(2.0, self.execute_realtime_trajectory)
            return

        pose_data = self.targets[self.current_target_index]
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = Duration(sec=5, nanosec=0)
        request.ik_request.constraints = self.create_joint_constraints()

        if self.previous_end_joint_state is None:
            request.ik_request.robot_state.is_diff = True
        else:
            request.ik_request.robot_state.joint_state = self.previous_end_joint_state

        pose = PoseStamped()
        pose.header.frame_id = 'base' 
        pose.pose.position.x = pose_data['x']
        pose.pose.position.y = pose_data['y']
        pose.pose.position.z = pose_data['z']
        pose.pose.orientation.x = pose_data['qx']
        pose.pose.orientation.y = pose_data['qy']
        pose.pose.orientation.z = pose_data['qz']
        pose.pose.orientation.w = pose_data['qw']

        request.ik_request.pose_stamped = pose
        self.get_logger().info(f'계산 중... [{self.current_target_index + 1}/{len(self.targets)}]')
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        response = future.result()
        if response.error_code.val == 1:
            self.pending_target_joint_state = response.solution.joint_state
            all_names = response.solution.joint_state.name
            all_positions = response.solution.joint_state.position
            target_positions = [all_positions[list(all_names).index(name)] for name in self.target_joints]
            self.request_trajectory(target_positions)
        else:
            self.get_logger().error(f'❌ IK 실패 (목표점: {self.current_target_index + 1})')
            rclpy.shutdown()

    def request_trajectory(self, target_positions):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'arm'
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.path_constraints = self.create_joint_constraints()
        
        if self.previous_end_joint_state is None:
            req.motion_plan_request.start_state.is_diff = True
        else:
            req.motion_plan_request.start_state.joint_state = self.previous_end_joint_state

        goal_constraint = Constraints()
        for name, pos in zip(self.target_joints, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraint.joint_constraints.append(jc)
        
        req.motion_plan_request.goal_constraints.append(goal_constraint)
        future = self.plan_client.call_async(req)
        future.add_done_callback(self.plan_callback)

    def plan_callback(self, future):
        response = future.result()
        if response.motion_plan_response.error_code.val == 1:
            trajectory_points = response.motion_plan_response.trajectory.joint_trajectory.points
            last_point_time = 0.0
            last_positions = None
            
            for point in trajectory_points:
                t = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
                last_point_time = t
                last_positions = list(point.positions)
                absolute_time = self.total_time_offset + t
                self.all_trajectory_points.append((absolute_time, last_positions))
            
            self.total_time_offset += last_point_time
            self.total_time_offset += 0.5
            self.all_trajectory_points.append((self.total_time_offset, last_positions))
            
            self.previous_end_joint_state = self.pending_target_joint_state
            self.current_target_index += 1
            self.process_next_target()
        else:
            self.get_logger().error(f'❌ 궤적 계획 실패')
            rclpy.shutdown()

    def save_to_csv(self):
        # 궤적이 잘 뽑혔는지 백업용으로 남겨둡니다.
        filename = 'waving_trajectory.csv'
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['경과시간(초)'] + self.target_joints)
            for absolute_time, positions in self.all_trajectory_points:
                row = [f"{absolute_time:.4f}"] + [f"{p:.6f}" for p in positions]
                writer.writerow(row)

    # ==========================================================
    # Phase 2: 실제 로봇 실행 및 RViz2 실시간 동기화
    # ==========================================================
    def execute_realtime_trajectory(self):
        if self.started_execution: return
        self.started_execution = True
        
        start_time = time.time()
        self.get_logger().info("🚀 2단계: 실제 로봇 제어 및 RViz2 동기화 시작!")

        for t_target, angles in self.all_trajectory_points:
            
            # 정확한 타임스탬프가 될 때까지 아주 짧게 대기
            while (time.time() - start_time) < t_target:
                time.sleep(0.001)

            # 1. 실제 다이나믹셀 모터 제어 데이터 전송
            for name, rad in zip(self.target_joints, angles):
                if name in JOINT_NAME_TO_ID:
                    dxl_id = JOINT_NAME_TO_ID[name]
                    delta_rad = rad - self.initial_planned_angles[name]
                    delta_deg = math.degrees(delta_rad)
                    pulse_change = int(delta_deg * GEAR_RATIOS[dxl_id] * (4096.0/360.0) * DIRECTION_MAP[dxl_id])
                    goal = (self.initial_motor_pulses[dxl_id] + pulse_change) & 0xFFFFFFFF
                    
                    param = [DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)), 
                             DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))]
                    self.groupSyncWrite.addParam(dxl_id, param)

            self.groupSyncWrite.txPacket()
            self.groupSyncWrite.clearParam()

            # 🌟 [추가됨] 2. RViz2 업데이트를 위한 JointState 발행
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.target_joints
            msg.position = angles  # MoveIt이 계산한 라디안 각도를 그대로 전달
            self.joint_pub.publish(msg)

        self.get_logger().info("🏁 모든 동작이 완료되었습니다. 로봇이 차렷 자세로 정지합니다.")
        self.create_timer(1.0, rclpy.shutdown)

def main(args=None):
    print("\n[알림] 원점 정렬 프로그램을 먼저 실행합니다. 정렬 후 q를 눌러주세요.")
    port_h, packet_h = calib.calibrate_origin()
    
    print("\n[알림] 정렬 완료! ROS 2 노드를 실행하여 궤적을 계산합니다.")
    rclpy.init(args=args)
    node = RealtimeWavingExecutor(port_h, packet_h)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if port_h.is_open:
            port_h.closePort()
        print("\n프로그램 종료.")

if __name__ == '__main__':
    main()