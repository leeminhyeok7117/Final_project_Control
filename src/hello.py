#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import csv
# import math

# # 🌟 [추가됨] 시간 설정을 위해 추가 (IK 타임아웃용)
# from builtin_interfaces.msg import Duration

# # 기존 서비스
# from moveit_msgs.srv import GetPositionIK
# from geometry_msgs.msg import PoseStamped

# # 모션 플랜 서비스
# from moveit_msgs.srv import GetMotionPlan
# from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

# class WavingTrajectoryExtractor(Node):
#     def __init__(self):
#         super().__init__('waving_trajectory_extractor')
        
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
#         self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
#         while not self.plan_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('MoveIt Planning 서비스를 기다리는 중...')

#         # 🌟 10개의 목표점 (손인사 웨이브 + 차렷 복귀 완벽 루프)
#         self.targets = [
#             {'x': 0.223, 'y': 0.129, 'z': 0.572, 'qx': 0.344, 'qy': -0.618, 'qz': -0.344, 'qw': 0.618}, # 1번: 시작점
#             {'x': 0.223, 'y': 0.384, 'z': 1.026, 'qx': -0.693, 'qy': 0.141, 'qz': 0.693, 'qw': -0.141}, # 2번: 들어올리기
#             {'x': 0.227, 'y': 0.331, 'z': 1.183, 'qx': -0.704, 'qy': 0.068, 'qz': 0.704, 'qw': -0.068}, # 3번: 가운데
#             {'x': 0.380, 'y': 0.298, 'z': 1.042, 'qx': -0.606, 'qy': 0.241, 'qz': 0.748, 'qw': 0.124}, # 4번: 오른쪽
#             {'x': 0.227, 'y': 0.331, 'z': 1.183, 'qx': -0.704, 'qy': 0.068, 'qz': 0.704, 'qw': -0.068}, # 5번: 가운데 
#             {'x': -0.003, 'y': 0.341, 'z': 1.044, 'qx': 0.738, 'qy': 0.165, 'qz': -0.639, 'qw': 0.141}, # 6번: 왼쪽
#             {'x': 0.227, 'y': 0.331, 'z': 1.183, 'qx': -0.704, 'qy': 0.068, 'qz': 0.704, 'qw': -0.068}, # 7번: 가운데
#             {'x': 0.380, 'y': 0.298, 'z': 1.042, 'qx': -0.606, 'qy': 0.241, 'qz': 0.748, 'qw': 0.124}, # 8번: 오른쪽
#             {'x': 0.223, 'y': 0.384, 'z': 1.026, 'qx': -0.693, 'qy': 0.141, 'qz': 0.693, 'qw': -0.141}, # 9번: 들어올렸던 팔 내리기 (2번 복사)
#             {'x': 0.223, 'y': 0.129, 'z': 0.572, 'qx': 0.344, 'qy': -0.618, 'qz': -0.344, 'qw': 0.618}  # 10번: 완전한 차렷 (1번 복사)
#         ]
        
#         self.target_joints = ['회전-30', '회전-22', '회전-23', '회전-24', '회전-25', '회전-26']
        
#         # 궤적을 이어붙이기 위한 변수들
#         self.current_target_index = 0
#         self.total_time_offset = 0.0
#         self.all_trajectory_points = []
        
#         self.previous_end_joint_state = None 
#         self.pending_target_joint_state = None

#         self.timer = self.create_timer(1.0, self.start_process)
#         self.started = False

#     def start_process(self):
#         if self.started: return
#         self.started = True
#         self.get_logger().info('👋 손인사(Waving) 다중 궤적 추출을 시작합니다!')
#         self.process_next_target()

#     # --- 1단계: N번째 목표점에 대한 정답 각도 구하기 ---
#     def process_next_target(self):
#         if self.current_target_index >= len(self.targets):
#             self.save_to_csv()
#             return

#         pose_data = self.targets[self.current_target_index]
        
#         request = GetPositionIK.Request()
#         request.ik_request.group_name = 'arm'

#         # 🌟 [추가됨] IK 계산기에게 "5초 동안 끈질기게 찾아봐!" 라고 명령 (Timeout 늘리기)
#         request.ik_request.timeout = Duration(sec=5, nanosec=0)

#         # 핵심: 첫 번째는 현재 로봇 자세에서 출발, 두 번째부터는 이전 도착 자세에서 출발
#         if self.previous_end_joint_state is None:
#             request.ik_request.robot_state.is_diff = True
#         else:
#             request.ik_request.robot_state.joint_state = self.previous_end_joint_state

#         pose = PoseStamped()
#         pose.header.frame_id = 'base' 
#         pose.pose.position.x = pose_data['x']
#         pose.pose.position.y = pose_data['y']
#         pose.pose.position.z = pose_data['z']
#         pose.pose.orientation.x = pose_data['qx']
#         pose.pose.orientation.y = pose_data['qy']
#         pose.pose.orientation.z = pose_data['qz']
#         pose.pose.orientation.w = pose_data['qw']

#         request.ik_request.pose_stamped = pose
        
#         self.get_logger().info(f'[{self.current_target_index + 1}/{len(self.targets)}] 번째 목표점 IK 계산 중...')
#         future = self.ik_client.call_async(request)
#         future.add_done_callback(self.ik_callback)

#     # --- 2단계: 정답 각도를 바탕으로 '경로 쪼개기' 요청 ---
#     def ik_callback(self, future):
#         response = future.result()
#         if response.error_code.val == 1:
#             self.pending_target_joint_state = response.solution.joint_state
            
#             all_names = response.solution.joint_state.name
#             all_positions = response.solution.joint_state.position
#             target_positions = [all_positions[list(all_names).index(name)] for name in self.target_joints]

#             solved_deg = [round(math.degrees(p), 1) for p in target_positions]
#             self.get_logger().info(f'🔎 찾아낸 정답 각도: {solved_deg}')

#             self.request_trajectory(target_positions)
#         else:
#             self.get_logger().error(f'❌ [{self.current_target_index + 1}] 번째 목표점 IK 실패! (에러코드: {response.error_code.val})')
#             # 에러 발생 시 강제 종료
#             rclpy.shutdown()

#     # --- 3단계: 플래너에게 이전 목표점 -> 현재 목표점 궤적 요구 ---
#     def request_trajectory(self, target_positions):
#         req = GetMotionPlan.Request()
#         req.motion_plan_request.group_name = 'arm'
        
#         # 🌟 [추가됨] 플래너 설정: 10번 재시도, 5초 허용 (랜덤성 극복 및 안정성 강화)
#         req.motion_plan_request.num_planning_attempts = 10
#         req.motion_plan_request.allowed_planning_time = 5.0
        
#         if self.previous_end_joint_state is None:
#             req.motion_plan_request.start_state.is_diff = True
#         else:
#             req.motion_plan_request.start_state.joint_state = self.previous_end_joint_state

#         constraint = Constraints()
#         for name, pos in zip(self.target_joints, target_positions):
#             jc = JointConstraint()
#             jc.joint_name = name
#             jc.position = pos
#             # 🌟 [수정됨] 오차 범위를 0.001 -> 0.01로 10배 넓혀서 관절 꺾임 압박 완화
#             jc.tolerance_above = 0.01
#             jc.tolerance_below = 0.01
#             jc.weight = 1.0
#             constraint.joint_constraints.append(jc)
        
#         req.motion_plan_request.goal_constraints.append(constraint)

#         future = self.plan_client.call_async(req)
#         future.add_done_callback(self.plan_callback)

#     # --- 4단계: 궤적 누적 및 0.5초 정지 추가 ---
#     def plan_callback(self, future):
#         response = future.result()
        
#         if response.motion_plan_response.error_code.val == 1:
#             trajectory_points = response.motion_plan_response.trajectory.joint_trajectory.points
            
#             last_point_time = 0.0
#             last_positions = None
            
#             for point in trajectory_points:
#                 t = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
#                 last_point_time = t
#                 last_positions = list(point.positions)
                
#                 absolute_time = self.total_time_offset + t
#                 self.all_trajectory_points.append((absolute_time, last_positions))
            
#             self.total_time_offset += last_point_time
            
#             # 🌟 도착 후 0.5초 대기 (이전 각도 그대로 유지)
#             self.total_time_offset += 0.5
#             self.all_trajectory_points.append((self.total_time_offset, last_positions))
            
#             self.get_logger().info(f'✅ [{self.current_target_index + 1}] 번째 구간 완료 (누적시간: {self.total_time_offset:.2f}초)')

#             # 이전 도착 자세를 저장하여 다음 플래닝의 씨앗(Seed)으로 사용!
#             self.previous_end_joint_state = self.pending_target_joint_state

#             self.current_target_index += 1
#             self.process_next_target()
            
#         else:
#             self.get_logger().error(f'❌ 궤적 계획 실패 (에러 코드: {response.motion_plan_response.error_code.val})')
#             rclpy.shutdown()

#     # --- 5단계: 완성된 전체 궤적 CSV 저장 ---
#     def save_to_csv(self):
#         filename = 'waving_trajectory.csv'
#         with open(filename, mode='w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(['경과시간(초)'] + self.target_joints)
            
#             for absolute_time, positions in self.all_trajectory_points:
#                 row = [f"{absolute_time:.4f}"] + [f"{p:.6f}" for p in positions]
#                 writer.writerow(row)
                
#         self.get_logger().info(f'🎉 축하합니다! 손인사 궤적이 [{filename}]에 완벽하게 저장되었습니다!')
#         self.get_logger().info(f'총 {len(self.all_trajectory_points)}개의 스텝, 소요 시간: {self.total_time_offset:.2f}초')
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = WavingTrajectoryExtractor()
#     rclpy.spin(node)

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import csv
import math

from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

# 🌟 [수정됨] 직관적인 도(Degree) 단위로 제한 범위 설정!
# (최소 각도, 최대 각도)
JOINT_LIMITS_DEG = {
    '회전-30': (-120.0, 120.0),    # 1번 축(허리): 좌우 90도
    '회전-22': (-30.0, 90.0),    # 2번 축(어깨): 뒤로 30도, 앞으로 90도
    '회전-23': (-90.0, 90.0),    # 3번 축: +- 90도
    '회전-24': (-115.0, 10.0),  # 4번 축: +- 115도
    '회전-25': (-90.0, 90.0),    # 5번 축: +- 90도
    '회전-26': (-180.0, 180.0)   # 6번 축(손목): 자유롭게 1바퀴
}

class WavingTrajectoryExtractor(Node):
    def __init__(self):
        super().__init__('waving_trajectory_extractor')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt Planning 서비스를 기다리는 중...')

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
        
        self.current_target_index = 0
        self.total_time_offset = 0.0
        self.all_trajectory_points = []
        
        self.previous_end_joint_state = None 
        self.pending_target_joint_state = None

        self.timer = self.create_timer(1.0, self.start_process)
        self.started = False

    def start_process(self):
        if self.started: return
        self.started = True
        self.get_logger().info('👋 각도(Degree) 제한이 적용된 궤적 추출을 시작합니다!')
        self.process_next_target()

    # --- 🌟 [핵심 변경] Degree를 Radian으로 자동 변환 ---
    def create_joint_constraints(self):
        constraints = Constraints()
        for joint_name, (min_deg, max_deg) in JOINT_LIMITS_DEG.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            
            # 입력받은 각도를 MoveIt이 쓰는 라디안으로 변환
            min_rad = math.radians(min_deg)
            max_rad = math.radians(max_deg)
            
            # 변환된 라디안으로 중간값과 허용 오차 계산
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
            self.save_to_csv()
            return

        pose_data = self.targets[self.current_target_index]
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = Duration(sec=5, nanosec=0)

        # 제약 조건 삽입
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
        
        self.get_logger().info(f'[{self.current_target_index + 1}/{len(self.targets)}] 번째 목표점 IK 계산 중...')
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
            self.get_logger().error(f'❌ [{self.current_target_index + 1}] 번째 목표점 IK 실패! (자세를 약간 수정하거나 제한 각도를 넓혀주세요)')
            rclpy.shutdown()

    def request_trajectory(self, target_positions):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'arm'
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0
        
        # 경로 이동 중 꺾임 방지
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
            
            self.get_logger().info(f'✅ [{self.current_target_index + 1}] 번째 구간 완료 (누적시간: {self.total_time_offset:.2f}초)')

            self.previous_end_joint_state = self.pending_target_joint_state
            self.current_target_index += 1
            self.process_next_target()
            
        else:
            self.get_logger().error(f'❌ 궤적 계획 실패 (에러 코드: {response.motion_plan_response.error_code.val})')
            rclpy.shutdown()

    def save_to_csv(self):
        filename = 'waving_trajectory.csv'
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['경과시간(초)'] + self.target_joints)
            
            for absolute_time, positions in self.all_trajectory_points:
                row = [f"{absolute_time:.4f}"] + [f"{p:.6f}" for p in positions]
                writer.writerow(row)
                
        self.get_logger().info(f'🎉 축하합니다! 손인사 궤적이 [{filename}]에 완벽하게 저장되었습니다!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WavingTrajectoryExtractor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()