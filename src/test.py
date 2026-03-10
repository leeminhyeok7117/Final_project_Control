#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from moveit_msgs.srv import GetPositionIK
# from geometry_msgs.msg import PoseStamped
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory

# class IKArmMoverAction(Node):
#     def __init__(self):
#         super().__init__('ik_arm_mover_action')
        
#         # 1. IK 계산 서비스 클라이언트
#         self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
#         # 2. 🌟 [핵심] 제어기에게 '액션'을 요청할 액션 클라이언트 생성!
#         # (토픽 이름 뒤에 보통 follow_joint_trajectory가 붙습니다)
#         self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
#         while not self.ik_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('MoveIt IK 서비스를 기다리는 중...')

#         self.timer = self.create_timer(2.0, self.send_ik_request)
#         self.already_requested = False

#     def send_ik_request(self):
#         if self.already_requested:
#             return
#         self.already_requested = True

#         # --- [목표 좌표 설정] ---
#         request = GetPositionIK.Request()
#         request.ik_request.group_name = 'arm'

#         pose = PoseStamped()
#         pose.header.frame_id = 'base' 
        
#         pose.pose.position.x = 0.383
#         pose.pose.position.y = 0.346
#         pose.pose.position.z = 0.820
        
#         pose.pose.orientation.x = 0.559
#         pose.pose.orientation.y = -0.433
#         pose.pose.orientation.z = -0.559
#         pose.pose.orientation.w = 0.433

#         request.ik_request.pose_stamped = pose
#         # ------------------------

#         self.get_logger().info(f'📍 IK 계산 요청 중... (목표 Z: {pose.pose.position.z})')
#         future = self.ik_client.call_async(request)
#         future.add_done_callback(self.ik_callback)

#     def ik_callback(self, future):
#         try:
#             response = future.result()
            
#             if response.error_code.val == 1:
#                 self.get_logger().info('✅ IK 계산 성공! 제어기에게 액션(Action) 명령을 보냅니다.')
                
#                 all_joint_names = response.solution.joint_state.name
#                 all_joint_positions = response.solution.joint_state.position
                
#                 target_joints = ['회전-30', '회전-22', '회전-23', '회전-24', '회전-25', '회전-26']
#                 target_positions = []
                
#                 for name in target_joints:
#                     idx = list(all_joint_names).index(name)
#                     target_positions.append(all_joint_positions[idx])

#                 # 🌟 [Action 목표(Goal) 메시지 만들기]
#                 goal_msg = FollowJointTrajectory.Goal()
#                 goal_msg.trajectory.joint_names = target_joints
                
#                 point = JointTrajectoryPoint()
#                 point.positions = target_positions
#                 # 깐깐한 제어기를 위해 속도(Velocity)를 0으로 명시적으로 넣어줍니다.
#                 point.velocities = [0.0] * len(target_joints) 
#                 point.time_from_start = Duration(sec=3, nanosec=0)
                
#                 goal_msg.trajectory.points.append(point)

#                 # 액션 서버가 켜져 있는지 확인하고 전송
#                 self.action_client.wait_for_server()
#                 self.get_logger().info('📮 제어기에 등기 우편(Action Goal) 발송 중...')
                
#                 # 비동기로 목표 전송 후, 수락 여부 콜백 지정
#                 self.send_goal_future = self.action_client.send_goal_async(goal_msg)
#                 self.send_goal_future.add_done_callback(self.goal_response_callback)
                
#             else:
#                 self.get_logger().error(f'❌ 도달할 수 없는 위치입니다! (에러 코드: {response.error_code.val})')

#         except Exception as e:
#             self.get_logger().error(f'IK 서비스 호출 실패: {e}')

#     # 🌟 [Action 콜백 1] 제어기가 내 명령을 수락했는지 확인
#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('🚫 제어기가 명령을 거절했습니다! (위험한 각도이거나 설정 오류)')
#             return

#         self.get_logger().info('👌 제어기가 명령을 수락했습니다! 로봇이 움직입니다...')
        
#         # 명령이 수락되었다면, 최종 결과를 기다림
#         self.result_future = goal_handle.get_result_async()
#         self.result_future.add_done_callback(self.get_result_callback)

#     # 🌟 [Action 콜백 2] 로봇이 도착한 뒤 최종 결과 보고받기
#     def get_result_callback(self, future):
#         result = future.result().result
        
#         # error_code가 0이면 완벽한 성공 (FollowJointTrajectory 기준)
#         if result.error_code == 0:
#             self.get_logger().info('🎉 로봇 이동 완벽하게 완료!!!')
#         else:
#             self.get_logger().error(f'⚠️ 이동 중 문제 발생 (제어기 에러 코드: {result.error_code})')

# def main(args=None):
#     rclpy.init(args=args)
#     node = IKArmMoverAction()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
import csv
import math

# 기존 IK 서비스
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

# 🌟 [새로 추가된 모듈] 모션 플랜(경로 계획) 서비스 관련
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

class TrajectoryExtractor(Node):
    def __init__(self):
        super().__init__('trajectory_extractor')
        
        # 1. IK 계산 서비스 (최종 각도 찾기용)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # 2. 🌟 경로 계획 서비스 (시간별 쪼개기용)
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt Planning 서비스를 기다리는 중...')

        self.timer = self.create_timer(1.0, self.get_target_angles)
        self.already_requested = False

    # --- 1단계: 목표 XYZ에 대한 최종 정답 각도 구하기 ---
    def get_target_angles(self):
        if self.already_requested:
            return
        self.already_requested = True

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.robot_state.is_diff = True # 현재 자세 기준!

        pose = PoseStamped()
        pose.header.frame_id = 'base' 
        pose.pose.position.x = 0.338
        pose.pose.position.y = 0.209
        pose.pose.position.z = 0.512
        
        pose.pose.orientation.x = 0.376
        pose.pose.orientation.y = -0.599
        pose.pose.orientation.z = -0.376
        pose.pose.orientation.w = 0.599

        request.ik_request.pose_stamped = pose
        
        self.get_logger().info('1️⃣ 목표 지점의 최종 모터 각도 계산 중...')
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    # --- 2단계: 구해진 정답 각도를 바탕으로 '경로 쪼개기' 요청 ---
    def ik_callback(self, future):
        response = future.result()
        if response.error_code.val == 1:
            all_names = response.solution.joint_state.name
            all_positions = response.solution.joint_state.position
            
            target_joints = ['회전-30', '회전-22', '회전-23', '회전-24', '회전-25', '회전-26']
            target_positions = [all_positions[list(all_names).index(name)] for name in target_joints]
            
            self.get_logger().info('✅ 최종 각도 확보 완료! 이제 시간 단위로 경로를 쪼갭니다.')
            self.request_trajectory(target_joints, target_positions)
        else:
            self.get_logger().error('❌ IK 계산 실패! 위치를 조금 변경해보세요.')

    # --- 3단계: MoveIt 플래너에게 시간별 시간표(Trajectory) 요구 ---
    def request_trajectory(self, joint_names, target_positions):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'arm'
        req.motion_plan_request.start_state.is_diff = True # 현재 자세에서 출발

        # 목표 각도를 '제약조건(Constraint)' 형태로 플래너에게 전달
        constraint = Constraints()
        for name, pos in zip(joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)
        
        req.motion_plan_request.goal_constraints.append(constraint)

        future = self.plan_client.call_async(req)
        future.add_done_callback(self.plan_callback)

    # --- 4단계: 받아온 시간별 궤적 데이터 추출 및 저장 ---
    def plan_callback(self, future):
        response = future.result()
        
        if response.motion_plan_response.error_code.val == 1:
            # 🌟 여기가 핵심입니다! MoveIt이 계산한 수백 개의 시간표(points)
            trajectory_points = response.motion_plan_response.trajectory.joint_trajectory.points
            
            self.get_logger().info(f'🎉 총 {len(trajectory_points)}개의 스텝으로 쪼개진 궤적을 얻었습니다!')
            
            # CSV 파일로 저장
            with open('time_step_trajectory.csv', mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['경과시간(초)'] + response.motion_plan_response.trajectory.joint_trajectory.joint_names)
                
                for point in trajectory_points:
                    # 각 스텝별 도달해야 할 시간 (초 + 나노초 환산)
                    time_sec = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
                    
                    # 각 스텝별 모터의 요구 각도 (도 단위로 변환해서 보기 편하게)
                    positions_deg = [math.degrees(p) for p in point.positions]
                    
                    # 터미널에 살짝 보여주기
                    self.get_logger().info(f'[시간: {time_sec:.3f}초] 각도: {[f"{deg:.1f}°" for deg in positions_deg]}')
                    
                    # CSV에는 라디안(또는 필요에 따라 Degree)으로 저장
                    writer.writerow([f"{time_sec:.4f}"] + list(point.positions))
                    
            self.get_logger().info('💾 [time_step_trajectory.csv] 파일에 모든 시간별 스텝 데이터 저장 완료!')
            
        else:
            self.get_logger().error(f'❌ 궤적 계획 실패 (에러 코드: {response.motion_plan_response.error_code.val})')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExtractor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()