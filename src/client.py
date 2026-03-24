#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time # sleep을 위해 추가

from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_LIMITS_DEG = {
    '회전-30': (-120.0, 120.0), '회전-22': (-30.0, 90.0), '회전-23': (-90.0, 90.0),
    '회전-24': (-115.0, 10.0),  '회전-25': (-90.0, 90.0), '회전-26': (-180.0, 180.0), '회전-28': (-180.0, 180.0)
}

class WavingActionClient(Node):
    def __init__(self):
        super().__init__('waving_action_client')
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        # 🌟 새롭게 따온 13개 스텝 + 그리퍼 상태 완벽 적용
        self.targets = [
            {'x': 0.228, 'y': -0.104, 'z': 0.536, 'qx': 0.263, 'qy': -0.656, 'qz': -0.263, 'qw': 0.656, 'gripper': -0.5},    # 1. 앙다물기
            {'x': 0.228, 'y': -0.067, 'z': 0.611, 'qx': 0.371, 'qy': -0.602, 'qz': -0.371, 'qw': 0.602, 'gripper': 0.01},    # 2. 앙다물기
            {'x': 0.228, 'y': 0.003, 'z': 0.621, 'qx': 0.478, 'qy': -0.521, 'qz': -0.478, 'qw': 0.521, 'gripper': 0.01},    # 3. 앙다물기
            {'x': 0.228, 'y': 0.178, 'z': 0.608, 'qx': 0.495, 'qy': -0.505, 'qz': -0.495, 'qw': 0.505, 'gripper': 0.01},    # 4. 앙다물기
            {'x': 0.228, 'y': 0.178, 'z': 0.608, 'qx': 0.495, 'qy': -0.505, 'qz': -0.495, 'qw': 0.505, 'gripper': -0.35},   # 5. 벌리기 (제자리)
            {'x': 0.228, 'y': 0.281, 'z': 0.614, 'qx': 0.500, 'qy': -0.500, 'qz': -0.500, 'qw': 0.500, 'gripper': -0.35},   # 6. 벌리기
            {'x': 0.228, 'y': 0.281, 'z': 0.614, 'qx': 0.500, 'qy': -0.500, 'qz': -0.500, 'qw': 0.500, 'gripper': -0.033},  # 7. 잡은상태 (제자리)
            {'x': 0.173, 'y': 0.258, 'z': 0.689, 'qx': 0.584, 'qy': -0.399, 'qz': -0.399, 'qw': 0.584, 'gripper': -0.033},  # 8. 잡은상태
            {'x': 0.151, 'y': 0.288, 'z': 0.633, 'qx': 0.584, 'qy': -0.399, 'qz': -0.399, 'qw': 0.584, 'gripper': -0.333},   # 9. 벌리기
            {'x': 0.151, 'y': 0.288, 'z': 0.633, 'qx': 0.584, 'qy': -0.399, 'qz': -0.399, 'qw': 0.584, 'gripper': -0.35},    # 10. 앙다물기 (제자리)
            {'x': 0.254, 'y': 0.158, 'z': 0.633, 'qx': 0.546, 'qy': -0.449, 'qz': -0.448, 'qw': 0.547, 'gripper': 0.01},    # 11. 앙다물기
            {'x': 0.264, 'y': 0.088, 'z': 0.617, 'qx': 0.533, 'qy': -0.464, 'qz': -0.464, 'qw': 0.534, 'gripper': 0.01},    # 12. 앙다물기
            {'x': 0.228, 'y': 0.001, 'z': 0.426, 'qx': 0.000, 'qy': -0.707, 'qz': -0.000, 'qw': 0.707, 'gripper': 0.01},    # 13. 앙다물기
        ]
        
        self.target_joints = ['회전-30', '회전-22', '회전-23', '회전-24', '회전-25', '회전-26','회전-28']
        self.current_target_index = 0
        self.total_time_offset = 0.0
        self.all_trajectory_points = []
        self.previous_end_joint_state = None 
        self.pending_target_joint_state = None

        self.get_logger().info('⏳ MoveIt 서비스가 켜질 때까지 기다립니다...')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('MoveIt /compute_ik 서비스를 찾을 수 없습니다. MoveIt이 켜져 있나요?')
        while not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('MoveIt /plan_kinematic_path 서비스를 찾을 수 없습니다.')
            
        self.get_logger().info('✅ MoveIt 서비스 연결 성공!')
        
        # 🌟 무한 타이머 제거! 직접 함수 호출
        self.start_planning()

    def start_planning(self):
        self.get_logger().info('🧠 1단계: MoveIt을 통한 손인사 궤적 계산 시작...')
        self.process_next_target()

    def create_joint_constraints(self):
        constraints = Constraints()
        for joint_name, (min_deg, max_deg) in JOINT_LIMITS_DEG.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            min_rad = math.radians(min_deg)
            max_rad = math.radians(max_deg)
            jc.position = (max_rad + min_rad) / 2.0
            jc.tolerance_above = (max_rad - min_rad) / 2.0
            jc.tolerance_below = (max_rad - min_rad) / 2.0
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def process_next_target(self):
        if self.current_target_index >= len(self.targets):
            self.get_logger().info('✅ 모든 궤적 계산 완료! 바로 액션 서버로 명령을 전송합니다.')
            # 🌟 무한 반복되는 타이머 삭제! 딱 한 번만 실행되도록 직접 호출!
            self.send_action_goal()
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
            self.get_logger().error(f'❌ IK 실패 (에러코드: {response.error_code.val})')
            rclpy.shutdown()

    def request_trajectory(self, target_positions):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'arm'
        req.motion_plan_request.num_planning_attempts = 10
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.path_constraints = self.create_joint_constraints()
        
        req.motion_plan_request.max_velocity_scaling_factor = 1.0
        req.motion_plan_request.max_acceleration_scaling_factor = 1.0
        
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
            plan_joint_names = response.motion_plan_response.trajectory.joint_trajectory.joint_names
            trajectory_points = response.motion_plan_response.trajectory.joint_trajectory.points
            
            last_point_time = 0.0
            
            for point in trajectory_points:
                t = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
                last_point_time = t
                absolute_time = self.total_time_offset + t
                
                ordered_positions = []
                current_gripper_val = self.targets[self.current_target_index]['gripper']
                for name in self.target_joints: 
                    if name in plan_joint_names:
                        # 팔 관절(6개)은 MoveIt이 계산해준 궤적에서 가져옵니다.
                        idx = list(plan_joint_names).index(name)
                        ordered_positions.append(point.positions[idx])
                    else:
                        # MoveIt 결과에 없는 관절('회전-28')은 우리가 적어둔 값을 그대로 씁니다!
                        ordered_positions.append(current_gripper_val)
                
                self.all_trajectory_points.append((absolute_time, ordered_positions))
            
            self.total_time_offset += last_point_time
            self.total_time_offset += 0.5
            
            self.previous_end_joint_state = self.pending_target_joint_state
            self.current_target_index += 1
            self.process_next_target()
        else:
            self.get_logger().error('❌ 궤적 계획 실패')
            rclpy.shutdown()

    def send_action_goal(self):
        self.get_logger().info('🚀 2단계: 궤적을 액션 서버로 던집니다!')
        
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ 액션 서버를 찾을 수 없습니다. action.py가 켜져 있나요?')
            rclpy.shutdown()
            return

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.target_joints

        for t_target, angles in self.all_trajectory_points:
            point = JointTrajectoryPoint()
            point.positions = angles
            sec = int(t_target)
            nanosec = int((t_target - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points.append(point)

        goal_msg.trajectory = trajectory

        self.get_logger().info('명령 전송 완료! 로봇 이동 대기 중...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 액션 서버가 명령을 거절했습니다.')
            rclpy.shutdown()
            return

        self.get_logger().info('✅ 액션 서버가 명령을 수락했습니다! 열심히 움직이는 중...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🏁 로봇으로부터 [손인사 완료] 보고를 받았습니다! 프로그램을 종료합니다.')
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WavingActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[액션 클라이언트] 종료되었습니다.")

if __name__ == '__main__':
    main()