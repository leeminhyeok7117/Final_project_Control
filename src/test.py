#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

class IKArmMoverAction(Node):
    def __init__(self):
        super().__init__('ik_arm_mover_action')
        
        # 1. IK 계산 서비스 클라이언트
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 2. 🌟 [핵심] 제어기에게 '액션'을 요청할 액션 클라이언트 생성!
        # (토픽 이름 뒤에 보통 follow_joint_trajectory가 붙습니다)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt IK 서비스를 기다리는 중...')

        self.timer = self.create_timer(2.0, self.send_ik_request)
        self.already_requested = False

    def send_ik_request(self):
        if self.already_requested:
            return
        self.already_requested = True

        # --- [목표 좌표 설정] ---
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'

        pose = PoseStamped()
        pose.header.frame_id = 'base' 
        
        pose.pose.position.x = 0.383
        pose.pose.position.y = 0.346
        pose.pose.position.z = 0.820
        
        pose.pose.orientation.x = 0.559
        pose.pose.orientation.y = -0.433
        pose.pose.orientation.z = -0.559
        pose.pose.orientation.w = 0.433

        request.ik_request.pose_stamped = pose
        # ------------------------

        self.get_logger().info(f'📍 IK 계산 요청 중... (목표 Z: {pose.pose.position.z})')
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        try:
            response = future.result()
            
            if response.error_code.val == 1:
                self.get_logger().info('✅ IK 계산 성공! 제어기에게 액션(Action) 명령을 보냅니다.')
                
                all_joint_names = response.solution.joint_state.name
                all_joint_positions = response.solution.joint_state.position
                
                target_joints = ['회전-30', '회전-22', '회전-23', '회전-24', '회전-25', '회전-26']
                target_positions = []
                
                for name in target_joints:
                    idx = list(all_joint_names).index(name)
                    target_positions.append(all_joint_positions[idx])

                # 🌟 [Action 목표(Goal) 메시지 만들기]
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory.joint_names = target_joints
                
                point = JointTrajectoryPoint()
                point.positions = target_positions
                # 깐깐한 제어기를 위해 속도(Velocity)를 0으로 명시적으로 넣어줍니다.
                point.velocities = [0.0] * len(target_joints) 
                point.time_from_start = Duration(sec=3, nanosec=0)
                
                goal_msg.trajectory.points.append(point)

                # 액션 서버가 켜져 있는지 확인하고 전송
                self.action_client.wait_for_server()
                self.get_logger().info('📮 제어기에 등기 우편(Action Goal) 발송 중...')
                
                # 비동기로 목표 전송 후, 수락 여부 콜백 지정
                self.send_goal_future = self.action_client.send_goal_async(goal_msg)
                self.send_goal_future.add_done_callback(self.goal_response_callback)
                
            else:
                self.get_logger().error(f'❌ 도달할 수 없는 위치입니다! (에러 코드: {response.error_code.val})')

        except Exception as e:
            self.get_logger().error(f'IK 서비스 호출 실패: {e}')

    # 🌟 [Action 콜백 1] 제어기가 내 명령을 수락했는지 확인
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('🚫 제어기가 명령을 거절했습니다! (위험한 각도이거나 설정 오류)')
            return

        self.get_logger().info('👌 제어기가 명령을 수락했습니다! 로봇이 움직입니다...')
        
        # 명령이 수락되었다면, 최종 결과를 기다림
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    # 🌟 [Action 콜백 2] 로봇이 도착한 뒤 최종 결과 보고받기
    def get_result_callback(self, future):
        result = future.result().result
        
        # error_code가 0이면 완벽한 성공 (FollowJointTrajectory 기준)
        if result.error_code == 0:
            self.get_logger().info('🎉 로봇 이동 완벽하게 완료!!!')
        else:
            self.get_logger().error(f'⚠️ 이동 중 문제 발생 (제어기 에러 코드: {result.error_code})')

def main(args=None):
    rclpy.init(args=args)
    node = IKArmMoverAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()