#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class IKArmMover(Node):
    def __init__(self):
        super().__init__('ik_arm_mover')
        
        # 1. IK 계산을 요청할 '서비스 클라이언트' 생성
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 2. 계산된 각도를 로봇 근육에 쏠 '퍼블리셔' 생성
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # 서비스가 준비될 때까지 대기
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MoveIt IK 서비스를 기다리는 중...')

        # 노드 실행 2초 뒤에 딱 한 번 IK 요청 실행
        self.timer = self.create_timer(2.0, self.send_ik_request)
        self.already_requested = False

    def send_ik_request(self):
        if self.already_requested:
            return
        self.already_requested = True

        # --- [여기서 목표 좌표를 설정합니다] ---
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'  # Setup Assistant에서 설정한 팔 그룹 이름

        pose = PoseStamped()
        # ⚠️ 중요: 로봇의 맨 밑바닥 중심이 되는 링크 이름을 적어야 합니다.
        # (보통 'base_link', 'base_footprint' 또는 'world' 입니다.)
        pose.header.frame_id = 'base_link' 
        
        # 목표 좌표 (단위: 미터)
        pose.pose.position.x = 0.7
        pose.pose.position.y = 0.7
        pose.pose.position.z = 0.7
        
        # 목표 방향 (쿼터니언 회전값. 일단 기본 정면 방향으로 설정)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        request.ik_request.pose_stamped = pose
        # --------------------------------------

        self.get_logger().info(f'📍 목표 좌표 (X:{pose.pose.position.x}, Y:{pose.pose.position.y}, Z:{pose.pose.position.z})로 IK 계산 요청 중...')
        
        # MoveIt에게 계산 요청! (비동기 방식)
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        try:
            response = future.result()
            
            # error_code.val == 1 이면 계산 성공을 의미합니다.
            if response.error_code.val == 1:
                self.get_logger().info('✅ IK 계산 성공! 해당 위치로 로봇을 이동시킵니다.')
                
                # MoveIt이 알려준 전체 관절 상태 뽑아오기
                all_joint_names = response.solution.joint_state.name
                all_joint_positions = response.solution.joint_state.position
                
                # 우리가 제어할 관절 이름들만 쏙 빼내기
                target_joints = ['회전-30', '회전-22', '회전-23', '회전-24', '회전-25', '회전-26']
                target_positions = []
                
                for name in target_joints:
                    idx = list(all_joint_names).index(name)
                    target_positions.append(all_joint_positions[idx])

                # 퍼블리시할 궤적 메시지 만들기
                msg = JointTrajectory()
                msg.joint_names = target_joints
                
                point = JointTrajectoryPoint()
                point.positions = target_positions
                point.time_from_start = Duration(sec=3, nanosec=0) # 3초에 걸쳐 이동
                
                msg.points.append(point)
                self.publisher_.publish(msg)
                
            else:
                # 닿을 수 없거나 관절이 꺾일 수 없는 위치일 경우 에러 발생
                self.get_logger().error(f'❌ 도달할 수 없는 위치입니다! (에러 코드: {response.error_code.val})')
                self.get_logger().error('x, y, z 좌표 값을 로봇이 닿을 수 있는 거리로 수정해 보세요.')

        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 문제 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IKArmMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()