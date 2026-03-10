#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv

# RViz2 시각화를 위한 메시지 타입들
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class CSVTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('csv_trajectory_player')
        
        # RViz2의 /display_planned_path 토픽으로 데이터를 쏘는 퍼블리셔 생성
        self.publisher_ = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        
        # 스크립트를 켜자마자 바로 쏘면 RViz가 못 받을 수 있으니 2초 뒤에 발사!
        self.timer = self.create_timer(2.0, self.publish_trajectory)
        self.published = False

    def publish_trajectory(self):
        # 한 번만 쏘고 멈춤 (RViz 내에서 반복 재생 가능)
        if self.published:
            return
        self.published = True
        
        filename = 'waving_trajectory1.csv'
        self.get_logger().info(f'📁 [{filename}] 파일을 읽어옵니다...')
        
        display_msg = DisplayTrajectory()
        display_msg.model_id = 'arm'  # 로봇 모델 이름
        
        robot_traj = RobotTrajectory()
        joint_traj = JointTrajectory()
        
        try:
            with open(filename, mode='r') as f:
                reader = csv.reader(f)
                header = next(reader)
                
                # 첫 번째 열('경과시간(초)')을 제외한 나머지 이름을 모터 이름으로 등록
                joint_traj.joint_names = header[1:]
                
                for row in reader:
                    pt = JointTrajectoryPoint()
                    
                    # 1. 시간 파싱 (초와 나노초로 분리)
                    time_sec_float = float(row[0])
                    pt.time_from_start.sec = int(time_sec_float)
                    pt.time_from_start.nanosec = int((time_sec_float - int(time_sec_float)) * 1e9)
                    
                    # 2. 모터 각도 파싱
                    pt.positions = [float(x) for x in row[1:]]
                    
                    joint_traj.points.append(pt)
                    
        except FileNotFoundError:
            self.get_logger().error(f'❌ {filename} 파일이 같은 폴더에 없습니다!')
            rclpy.shutdown()
            return

        robot_traj.joint_trajectory = joint_traj
        display_msg.trajectory.append(robot_traj)

        self.get_logger().info('📺 RViz2로 21초짜리 궤적을 전송합니다!')
        self.publisher_.publish(display_msg)
        
        self.get_logger().info('✅ 전송 완료! RViz 화면을 확인하세요. (종료하려면 Ctrl+C)')

def main(args=None):
    rclpy.init(args=args)
    node = CSVTrajectoryPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()