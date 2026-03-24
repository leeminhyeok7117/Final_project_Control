#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import sys, select, termios, tty
from dynamixel_sdk import *
import subprocess

import calibrate_origin_keyboard as calib

# 🌟 7번 모터(그리퍼) 정식 추가!
JOINT_NAME_TO_ID = {
    '회전-30': 1, '회전-22': 2, '회전-23': 3,
    '회전-24': 4, '회전-25': 5, '회전-26': 6,
    '회전-28': 7  
}
GEAR_RATIOS = {1: 25, 2: 25, 3: 1, 4: 15, 5: 1, 6: 1, 7: 1}
DIRECTION_MAP = {1: 1, 2: 1, 3: -1, 4: 1, 5: -1, 6: -1, 7: 1}

settings = termios.tcgetattr(sys.stdin)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeachingNode(Node):
    def __init__(self, port_handler, packet_handler):
        super().__init__('teaching_node')
        self.get_logger().info('방해꾼(ros2_control)을 잠재우는 중...')
        subprocess.run(['ros2', 'control', 'set_controller_state', 'joint_state_broadcaster', 'inactive'], capture_output=True)
        subprocess.run(['ros2', 'control', 'set_controller_state', 'joint_trajectory_controller', 'inactive'], capture_output=True)
        
        self.base_frame = 'base'        
        self.target_frame = 'G_28'     
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.portHandler = port_handler
        self.packetHandler = packet_handler
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 116, 4)
        self.initial_motor_pulses = {}
        self.target_joints = list(JOINT_NAME_TO_ID.keys())
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.current_angles = [0.0] * len(self.target_joints)
        
        self.capture_current_state_as_origin()
        
    def capture_current_state_as_origin(self):
        for dxl_id in JOINT_NAME_TO_ID.values():
            pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, 132) 
            if pos > 2147483647: pos -= 4294967296
            self.initial_motor_pulses[dxl_id] = pos
            speed = 0 if dxl_id in [1, 2, 4] else 50
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, 112, speed) 

    def publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.target_joints
        msg.position = self.current_angles
        self.joint_pub.publish(msg)

    def move_motors(self):
        for i, name in enumerate(self.target_joints):
            dxl_id = JOINT_NAME_TO_ID[name]
            rad = self.current_angles[i]
            delta_deg = math.degrees(rad)
            pulse_change = int(delta_deg * GEAR_RATIOS[dxl_id] * (4096.0/360.0) * DIRECTION_MAP[dxl_id])
            goal = (self.initial_motor_pulses[dxl_id] + pulse_change) & 0xFFFFFFFF
            param = [DXL_LOBYTE(DXL_LOWORD(goal)), DXL_HIBYTE(DXL_LOWORD(goal)), DXL_LOBYTE(DXL_HIWORD(goal)), DXL_HIBYTE(DXL_HIWORD(goal))]
            self.groupSyncWrite.addParam(dxl_id, param)
        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()

    def print_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, rclpy.time.Time())
            pos = t.transform.translation
            quat = t.transform.rotation
            # 🌟 그리퍼 각도 추가 출력!
            gripper_val = self.current_angles[6]
            output_str = (f"{{'x': {pos.x:.3f}, 'y': {pos.y:.3f}, 'z': {pos.z:.3f}, "
                          f"'qx': {quat.x:.3f}, 'qy': {quat.y:.3f}, 'qz': {quat.z:.3f}, 'qw': {quat.w:.3f}, 'gripper': {gripper_val:.3f}}},")
            print(f"\n[📍 위치 기록 완료!] 복사해서 client.py에 넣으세요 👇\n{output_str}\n")
        except TransformException as ex:
            self.get_logger().error(f'❌ 좌표를 읽을 수 없습니다: {ex}')

def main(args=None):
    print("\n[티칭 모드] 원점 정렬 완료 후 'q'를 누르세요.")
    port_h, packet_h = calib.calibrate_origin()
    rclpy.init(args=args)
    node = TeachingNode(port_h, packet_h)
    
    print("=============================================")
    print(" [추가 조작 키] 그리퍼 제어: 'z' (닫기) / 'x' (열기)")
    print("=============================================")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = getKey()
            if key == '\x1b': break
            
            step = math.radians(1.0)
            if key.isupper():
                step = math.radians(5.0)
                key = key.lower()
                
            update_needed = False
            if key == 'q': node.current_angles[0] += step; update_needed = True
            elif key == 'a': node.current_angles[0] -= step; update_needed = True
            elif key == 'w': node.current_angles[1] += step; update_needed = True
            elif key == 's': node.current_angles[1] -= step; update_needed = True
            elif key == 'e': node.current_angles[2] += step; update_needed = True
            elif key == 'd': node.current_angles[2] -= step; update_needed = True
            elif key == 'r': node.current_angles[3] += step; update_needed = True
            elif key == 'f': node.current_angles[3] -= step; update_needed = True
            elif key == 't': node.current_angles[4] += step; update_needed = True
            elif key == 'g': node.current_angles[4] -= step; update_needed = True
            elif key == 'y': node.current_angles[5] += step; update_needed = True
            elif key == 'h': node.current_angles[5] -= step; update_needed = True
            # 🌟 그리퍼 조작 키 추가!
            elif key == 'z': node.current_angles[6] += step; update_needed = True
            elif key == 'x': node.current_angles[6] -= step; update_needed = True
            elif key == 'p' or key == ' ':
                node.print_pose()
                
            if update_needed: node.move_motors()
            node.publish_states()
    except Exception as e: print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if port_h.is_open: port_h.closePort()
        print("\n티칭 종료.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()