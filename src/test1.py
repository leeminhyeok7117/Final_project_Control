#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import sys

class MasterReader(Node):
    def __init__(self):
        super().__init__('master_reader')
        
        # --- [설정구역] ---
        self.DEVICE_NAME = '/dev/ttyUSB0'
        self.BAUDRATE = 1000000
        self.MASTER_IDS = [0, 1, 2, 3, 4, 5]  # ID 5번은 XL430

        self.ADDR_AX_TORQUE = 24
        self.ADDR_AX_POS = 36
        self.ADDR_XL_TORQUE = 64
        self.ADDR_XL_POS = 132
        
        # [안전 설정]
        self.MAX_FAIL_COUNT = 10 
        # -----------------

        self.port_h = PortHandler(self.DEVICE_NAME)
        self.packet_h1 = PacketHandler(1.0) 
        self.packet_h2 = PacketHandler(2.0) 

        # 위치 저장소 (초기값 0)
        self.last_positions = {dxl_id: 0 for dxl_id in self.MASTER_IDS}
        self.fail_counts = {dxl_id: 0 for dxl_id in self.MASTER_IDS}

        if not self.setup_dynamixel():
            return

        self.timer = self.create_timer(0.1, self.read_master_pos)

    def setup_dynamixel(self):
        if not self.port_h.openPort():
            self.get_logger().error("포트 연결 실패!")
            return False
        if not self.port_h.setBaudRate(self.BAUDRATE):
            self.get_logger().error("보드레이트 설정 실패!")
            return False

        # 토크 OFF
        for dxl_id in self.MASTER_IDS:
            if dxl_id == 5:
                self.packet_h2.write1ByteTxRx(self.port_h, dxl_id, self.ADDR_XL_TORQUE, 0)
            else:
                self.packet_h1.write1ByteTxRx(self.port_h, dxl_id, self.ADDR_AX_TORQUE, 0)
        
        self.get_logger().info("초기화 완료: 모든 모터 토크 OFF")
        print("--------------------------------")
        return True

    # [수정됨] 각도 변환 함수 (5번 모터 조건 추가)
    def convert_to_degree(self, dxl_id, raw_pos):
        # 1. AX-12/18A 로직 (0 ~ 1023)
        if dxl_id != 5:
            if raw_pos < 0 or raw_pos > 1023:
                return " [RANGE ERR]" 
            
            degree = (raw_pos - 512) * 0.293 
            return f" => {degree:6.1f}°"

        # 2. XL430 로직 (ID 5번)
        else:
            # [추가된 로직] 2170 초과 시 CLOSE 표시
            if raw_pos > 2170:
                return " => [CLOSE]   "  # 줄 맞춤을 위해 공백 추가

            # 정상 각도 계산
            if raw_pos < 0 or raw_pos > 4095:
                return " [RANGE ERR]"
            
            degree = (raw_pos - 2048) * 0.088 
            return f" => {degree:6.1f}°"

    def read_master_pos(self):
        try:
            for dxl_id in self.MASTER_IDS:
                # 1. 읽기 시도
                if dxl_id == 5:
                    pos, res, err = self.packet_h2.read4ByteTxRx(self.port_h, dxl_id, self.ADDR_XL_POS)
                else:
                    pos, res, err = self.packet_h1.read2ByteTxRx(self.port_h, dxl_id, self.ADDR_AX_POS)
                
                # 2. 데이터 처리
                if res == COMM_SUCCESS:
                    self.last_positions[dxl_id] = pos
                    self.fail_counts[dxl_id] = 0
                    
                    final_pos = pos
                    status_note = "" 
                else:
                    self.fail_counts[dxl_id] += 1
                    final_pos = self.last_positions[dxl_id] 

                    if self.fail_counts[dxl_id] < self.MAX_FAIL_COUNT:
                        status_note = "(KEEP)"
                    else:
                        status_note = "(DISCONNECTED!)"

                # 3. 출력 메시지 조립
                model_name = "(XL430)" if dxl_id == 5 else "(AX-12)"
                
                # 각도 변환 값 가져오기
                angle_str = self.convert_to_degree(dxl_id, final_pos)
                
                # 최종 출력
                print(f"[ID:{dxl_id:02d}] {model_name} Pos: {final_pos:04d} {status_note:<15} {angle_str}")

            # 커서 올리기
            sys.stdout.write(f"\033[{len(self.MASTER_IDS)}F")
            sys.stdout.flush()

        except Exception as e:
            self.get_logger().error(f"시스템 에러: {e}")

    def stop(self):
        self.port_h.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = MasterReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" * len(node.MASTER_IDS)) 
        node.get_logger().info('테스트 종료')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()