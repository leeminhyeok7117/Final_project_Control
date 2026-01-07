#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import sys
import time

class MasterSlaveTeleop(Node):
    def __init__(self):
        super().__init__('master_slave_teleop')
        
        # --- [설정 구역] ---
        self.PORT_MASTER = '/dev/ttyUSB0' 
        self.PORT_SLAVE  = '/dev/ttyUSB1'
        self.BAUDRATE    = 1000000

        # Master ID -> Slave ID 매핑
        self.ID_MAP = {
            0: 10, 
            1: 11, 
            2: 12, 
            3: 13, 
            4: 14, 
            5: 15 
        }
        
        # 주소값
        self.ADDR_AX_TORQUE    = 24
        self.ADDR_AX_POS_READ  = 36
        self.ADDR_AX_POS_WRITE = 30
        
        self.ADDR_XL_TORQUE    = 64
        self.ADDR_XL_POS_READ  = 132
        self.ADDR_XL_POS_WRITE = 116

        self.MAX_FAIL_COUNT = 10
        self.MASTER_CLOSE_THRESHOLD = 2170 # 닫힘 판단 기준 (Master Raw 값)
        # -----------------

        self.ph_master = PortHandler(self.PORT_MASTER)
        self.ph_slave  = PortHandler(self.PORT_SLAVE)

        self.packet_1 = PacketHandler(1.0)
        self.packet_2 = PacketHandler(2.0)

        self.last_positions = {m_id: None for m_id in self.ID_MAP.keys()}
        self.fail_counts    = {m_id: 0 for m_id in self.ID_MAP.keys()}

        # 1. 시스템 설정
        if not self.setup_ports(): return

        # 2. 안전 동기화
        if not self.safety_synchronize():
            self.get_logger().error("초기화 실패!")
            return

        # 3. 루프 시작
        self.get_logger().info("제어 시작 (각도 및 상태 표시 포함)")
        print("================================================================================")
        print("  Master   |  Slave Target (Inverted) |  Deg (Slave)  |   Status  ")
        print("================================================================================")
        self.timer = self.create_timer(0.1, self.control_loop)

    def setup_ports(self):
        if not self.ph_master.openPort() or not self.ph_slave.openPort(): return False
        if not self.ph_master.setBaudRate(self.BAUDRATE) or not self.ph_slave.setBaudRate(self.BAUDRATE): return False
        return True

    def calculate_slave_target(self, m_id, raw_pos):
        # 5번 (XL430) -> 반전
        if m_id == 5:
            inverted = 4095 - raw_pos
            return max(0, min(4095, inverted)) # 0~4095 클램핑
        return raw_pos

    def get_display_info(self, m_id, slave_target, master_raw):
        """
        화면 출력용 문자열 생성 함수 (각도 변환 및 상태 체크)
        """
        # 1. 각도 계산 (Slave가 움직이는 각도 기준)
        if m_id == 5: # XL430
            deg = (slave_target - 2048) * 0.088
            deg_str = f"({deg:6.1f}°)"
            
            # 상태 메시지 (CLOSE 체크는 Master가 쥔 값을 기준으로 함)
            status = ""
            if master_raw > self.MASTER_CLOSE_THRESHOLD:
                status = "[CLOSE] !!!"
            elif master_raw < 0 or master_raw > 4095:
                status = "[RANGE ERR]"
            else:
                status = "Running"
                
        else: # AX-12
            deg = (slave_target - 512) * 0.293
            deg_str = f"({deg:6.1f}°)"
            
            if slave_target < 0 or slave_target > 1023:
                status = "[RANGE ERR]"
            else:
                status = "Running"

        return deg_str, status

    def safety_synchronize(self):
        self.get_logger().info("안전 모드: 초기 위치 스캔 중...")
        # 토크 OFF
        for m_id in self.ID_MAP.keys():
            if m_id == 5: self.packet_2.write1ByteTxRx(self.ph_master, m_id, self.ADDR_XL_TORQUE, 0)
            else:         self.packet_1.write1ByteTxRx(self.ph_master, m_id, self.ADDR_AX_TORQUE, 0)
        time.sleep(0.5)

        # 위치 읽기
        for _ in range(5):
            all_read = True
            for m_id in self.ID_MAP.keys():
                if m_id == 5: pos, res, err = self.packet_2.read4ByteTxRx(self.ph_master, m_id, self.ADDR_XL_POS_READ)
                else:         pos, res, err = self.packet_1.read2ByteTxRx(self.ph_master, m_id, self.ADDR_AX_POS_READ)
                
                if res == COMM_SUCCESS:
                    self.last_positions[m_id] = self.calculate_slave_target(m_id, pos)
                else:
                    all_read = False
            if all_read: time.sleep(0.1)
        
        if None in self.last_positions.values(): return False

        # Slave 토크 ON
        self.get_logger().info("Slave 토크 ON! 동기화 완료.")
        for s_id in self.ID_MAP.values():
            if s_id == 15: self.packet_2.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_TORQUE, 1)
            else:          self.packet_1.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_TORQUE, 1)
        return True

    def control_loop(self):
        try:
            print_buffer = []

            for m_id, s_id in self.ID_MAP.items():
                # 1. Master Read
                if m_id == 5: pos, res, err = self.packet_2.read4ByteTxRx(self.ph_master, m_id, self.ADDR_XL_POS_READ)
                else:         pos, res, err = self.packet_1.read2ByteTxRx(self.ph_master, m_id, self.ADDR_AX_POS_READ)

                target_pos = 0
                status_msg = ""
                deg_str = ""
                extra_status = ""

                if res == COMM_SUCCESS:
                    # 데이터 변환 (5번 반전)
                    target_pos = self.calculate_slave_target(m_id, pos)
                    self.last_positions[m_id] = target_pos
                    self.fail_counts[m_id] = 0
                    status_msg = "SYNC"
                    
                    # 각도 및 상태 텍스트 생성
                    deg_str, extra_status = self.get_display_info(m_id, target_pos, pos)

                else:
                    self.fail_counts[m_id] += 1
                    status_msg = "LOST"
                    if self.last_positions[m_id] is not None:
                        target_pos = self.last_positions[m_id]
                        # 실패 시에도 이전 값 기준으로 각도 표시
                        deg_str, _ = self.get_display_info(m_id, target_pos, target_pos) # raw값은 정확지 않으므로 target으로 대체

                # 2. Slave Write
                if target_pos is not None:
                    try:
                        if s_id == 15:
                            self.packet_2.write4ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_POS_WRITE, target_pos)
                        else:
                            self.packet_1.write2ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_POS_WRITE, target_pos)
                    except:
                         status_msg = "WR-ERR"

                # 3. 출력 포맷
                # ID:05 (XL430)는 조금 더 눈에 띄게 표시
                if m_id == 5:
                    line = f" [ID:{m_id:02d}]      -> [ID:{s_id:02d}] {target_pos:04d}   {deg_str:<9} | {extra_status}"
                else:
                    line = f" [ID:{m_id:02d}]      -> [ID:{s_id:02d}] {target_pos:04d}   {deg_str:<9} | {status_msg}"
                
                print_buffer.append(line)

            for line in print_buffer: print(line)
            sys.stdout.write(f"\033[{len(self.ID_MAP)}F")
            sys.stdout.flush()

        except Exception: pass 

    def stop(self):
        try:
            for s_id in self.ID_MAP.values():
                if s_id == 15: self.packet_2.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_TORQUE, 0)
                else:          self.packet_1.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_TORQUE, 0)
        except: pass
        self.ph_master.closePort()
        self.ph_slave.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = MasterSlaveTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: 
        print("\n" * 10)
        node.get_logger().info('종료')
    finally:
        node.stop()
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()