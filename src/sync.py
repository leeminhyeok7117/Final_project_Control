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
        # 포트가 바뀌었는지 꼭 확인하세요 (로그상 USB1, USB2 사용중)
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
        
        # 주소값 (Control Table)
        # AX (Protocol 1.0)
        self.ADDR_AX_TORQUE    = 24
        self.ADDR_AX_POS_READ  = 36
        self.ADDR_AX_POS_WRITE = 30
        
        # XL (Protocol 2.0)
        self.ADDR_XL_TORQUE    = 64
        self.ADDR_XL_POS_READ  = 132
        self.ADDR_XL_POS_WRITE = 116

        self.MAX_FAIL_COUNT = 10
        # -----------------

        self.ph_master = PortHandler(self.PORT_MASTER)
        self.ph_slave  = PortHandler(self.PORT_SLAVE)

        self.packet_1 = PacketHandler(1.0)
        self.packet_2 = PacketHandler(2.0)

        # 초기값을 0이 아닌 None으로 설정 (안전장치)
        self.last_positions = {m_id: None for m_id in self.ID_MAP.keys()}
        self.fail_counts    = {m_id: 0 for m_id in self.ID_MAP.keys()}

        # 1. 시스템 설정
        if not self.setup_ports():
            return

        # 2. [핵심] 마스터 위치 안전 초기화 (이게 끝나야 토크가 켜짐)
        if not self.safety_synchronize():
            self.get_logger().error("초기화 실패: 마스터 값을 읽을 수 없습니다.")
            return

        # 3. 제어 루프 시작
        self.get_logger().info("안전 동기화 완료! 제어를 시작합니다.")
        self.timer = self.create_timer(0.05, self.control_loop)

    def setup_ports(self):
        # 포트 열기
        if not self.ph_master.openPort() or not self.ph_slave.openPort():
            self.get_logger().error("포트 열기 실패! USB 연결 확인")
            return False

        # 보드레이트
        if not self.ph_master.setBaudRate(self.BAUDRATE) or not self.ph_slave.setBaudRate(self.BAUDRATE):
            self.get_logger().error("보드레이트 설정 실패!")
            return False
            
        return True

    def safety_synchronize(self):
        """
        슬레이브 토크를 켜기 전에 마스터의 현재 위치를 확실하게 읽어옵니다.
        """
        self.get_logger().info("안전 모드: 마스터 위치를 읽어오는 중... (움직이지 마세요)")
        
        # 1. 마스터 토크 끄기 (손으로 움직이기 위해)
        for m_id in self.ID_MAP.keys():
            if m_id == 5: self.packet_2.write1ByteTxRx(self.ph_master, m_id, self.ADDR_XL_TORQUE, 0)
            else:         self.packet_1.write1ByteTxRx(self.ph_master, m_id, self.ADDR_AX_TORQUE, 0)

        time.sleep(0.5) # 안정화 대기

        # 2. 마스터 위치 읽기 시도 (최대 5번 시도)
        success_count = 0
        for _ in range(5):
            all_read = True
            for m_id in self.ID_MAP.keys():
                # 읽기
                if m_id == 5:
                    pos, res, err = self.packet_2.read4ByteTxRx(self.ph_master, m_id, self.ADDR_XL_POS_READ)
                else:
                    pos, res, err = self.packet_1.read2ByteTxRx(self.ph_master, m_id, self.ADDR_AX_POS_READ)
                
                if res == COMM_SUCCESS:
                    self.last_positions[m_id] = pos # 여기서 실제 값으로 채워짐
                else:
                    all_read = False
            
            if all_read:
                success_count += 1
                time.sleep(0.1)
        
        # 하나라도 못 읽었으면 시작 안 함
        if None in self.last_positions.values():
            self.get_logger().error("마스터 위치를 읽지 못했습니다. 케이블을 확인하세요.")
            return False

        self.get_logger().info(f"초기 위치 확인 완료: {self.last_positions}")
        
        # 3. [중요] 이제서야 슬레이브 토크 켬 (초기값이 확보되었으므로)
        self.get_logger().info("슬레이브 토크 ON...")
        for s_id in self.ID_MAP.values():
            if s_id == 15:
                self.packet_2.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_TORQUE, 1)
            else:
                self.packet_1.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_TORQUE, 1)
        
        return True

    def control_loop(self):
        try:
            print_buffer = []

            for m_id, s_id in self.ID_MAP.items():
                # --- 1. Master Read ---
                if m_id == 5:
                    pos, res, err = self.packet_2.read4ByteTxRx(self.ph_master, m_id, self.ADDR_XL_POS_READ)
                else:
                    pos, res, err = self.packet_1.read2ByteTxRx(self.ph_master, m_id, self.ADDR_AX_POS_READ)

                target_pos = 0
                status_msg = ""

                if res == COMM_SUCCESS:
                    self.last_positions[m_id] = pos
                    self.fail_counts[m_id] = 0
                    target_pos = pos
                    status_msg = "SYNC"
                else:
                    # 읽기 실패 시
                    self.fail_counts[m_id] += 1
                    
                    # [안전] 초기값이 없으면(None) 절대 0을 보내지 않고 무시
                    if self.last_positions[m_id] is None:
                        status_msg = "INIT FAIL"
                        target_pos = None 
                    else:
                        target_pos = self.last_positions[m_id] # 이전 값 유지
                        status_msg = "KEEP"

                    if self.fail_counts[m_id] > self.MAX_FAIL_COUNT:
                        status_msg = "DISCONNECTED!"
                
                # --- 2. Slave Write ---
                # target_pos가 None이면(초기화 안됨) 쓰지 않음
                if target_pos is not None:
                    try:
                        if s_id == 15:
                            self.packet_2.write4ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_POS_WRITE, target_pos)
                            disp = f"{target_pos:04d}"
                            if target_pos > 2170: disp = "[CLOSE]"
                        else:
                            self.packet_1.write2ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_POS_WRITE, target_pos)
                            disp = f"{target_pos:04d}"
                    except Exception:
                         # 쓰기 실패해도 일단 넘어감 (I/O 에러 방지)
                         status_msg = "WRITE ERR"

                print_buffer.append(f"   [ID:{m_id:02d}] {disp:<7} -> [ID:{s_id:02d}]       | {status_msg}")

            # 출력
            for line in print_buffer:
                print(line)
            sys.stdout.write(f"\033[{len(self.ID_MAP)}F")
            sys.stdout.flush()

        except Exception as e:
            # I/O 에러 발생 시 로그만 남기고 멈추지 않음 (USB 흔들림 대비)
            pass 

    def stop(self):
        self.get_logger().info("종료: 슬레이브 힘 빼기")
        try:
            for s_id in self.ID_MAP.values():
                if s_id == 15: self.packet_2.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_TORQUE, 0)
                else:          self.packet_1.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_TORQUE, 0)
        except:
            pass
        
        self.ph_master.closePort()
        self.ph_slave.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = MasterSlaveTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" * 10)
        node.get_logger().info('사용자 종료')
    finally:
        node.stop()
        node.destroy_node()
        # rclpy.shutdown() # 중복 호출 에러 방지를 위해 제거하거나 try로 감쌈
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()