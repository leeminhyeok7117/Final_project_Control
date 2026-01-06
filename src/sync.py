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
        # 1. 포트 설정
        self.PORT_MASTER = '/dev/ttyUSB0' # 읽기용 (사람이 움직임)
        self.PORT_SLAVE  = '/dev/ttyUSB1' # 쓰기용 (로봇이 따라함)
        self.BAUDRATE    = 1000000

        # 2. ID 매핑 (Master ID -> Slave ID)
        # 0번->10번, 1번->11번 ... 5번->15번
        self.ID_MAP = {
            0: 10, 
            1: 11, 
            2: 12, 
            3: 13, 
            4: 14, 
            5: 15  # 5번과 15번은 XL430 (Protocol 2.0)
        }
        
        # 3. 주소값 (Control Table)
        # [AX Series: ID 0~4 -> 10~14] Protocol 1.0
        self.ADDR_AX_TORQUE    = 24
        self.ADDR_AX_POS_READ  = 36  # 현재 위치
        self.ADDR_AX_POS_WRITE = 30  # 목표 위치 (Goal Position)
        
        # [XL Series: ID 5 -> 15] Protocol 2.0
        self.ADDR_XL_TORQUE    = 64
        self.ADDR_XL_POS_READ  = 132 # 현재 위치
        self.ADDR_XL_POS_WRITE = 116 # 목표 위치 (Goal Position)

        # 안전 설정 (통신 끊김 허용 횟수)
        self.MAX_FAIL_COUNT = 10
        # -----------------

        # 포트 핸들러 생성
        self.ph_master = PortHandler(self.PORT_MASTER)
        self.ph_slave  = PortHandler(self.PORT_SLAVE)

        # 패킷 핸들러 (1.0: AX용, 2.0: XL용)
        self.packet_1 = PacketHandler(1.0)
        self.packet_2 = PacketHandler(2.0)

        # 데이터 저장소 (Master ID 기준)
        self.last_positions = {m_id: 0 for m_id in self.ID_MAP.keys()}
        self.fail_counts    = {m_id: 0 for m_id in self.ID_MAP.keys()}

        # 초기화 및 실행
        if self.setup_system():
            # 0.05초(20Hz) 간격으로 제어 루프 실행
            self.timer = self.create_timer(0.05, self.control_loop)

    def setup_system(self):
        self.get_logger().info("시스템 초기화 중...")

        # 1. 포트 열기
        if not self.ph_master.openPort() or not self.ph_slave.openPort():
            self.get_logger().error("포트 열기 실패! USB 연결을 확인하세요.")
            return False

        # 2. 보드레이트 설정
        if not self.ph_master.setBaudRate(self.BAUDRATE) or not self.ph_slave.setBaudRate(self.BAUDRATE):
            self.get_logger().error("보드레이트 설정 실패!")
            return False

        # 3. 토크 설정 (가장 중요!)
        # Master = OFF (손으로 움직여야 함), Slave = ON (힘 줘야 함)
        for m_id, s_id in self.ID_MAP.items():
            
            # (1) XL430 (Master 5번 -> Slave 15번)
            if m_id == 5:
                # Master: Torque OFF
                self.packet_2.write1ByteTxRx(self.ph_master, m_id, self.ADDR_XL_TORQUE, 0)
                # Slave: Torque ON
                self.packet_2.write1ByteTxRx(self.ph_slave,  s_id, self.ADDR_XL_TORQUE, 1)
            
            # (2) AX-12/18A (Master 0~4번 -> Slave 10~14번)
            else:
                # Master: Torque OFF
                self.packet_1.write1ByteTxRx(self.ph_master, m_id, self.ADDR_AX_TORQUE, 0)
                # Slave: Torque ON
                self.packet_1.write1ByteTxRx(self.ph_slave,  s_id, self.ADDR_AX_TORQUE, 1)

        self.get_logger().info(f"설정 완료: Master({self.PORT_MASTER}) OFF <-> Slave({self.PORT_SLAVE}) ON")
        print("==================================================")
        print("   Master ID   ->   Slave ID   |   Status   ")
        print("==================================================")
        return True

    def control_loop(self):
        try:
            print_buffer = []

            for m_id, s_id in self.ID_MAP.items():
                # -----------------------------
                # 1. Master 위치 읽기 (Read)
                # -----------------------------
                # XL430 (ID 5)
                if m_id == 5:
                    pos, res, err = self.packet_2.read4ByteTxRx(self.ph_master, m_id, self.ADDR_XL_POS_READ)
                # AX-12 (ID 0~4)
                else:
                    pos, res, err = self.packet_1.read2ByteTxRx(self.ph_master, m_id, self.ADDR_AX_POS_READ)

                target_pos = 0
                status_msg = ""

                if res == COMM_SUCCESS:
                    # 읽기 성공: 값 갱신
                    self.last_positions[m_id] = pos
                    self.fail_counts[m_id] = 0
                    target_pos = pos
                    status_msg = "SYNC"
                else:
                    # 읽기 실패: 이전 값 유지 (안전장치)
                    self.fail_counts[m_id] += 1
                    target_pos = self.last_positions[m_id]
                    status_msg = "LOST (KEEP)"
                    
                    if self.fail_counts[m_id] > self.MAX_FAIL_COUNT:
                        status_msg = "DISCONNECTED!"
                        # 너무 오래 끊기면 위험하므로 여기서 루프 건너뛰기 가능
                        # 하지만 일단은 마지막 위치 유지를 위해 진행

                # -----------------------------
                # 2. Slave 위치 쓰기 (Write)
                # -----------------------------
                # XL430 (ID 15) - Protocol 2.0 (4Byte)
                if s_id == 15:
                    self.packet_2.write4ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_POS_WRITE, target_pos)
                    
                    # 5/15번은 그리퍼일 확률이 높으므로 닫힘 표시 로직
                    display_pos = f"{target_pos:04d}"
                    if target_pos > 2170: display_pos = "[CLOSE]"

                # AX-12 (ID 10~14) - Protocol 1.0 (2Byte)
                else:
                    self.packet_1.write2ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_POS_WRITE, target_pos)
                    display_pos = f"{target_pos:04d}"

                # 출력 라인 생성
                print_buffer.append(f"   [ID:{m_id:02d}] {display_pos:<7} -> [ID:{s_id:02d}]       | {status_msg}")

            # -----------------------------
            # 3. 화면 출력
            # -----------------------------
            for line in print_buffer:
                print(line)
            
            # 커서 올리기 (터미널 깜빡임 방지)
            sys.stdout.write(f"\033[{len(self.ID_MAP)}F")
            sys.stdout.flush()

        except Exception as e:
            self.get_logger().error(f"Error Loop: {e}")

    def stop(self):
        # 종료 시 모든 토크 해제 (안전)
        self.get_logger().info("종료 중... Slave 토크 해제")
        for m_id, s_id in self.ID_MAP.items():
            if s_id == 15:
                self.packet_2.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_XL_TORQUE, 0)
            else:
                self.packet_1.write1ByteTxRx(self.ph_slave, s_id, self.ADDR_AX_TORQUE, 0)
        
        self.ph_master.closePort()
        self.ph_slave.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = MasterSlaveTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" * 10)
        node.get_logger().info('제어 종료')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()