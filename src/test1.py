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
        self.MASTER_IDS = [0, 1, 2, 3, 4, 5]

        self.ADDR_AX_TORQUE = 24
        self.ADDR_AX_POS = 36
        self.ADDR_XL_TORQUE = 64
        self.ADDR_XL_POS = 132
        
        # [CLOSE 기준값 (Master 기준)]
        self.MASTER_CLOSE_THRESHOLD = 2170 
        # -----------------

        self.port_h = PortHandler(self.DEVICE_NAME)
        self.packet_h1 = PacketHandler(1.0) 
        self.packet_h2 = PacketHandler(2.0) 

        # [핵심] 학습용 데이터 저장소 (외부에서 이 변수를 가져다 쓰면 됩니다)
        # ID 0~4: Master 값 그대로
        # ID 5  : Master 값 반전 (4095 - Master)
        self.learning_state = {dxl_id: 0 for dxl_id in self.MASTER_IDS}

        # 통신 실패 카운트 등
        self.last_raw_positions = {dxl_id: 0 for dxl_id in self.MASTER_IDS}
        self.fail_counts = {dxl_id: 0 for dxl_id in self.MASTER_IDS}
        self.MAX_FAIL_COUNT = 10

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

        # 토크 OFF (Master는 손으로 움직임)
        for dxl_id in self.MASTER_IDS:
            if dxl_id == 5:
                self.packet_h2.write1ByteTxRx(self.port_h, dxl_id, self.ADDR_XL_TORQUE, 0)
            else:
                self.packet_h1.write1ByteTxRx(self.port_h, dxl_id, self.ADDR_AX_TORQUE, 0)
        
        self.get_logger().info("초기화 완료: 토크 OFF")
        print("-------------------------------------------------------")
        print(" ID |  Model  | Master Raw | Learning Val | Status ")
        print("-------------------------------------------------------")
        return True

    def process_data_for_learning(self, dxl_id, raw_pos):
        """
        Master에서 읽은 원본 값(raw_pos)을 
        Slave(학습 모델)가 사용할 값으로 변환하여 리턴합니다.
        """
        # 1. AX-12 (ID 0~4): 그대로 사용
        if dxl_id != 5:
            return raw_pos

        # 2. XL430 (ID 5): 방향 반전 처리 (Invert)
        else:
            # XL430 범위: 0 ~ 4095
            # Master가 커지면 Slave는 작아져야 하므로 (4095 - 값)
            inverted_pos = 4095 - raw_pos
            
            # 범위 보정 (혹시 모를 음수 방지)
            if inverted_pos < 0: inverted_pos = 0
            if inverted_pos > 4095: inverted_pos = 4095
            
            return inverted_pos

    def read_master_pos(self):
        try:
            print_buffer = []

            for dxl_id in self.MASTER_IDS:
                # 1. 읽기
                if dxl_id == 5:
                    pos, res, err = self.packet_h2.read4ByteTxRx(self.port_h, dxl_id, self.ADDR_XL_POS)
                else:
                    pos, res, err = self.packet_h1.read2ByteTxRx(self.port_h, dxl_id, self.ADDR_AX_POS)
                
                status_msg = ""
                
                # 2. 통신 성공 시 처리
                if res == COMM_SUCCESS:
                    self.last_raw_positions[dxl_id] = pos
                    self.fail_counts[dxl_id] = 0
                    
                    # [핵심] 학습용 변수에 데이터 가공해서 저장
                    learning_val = self.process_data_for_learning(dxl_id, pos)
                    self.learning_state[dxl_id] = learning_val

                # 3. 통신 실패 시 (이전 값 유지)
                else:
                    self.fail_counts[dxl_id] += 1
                    pos = self.last_raw_positions[dxl_id]
                    learning_val = self.learning_state[dxl_id]
                    status_msg = "(LOST)" if self.fail_counts[dxl_id] < self.MAX_FAIL_COUNT else "(DISCON!)"

                # 4. 출력용 메시지 생성
                model_name = "XL430" if dxl_id == 5 else "AX-12"
                
                # XL430 (ID 5)에 대한 CLOSE 표시 로직
                # Master의 Raw 값이 2170을 넘으면 닫힌 것으로 간주
                if dxl_id == 5:
                    if pos > self.MASTER_CLOSE_THRESHOLD:
                        status_msg = f"[CLOSE] (Raw>{self.MASTER_CLOSE_THRESHOLD})"
                    else:
                        status_msg = " [OPEN]"
                
                # 출력 포맷: [ID] [모델] [마스터원본] -> [학습용저장값] [상태]
                print_buffer.append(f"[{dxl_id:02d}] {model_name} :  {pos:04d}    ->   {learning_val:04d}      {status_msg}")

            # 화면 출력
            for line in print_buffer:
                print(line)
            
            # 커서 복귀
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
        print("\n" * (len(node.MASTER_IDS) + 2)) 
        
        # [확인용] 종료 시 마지막 저장된 학습 데이터 출력
        print("=== [최종 학습 데이터 (self.learning_state)] ===")
        print(node.learning_state)
        print("==============================================")
        
        node.get_logger().info('종료')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()