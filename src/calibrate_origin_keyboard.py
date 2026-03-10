import os
import sys
import tty
import termios
import time
from dynamixel_sdk import * # --- 1. 통신 및 포트 설정 ---
DXL_PORT = '/dev/ttyUSB0'     # 우분투 다이나믹셀 포트
DXL_BAUDRATE = 1000000

# --- 2. 컨트롤 테이블 주소 (XL430 등 X시리즈 기준) ---
ADDR_OPERATING_MODE     = 11
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_PROFILE_VELOCITY   = 112 

# --- 3. 기본 설정 값 ---
PROTOCOL_VERSION        = 2.0
TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

OP_MODE_POSITION        = 3  # 일반 포지션 모드 (0~4095)
OP_MODE_EXT_POSITION    = 4  # 확장 포지션 모드 (멀티턴)

# 모터 분류 및 설정
GEARED_MOTORS = [1, 2, 4]
NORMAL_MOTORS = [3, 5, 6, 7]

JOG_STEP = 300 # 한 번 키보드를 누를 때마다 움직일 펄스 양 (크면 빨리, 작으면 정밀하게 돎)

portHandler = PortHandler(DXL_PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def getch():
    """리눅스 환경에서 엔터키 없이 실시간으로 키보드 입력을 받는 함수"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def setup():
    if not portHandler.openPort():
        print("다이나믹셀 포트를 열 수 없습니다.")
        quit()
    if not portHandler.setBaudRate(DXL_BAUDRATE):
        print("보드레이트 변경 실패.")
        quit()
        
    # 기어 모터들을 초기 조그(Jog) 이동을 위해 확장 포지션 모드로 세팅
    for dxl_id in GEARED_MOTORS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_EXT_POSITION)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def read_present_position(dxl_id):
    pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if pos > 2147483647:
        pos -= 4294967296 # 음수 처리 (2의 보수)
    return pos

def jog_motor(dxl_id, direction):
    """지정된 모터를 현재 위치에서 JOG_STEP 만큼 이동시킴"""
    current_pos = read_present_position(dxl_id)
    target_pos = current_pos + (JOG_STEP * direction)
    
    # 음수 값을 다이나믹셀 전송용 4바이트로 변환
    write_pos = target_pos & 0xFFFFFFFF
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, write_pos)
    print(f"\r[{dxl_id}번 모터] 현재위치: {current_pos} -> 목표위치: {target_pos}        ", end="")

def reboot_and_home_geared(dxl_id):
    """모터를 재부팅하여 멀티턴을 0회전으로 초기화한 뒤 2048(센터)로 정렬"""
    print(f"\n\n[{dxl_id}번 모터] 재부팅 및 멀티턴 초기화를 진행합니다...")
    packetHandler.reboot(portHandler, dxl_id)
    time.sleep(1.0) # 모터가 완전히 켜질 때까지 대기
    
    # 다시 세팅 (토크 끄기 -> 확장 포지션 모드 설정 -> 토크 켜기)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_EXT_POSITION)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    
    # 2048(현재 회전의 완벽한 중앙)로 이동
    print(f"[{dxl_id}번 모터] 절대 위치 2048로 정렬합니다.")
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, 2048)
    time.sleep(0.5)

def home_normal_motors():
    """나머지 일반 모터(3,5,6,7)를 포지션 모드로 변경 후 2048로 이동"""
    print("\n\n--- 일반 모터(3, 5, 6, 7) 원점 복귀 ---")
    for dxl_id in NORMAL_MOTORS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_POSITION)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 20)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, 2048)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 0)
        print(f"[{dxl_id}번 모터] 2048로 이동 명령 전송 완료.")

def calibrate_origin():
    setup()
    
    print("\n=======================================================")
    print("      키보드 수동 원점 정렬 (Manual Homing) 프로그램      ")
    print("=======================================================")
    print(" [ 1, 2, 4 ] 숫자키 : 조종할 모터 선택 (기본: 1번)")
    print(" [ a ] / [ d ] : 선택한 모터 시계 반대 / 시계 방향으로 회전")
    print(" [ r ] : 선택한 모터 멀티턴 초기화(Reboot) 및 2048로 정렬")
    print(" [ h ] : 나머지 모터(3, 5, 6, 7) 2048로 원점 복귀")
    print(" [ q ] : 프로그램 종료")
    print("=======================================================\n")
    
    selected_motor = 1
    print(f"-> 현재 선택된 모터: {selected_motor}번")
    
    try:
        while True:
            key = getch()
            if key == '\x03':
                print("\n[Ctrl+C] 강제 종료됨")
                break
            if key == 'q':
                print("\n프로그램을 종료합니다.")
                break
            elif key in ['1', '2', '4']:
                selected_motor = int(key)
                print(f"\n-> {selected_motor}번 모터 선택됨")
            elif key == 'a':
                jog_motor(selected_motor, -1) # 감소 방향
            elif key == 'd':
                jog_motor(selected_motor, 1)  # 증가 방향
            elif key == 'r':
                reboot_and_home_geared(selected_motor)
            elif key == 'h':
                home_normal_motors()
                
    except KeyboardInterrupt:
        print("\n강제 종료됨")
        portHandler.closePort()
        quit()
    return portHandler, packetHandler

if __name__ == '__main__':
    port, packet = calibrate_origin()
    # 단독 실행 테스트가 끝나면 포트를 닫아줌
    port.closePort()