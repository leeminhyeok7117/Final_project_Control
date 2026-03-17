#!/usr/bin/env python3
import time
from dynamixel_sdk import *

# 기존에 사용하시던 캘리브레이션 모듈 임포트
import calibrate_origin_keyboard as calib

# 테스트할 하드웨어 설정 (Follower 모터들)
FOLLOWER_IDS = [1, 2, 3, 4, 5, 6, 7]
GEAR_RATIOS = {1: 25, 2: 25, 3: 1, 4: 15, 5: 1, 6: 1, 7: 1}

# 🌟 방향 맵 (이 값을 수정해가며 테스트하세요!)
# 1이면 원래 방향, -1이면 반대 방향으로 돕니다.
DIRECTION_MAP = {1: 1, 2: 1, 3: -1, 4: 1, 5: 1, 6: -1, 7: 1}

BASE_MAX_VELOCITY = 200 # 안전을 위해 테스트 속도는 조금 낮게 설정

def main():
    print("\n[STEP 1] 로봇 팔을 원하는 '0도(차렷 자세)'로 만들고 정렬을 완료해주세요.")
    portHandler, packetHandler = calib.calibrate_origin()
    
    initial_pulses = {}
    max_ratio = max(GEAR_RATIOS.values())
    
    # [STEP 2] 정렬된 현재 위치를 '0도(Origin)'로 저장
    for dxl_id in FOLLOWER_IDS:
        pos, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, 132) # 현재 위치 읽기
        if pos > 2147483647: pos -= 4294967296
        initial_pulses[dxl_id] = pos
        
        # 토크 켜기 및 안전 속도 설정
        packetHandler.write1ByteTxRx(portHandler, dxl_id, 64, 1)
        vel = max(1, int(BASE_MAX_VELOCITY * (GEAR_RATIOS[dxl_id] / max_ratio)))
        packetHandler.write4ByteTxRx(portHandler, dxl_id, 112, vel)
        
    print("\n==================================================")
    print("✅ 준비 완료! 모터 각도 테스트를 시작합니다.")
    print("   입력 방법: [모터ID] [각도]")
    print("   입력 예시: 1 30   (1번 모터를 +30도 위치로 이동)")
    print("   입력 예시: 2 -45  (2번 모터를 -45도 위치로 이동)")
    print("   종료하려면 'q'를 입력하세요.")
    print("==================================================\n")
    
    try:
        while True:
            cmd = input("명령 입력 >> ")
            if cmd.lower() == 'q':
                break
                
            parts = cmd.split()
            if len(parts) != 2:
                print("❌ 잘못된 입력입니다. (띄어쓰기로 구분하여 2개 입력)")
                continue
                
            dxl_id = int(parts[0])
            angle_deg = float(parts[1])
            
            if dxl_id not in FOLLOWER_IDS:
                print(f"❌ {dxl_id}번 모터는 존재하지 않습니다.")
                continue
                
            # 🌟 핵심 계산: 사용자가 입력한 각도를 기어비/방향에 맞춰 펄스로 변환
            pulse_change = int(angle_deg * GEAR_RATIOS[dxl_id] * (4096.0 / 360.0) * DIRECTION_MAP[dxl_id])
            
            # 초기 영점 위치에 변화량을 더함
            goal_pulse = (initial_pulses[dxl_id] + pulse_change) & 0xFFFFFFFF
            
            # 모터로 목표 위치 전송
            packetHandler.write4ByteTxRx(portHandler, dxl_id, 116, goal_pulse)
            print(f" 🤖 -> {dxl_id}번 모터가 {angle_deg}도 위치로 이동합니다.")
            
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 안전하게 토크 끄고 포트 닫기
        for dxl_id in FOLLOWER_IDS:
            packetHandler.write1ByteTxRx(portHandler, dxl_id, 64, 0)
        portHandler.closePort()
        print("\n프로그램이 종료되었습니다.")

if __name__ == '__main__':
    main()