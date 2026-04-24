#!/usr/bin/env python3
"""
15:1 감속기 모터 각도 정확도 테스트
대상: Motor ID 4 (GEAR_RATIO = 15)
동작: 0° → 30° → 60° → 90° 순서로 이동, 10회 반복
확인: 각 위치에서 정지 후 배경 각도기와 육안 비교
"""
import sys
import time
import statistics
from dynamixel_sdk import *

# ============================================================
# 통신 설정
# ============================================================
DXL_PORT     = '/dev/ttyUSB0'
DXL_BAUDRATE = 1000000
PROTOCOL_VER = 2.0

# 컨트롤 테이블 주소 (XL430 X-Series)
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE        = 1
TORQUE_DISABLE       = 0
OP_MODE_EXT_POSITION = 4   # 확장 포지션 모드 (멀티턴, 감속기 장착 모터 필수)

# ============================================================
# 테스트 파라미터
# ============================================================
MOTOR_ID     = 26            # 15:1 감속기가 달린 모터 ID
GEAR_RATIO   = 15           # 감속비

PROFILE_VEL  = 300           # 이동 속도 (낮을수록 천천히, 정밀도 ↑)
HOLD_SEC     = 2.0          # 각 위치에서 육안 확인을 위해 정지하는 시간 (초)
TOLERANCE    = 25           # 도달 판정 허용 오차 (펄스 단위, 출력축 기준 ~0.15°)
TIMEOUT_SEC  = 12.0         # 위치 도달 최대 대기 시간 (초)

TEST_ANGLES  = [0, -30, -60, -90]   # 테스트할 출력축 목표 각도 (도)
REPETITIONS  = 10                # 반복 횟수

# ============================================================
# 유틸리티 함수
# ============================================================
def deg_to_pulse(deg: float) -> int:
    """출력축 각도(도) → 모터 펄스 변화량"""
    return int(deg * GEAR_RATIO * 4096.0 / 360.0)

def pulse_to_deg(pulse: int) -> float:
    """모터 펄스 변화량 → 출력축 각도(도)"""
    return pulse * 360.0 / (4096.0 * GEAR_RATIO)

def read_position(ph, port, motor_id) -> int | None:
    pos, res, _ = ph.read4ByteTxRx(port, motor_id, ADDR_PRESENT_POSITION)
    if res != COMM_SUCCESS:
        return None
    if pos > 2147483647:
        pos -= 4294967296
    return pos

def send_goal(ph, port, motor_id, goal_pulse: int):
    val = goal_pulse & 0xFFFFFFFF
    ph.write4ByteTxRx(port, motor_id, ADDR_GOAL_POSITION, val)

def wait_until_reached(ph, port, motor_id, goal_pulse, tolerance, timeout) -> tuple[bool, int]:
    start = time.time()
    while True:
        pos = read_position(ph, port, motor_id)
        if pos is not None and abs(pos - goal_pulse) <= tolerance:
            return True, pos
        if time.time() - start > timeout:
            pos = read_position(ph, port, motor_id) or goal_pulse
            return False, pos
        time.sleep(0.01)

# ============================================================
# 메인
# ============================================================
def main():
    port = PortHandler(DXL_PORT)
    ph   = PacketHandler(PROTOCOL_VER)

    # 포트 열기
    if not port.openPort():
        print(f"[ERROR] 포트 열기 실패: {DXL_PORT}")
        sys.exit(1)
    if not port.setBaudRate(DXL_BAUDRATE):
        print(f"[ERROR] 보드레이트 설정 실패: {DXL_BAUDRATE}")
        sys.exit(1)
    print(f"[OK] 연결 성공: {DXL_PORT} @ {DXL_BAUDRATE}bps")

    # 확장 포지션 모드 설정 (토크 OFF → 모드 변경 → 토크 ON)
    ph.write1ByteTxRx(port, MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    ph.write1ByteTxRx(port, MOTOR_ID, ADDR_OPERATING_MODE, OP_MODE_EXT_POSITION)
    ph.write1ByteTxRx(port, MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    ph.write4ByteTxRx(port, MOTOR_ID, ADDR_PROFILE_VELOCITY, PROFILE_VEL)
    print(f"[OK] 모터 {MOTOR_ID}번 준비완료 (감속비 {GEAR_RATIO}:1, 확장포지션 모드, 속도={PROFILE_VEL})")

    # 현재 위치를 원점(0°)으로 기록
    origin = read_position(ph, port, MOTOR_ID)
    if origin is None:
        print("[ERROR] 모터 위치 읽기 실패")
        port.closePort()
        sys.exit(1)
    print(f"[OK] 원점 펄스 기록: {origin}")

    print(f"\n테스트 시작: 목표 각도 {TEST_ANGLES}°, {REPETITIONS}회 반복")
    print(f"각 위치에서 {HOLD_SEC}초 정지 (육안 확인)")
    print("=" * 65)

    all_results: list[list[tuple]] = []

    try:
        for rep in range(1, REPETITIONS + 1):
            print(f"\n[ 반복 {rep:2d} / {REPETITIONS} ]")
            rep_row = []

            for target_deg in TEST_ANGLES:
                goal_pulse = origin + deg_to_pulse(target_deg)
                send_goal(ph, port, MOTOR_ID, goal_pulse)

                reached, actual_pulse = wait_until_reached(
                    ph, port, MOTOR_ID, goal_pulse, TOLERANCE, TIMEOUT_SEC
                )

                actual_deg = pulse_to_deg(actual_pulse - origin)
                error_deg  = actual_deg - target_deg
                flag = "OK     " if reached else "TIMEOUT"

                print(f"  목표 {target_deg:2d}°  →  실제 {actual_deg:6.2f}°  "
                      f"(오차 {error_deg:+.2f}°)  [{flag}]")

                rep_row.append((target_deg, actual_deg, error_deg, reached))
                time.sleep(HOLD_SEC)

            all_results.append(rep_row)

    except KeyboardInterrupt:
        print("\n[중단] 키보드 인터럽트")

    finally:
        print(f"\n원점으로 복귀 중...")
        send_goal(ph, port, MOTOR_ID, origin)
        time.sleep(3.0)

        ph.write1ByteTxRx(port, MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        port.closePort()
        print("[OK] 토크 OFF, 포트 닫힘")

    # ---- 요약 출력 ----
    if not all_results:
        return

    print("\n" + "=" * 65)
    print("결과 요약 (출력축 기준)")
    print("=" * 65)
    print(f"{'각도':>6}  {'평균오차':>10}  {'최대|오차|':>10}  {'성공':>6}")
    print("-" * 65)

    for i, angle in enumerate(TEST_ANGLES):
        rows = [rep[i] for rep in all_results if len(rep) > i]
        errors  = [r[2] for r in rows]
        success = sum(1 for r in rows if r[3])
        avg_e   = statistics.mean(errors)
        max_e   = max(abs(e) for e in errors)
        print(f"  {angle:3d}°  {avg_e:>+10.3f}°  {max_e:>10.3f}°  {success:>3}/{len(rows)}")

    print("=" * 65)


if __name__ == '__main__':
    main()
