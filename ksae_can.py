# -*- coding: utf-8 -*-
# ============================================================
#  ksae_can.py  -  ksae_can.ino가 보내는 CAN 텔레메트리 수신 -> CSV 기록
#  - ksae_can.ino가 CAN(EZkontrol)에서 필요한 필드만 뽑아 시리얼로 보내면
#    이 스크립트가 받아서 CSV로 저장 (ksae_canlogging.ino의 SD 로깅 대체)
#  - 시리얼 라인 형식 : <speed_rpm>,<batt_v>,<batt_a>,<phase_a>,<gear>
# ============================================================

import argparse
import csv
import sys
import time

import serial
from serial.tools import list_ports

GEAR_NAMES = {0: "NO", 1: "R", 2: "N", 3: "D1", 4: "D2", 5: "D3", 6: "S", 7: "P"}

# ---------------------- 포트 자동 탐지 (kasa_example_control.py와 동일 방식) ----------------------
KNOWN_MEGA_VIDPID = {(0x2341, 0x0042), (0x2341, 0x0010), (0x2341, 0x003F), (0x2A03, 0x0042)}
CANDIDATE_VIDS = {0x1A86, 0x0403, 0x10C4}


def find_mega_port():
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid is not None and p.pid is not None and (p.vid, p.pid) in KNOWN_MEGA_VIDPID:
            return p.device, f"Arduino Mega 확정 ({p.description})"
    for p in ports:
        desc = (p.description or "").lower()
        if "mega" in desc or "arduino" in desc:
            return p.device, f"이름 매칭 ({p.description})"
    candidates = [p for p in ports if p.vid in CANDIDATE_VIDS]
    if len(candidates) == 1:
        p = candidates[0]
        return p.device, f"호환 칩 후보 1개 자동선택 ({p.description})"
    return None, None


def list_all_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("  사용 가능한 시리얼 포트가 없습니다.")
        return []
    print("  사용 가능한 포트:")
    for i, p in enumerate(ports):
        vid = f"{p.vid:04X}" if p.vid else "----"
        pid = f"{p.pid:04X}" if p.pid else "----"
        print(f"   [{i}] {p.device}  (VID:{vid} PID:{pid})  {p.description}")
    return ports


def resolve_port():
    port, reason = find_mega_port()
    if port:
        print(f"[포트] 자동 감지: {port}  ({reason})")
        return port
    print("[포트] 자동 감지 실패. 수동으로 선택하세요.")
    ports = list_all_ports()
    if not ports:
        return None
    sel = input("  포트 번호 선택 (그 외 입력=취소) > ").strip()
    if sel.isdigit() and 0 <= int(sel) < len(ports):
        return ports[int(sel)].device
    return None


# ---------------------- 라인 파싱 ----------------------
def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 5:
        return None
    try:
        speed = int(parts[0])
        batt_v = float(parts[1])
        batt_a = float(parts[2])
        phase_a = float(parts[3])
        gear = int(parts[4])
    except ValueError:
        return None
    return speed, batt_v, batt_a, phase_a, gear


# ---------------------- 메인 ----------------------
def main():
    ap = argparse.ArgumentParser(description="ksae_can.ino 시리얼 텔레메트리 수신 -> CSV 기록")
    ap.add_argument("--port", default=None, help="예: /dev/ttyACM0 또는 COM3 (생략 시 자동 탐지)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="ksae_can_log.csv")
    args = ap.parse_args()

    port = args.port or resolve_port()
    if port is None:
        print("연결할 포트를 결정하지 못했습니다. 종료합니다.")
        return

    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"시리얼 연결 실패: {e}")
        return

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["host_time", "speed_rpm", "batt_v", "batt_a", "phase_a", "gear"])

        print(f"[{port}] 수신 시작 -> {args.out} 에 기록 (Ctrl+C로 종료)")
        try:
            while True:
                raw = ser.readline().decode("ascii", errors="ignore")
                if not raw:
                    continue
                parsed = parse_line(raw)
                if parsed is None:
                    continue
                speed, batt_v, batt_a, phase_a, gear = parsed

                writer.writerow([time.time(), speed, batt_v, batt_a, phase_a, gear])
                f.flush()

                print(f"speed={speed:5d} rpm  battV={batt_v:5.1f} V  battA={batt_a:6.1f} A  "
                      f"phaseA={phase_a:6.1f} A  gear={GEAR_NAMES.get(gear, '?')}")
        except KeyboardInterrupt:
            print("\n중단됨.")
        finally:
            ser.close()


if __name__ == "__main__":
    main()
