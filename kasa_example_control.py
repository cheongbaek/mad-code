# -*- coding: utf-8 -*-
# ============================================================
#  kasa.py  -  통합 차량 간이 제어 (PC <-> Arduino Mega2560)
#  - json 파일 의존 없음 (모든 설정을 이 파일 상단에 하드코딩)
#  - 시리얼 포트 자동 감지 (Mega2560 / 호환보드)
#  - DC 조향: 각도/한계/매핑은 파이썬이 관리, 아두이노는 GP,<raw>만 수행
#  - DC 메뉴: 1)보정 시도  2)진행(하드코딩값)
#  - 메뉴: 1)인휠 2)DC 3)리니어(틀만) 4)E-stop / 그 외 종료
# ============================================================

import serial
from serial.tools import list_ports
import threading
import time
import sys
import asyncio

if sys.platform == "win32":
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout

# ==========================================================
#  설정 (json 대체 - 전부 여기 하드코딩)
# ==========================================================
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.05
INWHEEL_PWM_MAX_REF = 150

# DC 조향 보정값
DC = {
    "pos_left": 944,
    "pos_right": 645,
    "pos_center": 793,
    "steer_angle_max": 30,
    "calibrated": True,
    "invert": False,   # 방향 확인 후 런타임에서 갱신될 수 있음
}


# ---------------------- 포트 자동 탐지 ----------------------
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


# ---------------------- 각도 <-> 위치 매핑 ----------------------
def _imap(x, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    return int(out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min))


def angle_to_pos(deg, dc, edge_margin=15):
    """각도(-amax~amax) -> 목표 저항값. invert 시 좌우 반전, 끝에서 margin 안쪽 클램프."""
    amax = dc["steer_angle_max"]
    left = dc["pos_left"]
    right = dc["pos_right"]
    if dc.get("invert", False):
        left, right = right, left
    if left > right:
        work_left = left - edge_margin
        work_right = right + edge_margin
    else:
        work_left = left + edge_margin
        work_right = right - edge_margin
    deg = max(-amax, min(amax, deg))
    return _imap(deg, -amax, amax, work_left, work_right)


# ---------------------- SerialLink ----------------------
class SerialLink:
    def __init__(self, port, baud, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._running = True
        self._print_enabled = False
        self._fmt = "raw"
        self._lock = threading.Lock()
        self._last_lines = []
        self._capture = False
        self._steer_guard = False
        self._steer_lo = 0
        self._steer_hi = 1023
        self.cur_pos = 512
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()
        time.sleep(2.0)

    def _reader(self):
        buf = b""
        while self._running:
            try:
                data = self.ser.read(256)
            except (serial.SerialException, OSError):
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if not text:
                    continue
                if text.startswith("D,"):
                    parts = text.split(",")
                    if len(parts) >= 3:
                        try:
                            self.cur_pos = int(parts[2])
                        except ValueError:
                            pass
                if self._capture:
                    self._last_lines.append(text)
                if self._print_enabled:
                    self._print_formatted(text)

    def _print_formatted(self, text):
        parts = text.split(",")
        tag = parts[0]
        try:
            if self._fmt == "ref" and tag == "W":
                print(f"{parts[2]} {parts[3]} {parts[4]} {parts[5]}")
            elif self._fmt == "pwm" and tag == "W":
                print(f"{parts[2]} {parts[6]} {parts[5]}")
            elif self._fmt == "cal" and tag == "D":
                print(parts[2])  # current
            elif self._fmt == "steer" and tag == "D":
                cur = int(parts[2])
                print(f"Current:{cur}  Target:{parts[1]}")
                if self._steer_guard and (cur <= self._steer_lo or cur >= self._steer_hi):
                    self.send("E")
                    self._steer_guard = False
                    print(f"!! 끝단 도달({cur}) → E-STOP")
            elif self._fmt == "raw":
                print(text)
        except (IndexError, ValueError):
            pass

    def send(self, msg):
        with self._lock:
            try:
                self.ser.write((msg + "\n").encode())
            except (serial.SerialException, OSError):
                pass

    def query(self, msg, prefix, timeout=1.0):
        self._last_lines = []
        self._capture = True
        self.send(msg)
        t0 = time.time()
        result = None
        while time.time() - t0 < timeout:
            for ln in list(self._last_lines):
                if ln.startswith(prefix):
                    result = ln
                    break
            if result:
                break
            time.sleep(0.01)
        self._capture = False
        return result

    def set_print(self, on, fmt="raw"):
        self._fmt = fmt
        self._print_enabled = on

    def set_steer_limits(self, pos_left, pos_right, margin=0):
        lo = min(pos_left, pos_right)
        hi = max(pos_left, pos_right)
        self._steer_lo = lo + margin
        self._steer_hi = hi - margin
        self._steer_guard = True

    def clear_steer_guard(self):
        self._steer_guard = False

    def read_pos(self, timeout=0.5):
        r = self.query("?", "D,", timeout=timeout)
        if r:
            parts = r.split(",")
            if len(parts) >= 3:
                try:
                    return int(parts[2])
                except ValueError:
                    pass
        return self.cur_pos

    def close(self):
        self._running = False
        time.sleep(0.1)
        try:
            self.ser.close()
        except OSError:
            pass


# ---------------------- UI 유틸 ----------------------
async def ask(session, prompt_text):
    with patch_stdout():
        return (await session.prompt_async(prompt_text)).strip()


# ---------------------- 메뉴 1: 인휠 ----------------------
async def menu_inwheel(session, link):
    while True:
        print("\n[인휠모터] 모드 선택: 1)REF  2)PWM  3)THROTTLE  (q=뒤로)")
        sel = await ask(session, "inwheel> ")
        if sel == "q":
            link.send("MW,STOP")
            return
        elif sel == "1":
            await inwheel_ref(session, link)
        elif sel == "2":
            await inwheel_pwm(session, link)
        elif sel == "3":
            await inwheel_throttle(session, link)
        else:
            print("잘못된 선택")


async def inwheel_ref(session, link):
    link.send("MW,REF")
    link.send("S,0")
    link.set_print(True, fmt="ref")
    print("[REF 모드] 0 이상 정수 = 목표 REF / q=뒤로")
    print("(출력: REF  실제속도  ERR  출력PWM)")
    try:
        while True:
            s = await ask(session, "REF> ")
            if s == "q":
                break
            if s.isdigit():
                link.send(f"S,{int(s)}")
            else:
                print("0 이상 정수만")
    finally:
        link.send("S,0")
        link.send("MW,STOP")
        link.set_print(False)


async def inwheel_pwm(session, link):
    link.send("MW,PWM")
    link.send("P,0")
    link.set_print(True, fmt="pwm")
    print("[PWM 모드] 0~255 직접 전송(상한 없음) / q=뒤로")
    print("(출력: 목표PWM  환산REF  출력PWM)")
    try:
        while True:
            s = await ask(session, "PWM> ")
            if s == "q":
                break
            if s.isdigit() and 0 <= int(s) <= 255:
                link.send(f"P,{int(s)}")
            else:
                print("0~255 정수만")
    finally:
        link.send("P,0")
        link.send("MW,STOP")
        link.set_print(False)


async def inwheel_throttle(session, link):
    link.send("MW,THR")
    link.set_print(True, fmt="pwm")
    print("[THROTTLE 모드] A0 페달값 그대로 전송(상한 없음) / q=뒤로")
    print("(출력: 목표PWM  환산REF  출력PWM)")
    try:
        while True:
            s = await ask(session, "THR> ")
            if s == "q":
                break
            print("페달로 제어 중. (q=뒤로)")
    finally:
        link.send("MW,STOP")
        link.set_print(False)


# ---------------------- 메뉴 2: DC 조향 ----------------------
async def menu_dc(session, link):
    print(f"\n[DC] 현재 보정값: L={DC['pos_left']} R={DC['pos_right']} "
          f"C={DC['pos_center']} angle=±{DC['steer_angle_max']} invert={DC['invert']}")
    print("  1) 보정 시도   2) 진행(현재값 그대로)   (그 외=뒤로)")
    sel = await ask(session, "dc> ")
    if sel == "1":
        if not await calibrate(session, link):
            print("[DC] 보정 취소 → 메인 메뉴로 복귀")
            return
        await steer_test(session, link)
    elif sel == "2":
        await steer_test(session, link)
    else:
        return


async def calibrate(session, link):
    """방향 확인 -> 왼쪽 끝 -> 오른쪽 끝 -> 중앙 -> 각도. 결과는 런타임 DC에 반영."""
    # 1) 방향 확인 먼저
    await verify_direction(session, link)

    # 2) 수동 보정 (손으로 핸들 회전, 모터 미구동)
    link.send("CAL,START")
    print("\n[보정] 모터는 구동되지 않습니다. 손으로 핸들을 돌리세요.")
    print("       (현재 A1값 실시간 표시)")

    link.set_print(True, fmt="cal")
    await ask(session, "1) 핸들을 '왼쪽 끝'까지 돌린 뒤 Enter > ")
    link.set_print(False)
    DC["pos_left"] = link.read_pos()
    print(f"   왼쪽 끝 = {DC['pos_left']}")

    link.set_print(True, fmt="cal")
    await ask(session, "2) 핸들을 '오른쪽 끝'까지 돌린 뒤 Enter > ")
    link.set_print(False)
    DC["pos_right"] = link.read_pos()
    print(f"   오른쪽 끝 = {DC['pos_right']}")

    link.set_print(True, fmt="cal")
    await ask(session, "3) 핸들을 '중앙'에 둔 뒤 Enter > ")
    link.set_print(False)
    DC["pos_center"] = link.read_pos()
    print(f"   중앙 = {DC['pos_center']}")

    s = await ask(session, "4) 최대 조향각(예: 30 → -30~30도) 입력 > ")
    if s.lstrip("-").isdigit():
        DC["steer_angle_max"] = abs(int(s))

    link.send("CAL,END")
    DC["calibrated"] = True
    print(f"   적용 완료(런타임): L={DC['pos_left']} R={DC['pos_right']} C={DC['pos_center']} "
          f"angle=±{DC['steer_angle_max']} invert={DC['invert']}")
    print("   ※ 영구 반영하려면 코드 상단 DC 딕셔너리 값을 이 값으로 수정하세요.")
    return True


async def verify_direction(session, link):
    """
    방향 재확인:
      - 중앙 정렬 후 '왼쪽(pos_left)' 방향으로 0.2s 이동
      - "왼쪽으로 돌았습니까?" 질문 (y/n)
      - 애매하면 반대로 한 번 더 돌려 재확인
      - 결과로 DC['invert'] 갱신 (런타임 한정)
    """
    center = DC["pos_center"]
    step = 40  # 검증 이동폭(raw). 모터 속도에 맞게 조정.

    print("\n[방향 확인] 모터가 짧게 '왼쪽' 방향으로 움직입니다. 실제 움직임을 보세요.")

    link.send(f"GP,{center}")
    await asyncio.sleep(0.3)
    target_left = center + (step if DC["pos_left"] > center else -step)
    link.send(f"GP,{target_left}")
    await asyncio.sleep(0.2)
    link.send(f"GP,{link.read_pos()}")  # 정지

    ans = (await ask(session, "왼쪽으로 돌았습니까? (y/n) > ")).lower()

    if ans == "y":
        DC["invert"] = False
        print("   정상 → invert=False")
        link.send(f"GP,{center}")
        return

    if ans == "n":
        print("[방향 재확인] 이번엔 '오른쪽(pos_right)' 방향으로 움직입니다.")
        link.send(f"GP,{center}")
        await asyncio.sleep(0.3)
        target_right = center + (step if DC["pos_right"] > center else -step)
        link.send(f"GP,{target_right}")
        await asyncio.sleep(0.2)
        link.send(f"GP,{link.read_pos()}")  # 정지

        ans2 = (await ask(session, "오른쪽으로 돌았습니까? (y/n) > ")).lower()
        if ans2 == "y":
            DC["invert"] = True
            print("   좌우 반전 감지 → invert=True")
        else:
            print("   ⚠ 두 방향 명령에 모두 같은 쪽으로 회전 → DIR/배선 재확인 필요.")
            DC["invert"] = True
            print("   임시로 invert=True 적용 (하드웨어 점검 권장).")
        link.send(f"GP,{center}")
        return

    print("   입력 취소 → invert 기존값 유지")
    link.send(f"GP,{center}")


async def steer_test(session, link):
    amax = DC["steer_angle_max"]
    print(f"\n[조향 테스트] -{amax}~{amax} 입력 / c=중앙 / r=E-stop해제 / q=뒤로")
    print("(끝단 도달 시 E-stop / 입력 대기 중 무출력)")
    try:
        while True:
            s = await ask(session, "angle> ")
            link.set_print(False)
            link.clear_steer_guard()
            if s == "q":
                break
            if s == "r":
                resp = link.query("RST", "OK,RST", timeout=1.0)
                print("  E-stop 해제됨" if resp else "  해제 실패(40번/E-stop 확인)")
                continue
            if s == "c":
                link.set_steer_limits(DC["pos_left"], DC["pos_right"], margin=0)
                link.send(f"GP,{DC['pos_center']}")
                link.set_print(True, fmt="steer")
                continue
            if s.lstrip("-").isdigit():
                deg = int(s)
                if -amax <= deg <= amax:
                    target = angle_to_pos(deg, DC)
                    link.set_steer_limits(DC["pos_left"], DC["pos_right"], margin=0)
                    link.send(f"GP,{target}")
                    link.set_print(True, fmt="steer")
                else:
                    print(f"  범위 초과 (-{amax}~{amax})")
            else:
                print("  정수 / c / r / q")
    finally:
        link.send(f"GP,{DC['pos_center']}")
        link.set_print(False)
        link.clear_steer_guard()


# ---------------------- 메뉴 3: 리니어 ----------------------
async def menu_linear(session, link):
    print("[리니어모터] 미구현 단계입니다. 메인 메뉴로 돌아갑니다.")
    return


# ---------------------- 메뉴 4: E-stop ----------------------
async def menu_estop(session, link):
    link.send("E")
    print("[E-STOP] 비상정지 신호 전송. 전 모터 정지/래치.")
    a = await ask(session, "해제하려면 r 입력(NC 복구 필요) / 그 외 메뉴 복귀 > ")
    if a == "r":
        r = link.query("RST", "OK,RST", timeout=1.0)
        print("  해제됨" if r else "  해제 실패(E-stop 회로 확인)")
    return


# ---------------------- 메인 루프 ----------------------
async def main_async(link):
    session = PromptSession()
    while True:
        print("\n========== 메인 메뉴 ==========")
        print(" 1) 인휠모터")
        print(" 2) DC모터")
        print(" 3) 리니어모터")
        print(" 4) E-stop")
        print(" 그 외) 종료")
        print("===============================")
        sel = await ask(session, "menu> ")
        if sel == "1":
            await menu_inwheel(session, link)
        elif sel == "2":
            await menu_dc(session, link)
        elif sel == "3":
            await menu_linear(session, link)
        elif sel == "4":
            await menu_estop(session, link)
        else:
            print("종료합니다.")
            break


def main():
    port = resolve_port()
    if port is None:
        print("연결할 포트를 결정하지 못했습니다. 종료합니다.")
        return
    try:
        link = SerialLink(port, SERIAL_BAUD, SERIAL_TIMEOUT)
    except serial.SerialException as e:
        print(f"시리얼 연결 실패: {e}")
        return
    try:
        asyncio.run(main_async(link))
    except (KeyboardInterrupt, EOFError):
        print("\n중단됨.")
    finally:
        link.send("E")
        time.sleep(0.1)
        link.close()


if __name__ == "__main__":
    main()
