# -*- coding: utf-8 -*-
# ============================================================
#  y_can.py  -  y_can.ino가 보내는 CAN 텔레메트리 GUI 대시보드 / 기록 / 시각화
#
#  - y_can.ino는 두 갈래로 데이터를 낸다.
#      SD카드  : CAN에서 받는 필드 전량을 16열 CSV로 기록 (CANLOG.CSV 양식)
#      시리얼  : 대시보드용 5개 필드만 100ms 주기
#    이 스크립트는 시리얼을 받아 실시간 표시 + **같은 16열 양식**으로 CSV를 쓴다.
#    시리얼로 나오지 않는 열(온도/에러비트/브레이크/모드/컨택터/가속페달/lifeCounter)은
#    공란으로 남긴다. 덕분에 SD카드에서 뽑은 CSV와 PC가 쓴 CSV를 같은 코드로 읽는다.
#
#  - 시리얼 라인 형식 : <speed_div10>,<batt_v>,<batt_a>,<phase_a>,<gear>
#    ★ 첫 필드는 rpm을 10으로 나눈 정수 절사값이다. 실제 rpm = 값 x 10 ★
#    '#'로 시작하는 줄은 아두이노의 경고/이벤트 메시지이므로 건너뛴다.
#
#  - 배터리% / 전력량 같은 파생값은 CSV에 저장하지 않는다(양식을 지키려고).
#    시각화·요약할 때 파일을 읽고 나서 그 자리에서 계산한다.
#
#  - Windows + 아두이노 메가 1대 연결 전제. 포트는 자동으로 찾아 붙고,
#    실행 중 끊겨도 RECONNECT_INTERVAL_S(3초)마다 재연결을 시도한다.
#    아두이노가 없는 상태로 실행해도 GUI는 정상 동작한다.
# ============================================================

import collections
import csv
import os
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports


# ====================== 설정값 ======================

# ---- 배터리 스펙 (배터리 퍼센트 환산 기준) ----
# ★★ 반드시 실제 팩 스펙으로 검증할 것 ★★
# 아래는 "정격 58V = 16S 리튬이온"으로 가정한 값이다(16 x 3.6V = 57.6V).
# 팩이 다르면(예: LiFePO4 16S면 만충 58.4V / 방전종지 40V) 이 두 값만 고치면 된다.
BATT_V_FULL  = 67.2   # 만충전 전압 (16 x 4.20V)
BATT_V_EMPTY = 48.0   # 방전종지 전압 (16 x 3.00V)

# ---- 시리얼 ----
BAUD                  = 115200
SERIAL_TIMEOUT_S      = 1.0    # readline 타임아웃
RECONNECT_INTERVAL_S  = 3.0    # 연결 실패/끊김 시 재시도 간격
DATA_WATCHDOG_S       = 5.0    # 이 시간 동안 유효 데이터가 없으면 끊긴 것으로 보고 재연결

# ---- 기록 ----
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))  # csvdata_NNN 폴더가 생기는 위치
FOLDER_PREFIX = "csvdata_"
CSV_NAME      = "canlog.csv"
SUMMARY_NAME  = "summary.txt"

# CANLOG.CSV 16열 양식. 아두이노 SD 출력과 글자 하나까지 같아야 한다.
CSV_HEADER = ["millis", "batteryVoltage_V", "batteryCurrent_A", "phaseCurrent_A",
              "motorSpeed_rpm", "controllerTemp_C", "motorTemp_C", "accelPct",
              "gear", "brake", "opMode", "dcContactor",
              "err1_hex", "err2_hex", "err3_hex", "lifeCounter"]

# 이 열들이 없으면 CANLOG 양식이 아니라고 본다 (나머지는 공란이어도 읽는다)
REQUIRED_COLS = ("millis", "batteryVoltage_V", "batteryCurrent_A",
                 "phaseCurrent_A", "motorSpeed_rpm")

# ---- 연산 ----
TICK_MS          = 100   # 큐 배출 + 화면 갱신 주기
PCT_SMOOTH_N     = 20    # 배터리% 표시용 이동평균 표본수 (2초). CSV에는 원시 전압이 들어간다
DT_GAP_MAX_LIVE  = 1.0   # 실시간 기록(100ms 간격)에서 이보다 긴 간격은 통신 끊김으로 보고 적분 제외
DT_GAP_MAX_FILE  = 3.0   # 파일 분석용. 아두이노 SD는 500ms 간격이라 여유를 더 둔다

GEAR_NAMES = {0: "NO", 1: "R", 2: "N", 3: "D1", 4: "D2", 5: "D3", 6: "S", 7: "P"}


# ====================== 포트 자동 탐지 ======================

KNOWN_MEGA_VIDPID = {(0x2341, 0x0042), (0x2341, 0x0010), (0x2341, 0x003F), (0x2A03, 0x0042)}
CANDIDATE_VIDS = {0x1A86, 0x0403, 0x10C4, 0x2341, 0x2A03}  # CH340 / FTDI / CP210x / Arduino


def find_arduino_port():
    """아두이노로 보이는 COM 포트를 점수순으로 1개 골라 반환. 없으면 None.

    '아두이노면 닥치고 연결' 전제라 사용자에게 묻지 않는다. 다만 블루투스
    가상 COM 포트는 아두이노와 무관하게 흔히 잡히므로 후보에서 제외한다.
    """
    best, best_score = None, 0
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "bluetooth" in desc or "bluetooth" in hwid:
            continue

        if p.vid is not None and (p.vid, p.pid) in KNOWN_MEGA_VIDPID:
            score = 100
        elif "mega" in desc:
            score = 90
        elif "arduino" in desc:
            score = 80
        elif p.vid in CANDIDATE_VIDS:
            score = 70
        elif "usb-serial" in desc or "usb serial" in desc:
            score = 50
        else:
            score = 0

        if score > best_score:
            best, best_score = p, score
    return best.device if best else None


def parse_line(line):
    """'654,58.3,12.5,45.2,3' -> (speed_div10, batt_v, batt_a, phase_a, gear)"""
    parts = line.split(",")
    if len(parts) != 5:
        return None
    try:
        return (int(parts[0]), float(parts[1]), float(parts[2]),
                float(parts[3]), int(parts[4]))
    except ValueError:
        return None


def batt_pct(v):
    """전압 -> 배터리 잔량(%). 부하 중엔 내부저항 전압강하로 실제보다 낮게 나오는 근사치다."""
    if BATT_V_FULL <= BATT_V_EMPTY:
        return 0.0
    pct = (v - BATT_V_EMPTY) / (BATT_V_FULL - BATT_V_EMPTY) * 100.0
    return max(0.0, min(100.0, pct))


# ====================== 시리얼 수신 스레드 ======================

class SerialWorker(threading.Thread):
    """백그라운드로 시리얼을 읽어 큐에 넣는다. 연결 실패/끊김은 스스로 재시도한다."""

    def __init__(self, data_q, status_q):
        super().__init__(daemon=True)
        self.data_q = data_q
        self.status_q = status_q
        self._stop_evt = threading.Event()

    def stop(self):
        self._stop_evt.set()

    def _status(self, text):
        self.status_q.put(text)

    def run(self):
        while not self._stop_evt.is_set():
            port = find_arduino_port()
            if port is None:
                self._status("아두이노 탐색 중... (연결되지 않음)")
                self._stop_evt.wait(RECONNECT_INTERVAL_S)
                continue

            try:
                ser = serial.Serial(port, BAUD, timeout=SERIAL_TIMEOUT_S)
            except Exception as e:
                self._status(f"{port} 연결 실패 ({e.__class__.__name__}) - {RECONNECT_INTERVAL_S:.0f}초 후 재시도")
                self._stop_evt.wait(RECONNECT_INTERVAL_S)
                continue

            self._status(f"{port} 연결됨 - 데이터 대기 중")
            last_data = time.monotonic()
            try:
                ser.reset_input_buffer()
                while not self._stop_evt.is_set():
                    raw = ser.readline()
                    now = time.monotonic()
                    if raw:
                        line = raw.decode("utf-8", errors="ignore").strip()
                        # '#' 줄은 아두이노쪽 경고/이벤트(MCP2515 실패, 기어·에러 변화 등)
                        if line and not line.startswith("#"):
                            sample = parse_line(line)
                            if sample is not None:
                                if now - last_data > 1.0:
                                    self._status(f"{port} 수신 중")
                                last_data = now
                                self.data_q.put(sample)
                    # USB가 뽑혀도 예외 없이 타임아웃만 반복되는 경우가 있어 워치독을 둔다
                    if now - last_data > DATA_WATCHDOG_S:
                        self._status(f"{port} 무응답 {DATA_WATCHDOG_S:.0f}초 - 재연결")
                        break
            except Exception:
                self._status(f"{port} 연결 끊김 - {RECONNECT_INTERVAL_S:.0f}초 후 재연결")
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

            if not self._stop_evt.is_set():
                self._stop_evt.wait(RECONNECT_INTERVAL_S)


# ====================== 기록 ======================

def next_folder_path(base_dir):
    """csvdata_001 ... csvdata_999 -> csvdata_000 -> csvdata_001(덮어쓰기) 순환."""
    pat = re.compile(r"^" + re.escape(FOLDER_PREFIX) + r"(\d{3})$")
    found = []
    try:
        for name in os.listdir(base_dir):
            m = pat.match(name)
            full = os.path.join(base_dir, name)
            if m and os.path.isdir(full):
                found.append((os.path.getmtime(full), int(m.group(1))))
    except OSError:
        pass

    if not found:
        n = 1
    else:
        found.sort()                      # 번호가 순환하므로 '가장 최근에 쓴 폴더' 기준으로 잇는다
        n = (found[-1][1] + 1) % 1000
    return os.path.join(base_dir, f"{FOLDER_PREFIX}{n:03d}")


class Recorder:
    """CANLOG 16열 양식으로 CSV를 쓰고, 표시용 누적값(전력량 등)을 함께 계산한다.

    파생값(배터리%/전력량)은 CSV에 넣지 않는다 — 아두이노 SD 출력과 양식을
    똑같이 유지해야 하므로. 나중에 읽을 때 compute_derived()로 다시 구한다.
    """

    def __init__(self, folder):
        self.folder = folder
        os.makedirs(folder, exist_ok=True)
        self.csv_path = os.path.join(folder, CSV_NAME)
        self._f = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(CSV_HEADER)

        self.t0 = time.monotonic()
        self.wall_start = time.time()
        self._last_t = None

        self.n = 0
        self.energy_wh = 0.0       # 순 소비량 (회생분이 차감된 값)
        self.discharge_wh = 0.0
        self.regen_wh = 0.0
        self.start_v = None
        self.last_v = None
        self.start_pct = None
        self.last_pct = None
        self.peak_rpm = 0
        self.peak_phase_a = 0.0
        self.peak_batt_a = 0.0

    def add(self, speed_div10, batt_v, batt_a, phase_a, gear):
        now = time.monotonic()
        t = now - self.t0
        dt = 0.0 if self._last_t is None else (now - self._last_t)
        if dt > DT_GAP_MAX_LIVE:   # 끊겼다 붙은 구간을 전력량에 통째로 넣지 않는다
            dt = 0.0
        self._last_t = now

        rpm = speed_div10 * 10
        power_w = batt_v * batt_a          # 전력량은 버스(배터리) 전류 기준. 상전류로는 계산 불가
        wh = power_w * dt / 3600.0
        self.energy_wh += wh
        if wh >= 0.0:
            self.discharge_wh += wh
        else:
            self.regen_wh += -wh

        pct = batt_pct(batt_v)

        self.n += 1
        if self.start_v is None:
            self.start_v, self.start_pct = batt_v, pct
        self.last_v, self.last_pct = batt_v, pct
        self.peak_rpm = max(self.peak_rpm, rpm)
        self.peak_phase_a = max(self.peak_phase_a, abs(phase_a))
        self.peak_batt_a = max(self.peak_batt_a, abs(batt_a))

        # 시리얼로 나오지 않는 열은 공란으로 둔다
        row = {c: "" for c in CSV_HEADER}
        row["millis"]           = int(round(t * 1000))   # 기록 시작 기준 경과 ms
        row["batteryVoltage_V"] = f"{batt_v:.1f}"
        row["batteryCurrent_A"] = f"{batt_a:.1f}"
        row["phaseCurrent_A"]   = f"{phase_a:.1f}"
        row["motorSpeed_rpm"]   = rpm
        row["gear"]             = GEAR_NAMES.get(gear, "UNKNOWN")
        self._w.writerow([row[c] for c in CSV_HEADER])
        self._f.flush()   # 전원이 갑자기 끊겨도 마지막 줄까지 남도록 (SD 로거와 같은 이유)

    def elapsed(self):
        return time.monotonic() - self.t0

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass
        self._write_summary()

    def _write_summary(self):
        def fmt(v, nd=1):
            return "-" if v is None else f"{round(v, nd)}"

        lines = [
            "y_can 기록 요약",
            "=" * 46,
            f"CSV         : {os.path.join(os.path.basename(self.folder), CSV_NAME)}",
            f"시작        : {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.wall_start))}",
            f"기록 시간   : {self.elapsed():.1f} 초",
            f"샘플 수     : {self.n}",
            "",
            "--- 배터리 ---",
            f"시작 전압   : {fmt(self.start_v)} V",
            f"종료 전압   : {fmt(self.last_v)} V",
            f"시작 잔량   : {fmt(self.start_pct)} %",
            f"종료 잔량   : {fmt(self.last_pct)} %",
            f"(환산 기준  : 만충 {BATT_V_FULL} V / 방전종지 {BATT_V_EMPTY} V)",
            "",
            "--- 전력량 (배터리전압 x 버스전류 적분) ---",
            f"총 사용량   : {self.energy_wh:.2f} Wh",
            f"  방전      : {self.discharge_wh:.2f} Wh",
            f"  회생      : {self.regen_wh:.2f} Wh",
            "",
            "--- 피크 ---",
            f"최대 RPM    : {self.peak_rpm}",
            f"최대 상전류 : {self.peak_phase_a:.1f} A",
            f"최대 버스전류: {self.peak_batt_a:.1f} A",
            "",
            "※ CSV는 CANLOG 16열 양식이며, 시리얼로 나오지 않는 열(온도/에러비트/",
            "   브레이크/모드/컨택터/가속페달/lifeCounter)은 공란이다.",
            "   SD카드에서 뽑은 CSV에는 그 열들도 채워져 있다.",
        ]
        try:
            with open(os.path.join(self.folder, SUMMARY_NAME), "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
        except OSError:
            pass


# ====================== CSV 읽기 + 파생값 계산 ======================

def load_canlog(path):
    """CANLOG 16열 CSV를 읽어 (rows, 오류메시지) 반환.

    아두이노 SD 출력과 PC 기록 출력이 같은 양식이라 둘 다 그대로 처리된다.
    PC 출력의 공란 열은 애초에 읽지 않으므로 문제되지 않는다.
    """
    if not path or not os.path.isfile(path):
        return None, "파일을 찾을 수 없습니다."

    rows = []
    try:
        with open(path, "r", newline="", encoding="utf-8", errors="replace") as f:
            reader = csv.DictReader(f)
            names = reader.fieldnames or []
            missing = [c for c in REQUIRED_COLS if c not in names]
            if missing:
                return None, "CANLOG 양식이 아닙니다.\n없는 열: " + ", ".join(missing)

            for raw in reader:
                try:
                    ms   = float(raw["millis"])
                    volt = float(raw["batteryVoltage_V"])
                    amp  = float(raw["batteryCurrent_A"])
                    ph   = float(raw["phaseCurrent_A"])
                    rpm  = float(raw["motorSpeed_rpm"])
                except (TypeError, ValueError):
                    continue    # 공란이거나, 기록 중 전원이 끊겨 잘린 마지막 줄
                rows.append({"millis": ms, "volt": volt, "amp": amp,
                             "phase": ph, "rpm": rpm,
                             "gear": (raw.get("gear") or "").strip()})
    except OSError as e:
        return None, f"파일을 열 수 없습니다.\n{e}"

    if not rows:
        return None, "읽을 수 있는 데이터 줄이 없습니다."
    return rows, None


def compute_derived(rows):
    """millis 간격으로 전력량을 적분하고 각 행에 t_s/power_w/energy_wh/batt_pct를 채운다.

    파생값을 CSV에 저장하지 않는 대신 읽을 때마다 여기서 만든다.
    시간축은 첫 줄을 0으로 맞춘다 — 아두이노 millis는 부팅 후 경과라 1048처럼 시작한다.
    """
    t0 = rows[0]["millis"]
    energy = discharge = regen = 0.0
    peak_rpm = peak_phase = peak_amp = 0.0
    prev_ms = None

    for r in rows:
        ms = r["millis"]
        dt = 0.0 if prev_ms is None else (ms - prev_ms) / 1000.0
        if dt < 0.0 or dt > DT_GAP_MAX_FILE:
            dt = 0.0      # 음수 = 로그가 이어붙어 millis가 되돌아간 경우
        prev_ms = ms

        p = r["volt"] * r["amp"]
        wh = p * dt / 3600.0
        energy += wh
        if wh >= 0.0:
            discharge += wh
        else:
            regen += -wh

        r["t_s"]       = (ms - t0) / 1000.0
        r["power_w"]   = p
        r["energy_wh"] = energy
        r["batt_pct"]  = batt_pct(r["volt"])

        peak_rpm   = max(peak_rpm, r["rpm"])
        peak_phase = max(peak_phase, abs(r["phase"]))
        peak_amp   = max(peak_amp, abs(r["amp"]))

    return {
        "pct": rows[-1]["batt_pct"],
        "energy_wh": energy,
        "discharge_wh": discharge,
        "regen_wh": regen,
        "duration": rows[-1]["t_s"],
        "n": len(rows),
        "peak_rpm": peak_rpm,
        "peak_phase_a": peak_phase,
        "peak_batt_a": peak_amp,
    }


# ====================== GUI ======================

class App:
    def __init__(self, root):
        self.root = root
        root.title("y_can - CAN 텔레메트리 대시보드")
        root.geometry("940x430")
        root.minsize(820, 400)

        self.data_q = queue.Queue()
        self.status_q = queue.Queue()
        self.worker = SerialWorker(self.data_q, self.status_q)

        self.recorder = None        # 기록 중이면 Recorder 인스턴스
        self.record_csv = None      # 이번 세션에서 기록한(하는) CSV 경로
        self.loaded_csv = None      # 시작할 때 불러온 이전 CSV 경로
        self.pct_buf = collections.deque(maxlen=PCT_SMOOTH_N)

        self._build_ui()
        self.worker.start()
        root.protocol("WM_DELETE_WINDOW", self.on_quit)
        root.after(TICK_MS, self._tick)
        root.after(250, self._startup_file_prompt)   # 창이 그려진 뒤에 띄운다

    # ---------- UI 구성 ----------

    def _build_ui(self):
        pad = 10
        head_font = ("Malgun Gothic", 12, "bold")

        ttk.Label(self.root, text="실시간 대시보드", font=head_font).pack(anchor="w", padx=pad, pady=(pad, 2))
        dash = ttk.Frame(self.root)
        dash.pack(fill=tk.X, padx=pad)
        self.v_speed = self._cell(dash, 0, "모터속도", "rpm")
        self.v_volt  = self._cell(dash, 1, "배터리 전압", "V")
        self.v_amp   = self._cell(dash, 2, "배터리 전류", "A")
        self.v_phase = self._cell(dash, 3, "상전류", "A")

        ttk.Label(self.root, text="상태", font=head_font).pack(anchor="w", padx=pad, pady=(pad, 2))
        stat = ttk.Frame(self.root)
        stat.pack(fill=tk.X, padx=pad)
        self.v_pct    = self._cell(stat, 0, "[1] 배터리 잔량", "%")
        self.v_energy = self._cell(stat, 1, "[2] 총 사용 전력량", "Wh")
        self.v_slot3  = self._cell(stat, 2, "[3]", "")
        self.v_slot4  = self._cell(stat, 3, "[4]", "")

        btns = ttk.Frame(self.root)
        btns.pack(fill=tk.X, padx=pad, pady=(pad + 4, 4))
        self.btn_rec = ttk.Button(btns, text="기록 시작", width=18, command=self.on_toggle_record)
        self.btn_rec.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btns, text="시각화", width=14, command=self.on_visualize).pack(side=tk.LEFT, padx=6)
        ttk.Button(btns, text="프로그램 종료", width=14, command=self.on_quit).pack(side=tk.LEFT, padx=6)

        self.conn_status = tk.StringVar(value="아두이노 탐색 중...")
        self.rec_status = tk.StringVar(value="대기 중")
        bar = ttk.Frame(self.root)
        bar.pack(fill=tk.X, side=tk.BOTTOM, padx=pad, pady=(0, 6))
        ttk.Label(bar, textvariable=self.conn_status, foreground="#555").pack(side=tk.LEFT)
        ttk.Label(bar, textvariable=self.rec_status, foreground="#555").pack(side=tk.RIGHT)

    def _cell(self, parent, col, title, unit):
        var = tk.StringVar(value="—")
        text = f"{title} ({unit})" if unit else title
        box = ttk.LabelFrame(parent, text=text)
        box.grid(row=0, column=col, padx=4, pady=2, sticky="nsew")
        parent.columnconfigure(col, weight=1, uniform="cells")
        ttk.Label(box, textvariable=var, font=("Consolas", 19, "bold"),
                  anchor="center").pack(fill=tk.BOTH, expand=True, padx=6, pady=8)
        return var

    # ---------- 시작 시 이전 CSV 불러오기 ----------

    def _startup_file_prompt(self):
        path = filedialog.askopenfilename(
            title="이전 데이터 CSV 파일 선택 (취소하면 비운 채로 시작)",
            initialdir=BASE_DIR,
            filetypes=[("CANLOG CSV", "*.csv"), ("모든 파일", "*.*")])
        if not path:
            self.rec_status.set("대기 중 (이전 데이터 없음)")
            return

        rows, err = load_canlog(path)
        if rows is None:
            messagebox.showwarning("불러오기", f"{os.path.basename(path)}\n\n{err}")
            self.rec_status.set("대기 중 (불러오기 실패)")
            return

        summary = compute_derived(rows)
        self.loaded_csv = path
        self._show_summary(summary)
        self.rec_status.set(f"불러옴: {os.path.basename(path)} "
                            f"({summary['n']}샘플 / {summary['duration']:.0f}초)")

    def _show_summary(self, s):
        self.v_pct.set(f"{s['pct']:.1f}")
        self.v_energy.set(f"{s['energy_wh']:.1f}")

    # ---------- 주기 갱신 ----------

    def _tick(self):
        while True:
            try:
                self.conn_status.set(self.status_q.get_nowait())
            except queue.Empty:
                break

        latest = None
        while True:
            try:
                sample = self.data_q.get_nowait()
            except queue.Empty:
                break
            latest = sample
            if self.recorder is not None:
                self.recorder.add(*sample)

        if latest is not None:
            self._update_dashboard(latest)

        if self.recorder is not None:
            self._update_live_status()

        self.root.after(TICK_MS, self._tick)

    def _update_dashboard(self, sample):
        speed_div10, batt_v, batt_a, phase_a, gear = sample
        self.v_speed.set(f"{speed_div10 * 10}")     # 아두이노가 10으로 나눠 보내므로 되돌린다
        self.v_volt.set(f"{batt_v:.1f}")
        self.v_amp.set(f"{batt_a:.1f}")
        self.v_phase.set(f"{phase_a:.1f}")
        self.pct_buf.append(batt_pct(batt_v))

    def _update_live_status(self):
        if self.pct_buf:
            self.v_pct.set(f"{sum(self.pct_buf) / len(self.pct_buf):.1f}")
        self.v_energy.set(f"{self.recorder.energy_wh:.1f}")
        self.rec_status.set(f"● 기록 중 {os.path.basename(os.path.dirname(self.record_csv))} "
                            f"| {self.recorder.elapsed():.0f}초 | {self.recorder.n}샘플")

    # ---------- 버튼 ----------

    def on_toggle_record(self):
        if self.recorder is None:
            self._start_record()
        else:
            self._stop_record()

    def _start_record(self):
        folder = next_folder_path(BASE_DIR)
        try:
            recorder = Recorder(folder)
        except OSError as e:
            messagebox.showerror("기록 시작 실패", f"{folder}\n{e}")
            return

        self.recorder = recorder
        self.record_csv = recorder.csv_path
        self.pct_buf.clear()

        # 이전에 불러와 띄워둔 값은 지운다 (디스크의 데이터는 그대로 남는다)
        self.v_pct.set("—")
        self.v_energy.set("—")

        self.btn_rec.config(text="기록 중지")
        self.rec_status.set(f"● 기록 중 {os.path.basename(folder)}")

    def _stop_record(self):
        rec = self.recorder
        self.recorder = None
        rec.close()
        self.btn_rec.config(text="기록 시작")
        tag = os.path.basename(rec.folder)

        if rec.n == 0:
            self.v_pct.set("—")
            self.v_energy.set("—")
            self.rec_status.set(f"기록 종료 - 수신 데이터 없음 ({tag})")
            return

        self.v_pct.set(f"{rec.last_pct:.1f}")
        self.v_energy.set(f"{rec.energy_wh:.1f}")
        self.rec_status.set(f"기록 종료: {tag} | {rec.elapsed():.0f}초 | {rec.n}샘플")

    def _viz_target(self):
        """이번 세션 기록이 우선, 없으면 시작할 때 불러온 CSV."""
        return self.record_csv or self.loaded_csv

    def on_visualize(self):
        path = self._viz_target()
        if path is None:
            messagebox.showinfo("시각화", "표시할 데이터가 없습니다.\n"
                                          "기록을 하거나, 프로그램을 다시 켜서 이전 CSV를 선택하세요.")
            return

        # 파일을 먼저 읽고 나서 파생값을 계산한다 (CSV에는 원시 열만 들어 있다)
        rows, err = load_canlog(path)
        if rows is None:
            messagebox.showerror("시각화", f"{os.path.basename(path)}\n\n{err}")
            return
        if len(rows) < 2:
            messagebox.showinfo("시각화", "그래프를 그리기에 데이터가 부족합니다.")
            return
        compute_derived(rows)

        try:
            self._draw_all(path, rows)
        except ImportError:
            messagebox.showerror("시각화", "matplotlib이 설치되어 있지 않습니다.\n"
                                           "pip install matplotlib")
        except Exception as e:
            messagebox.showerror("시각화", f"그래프 생성 중 오류\n{e}")

    def on_quit(self):
        if self.recorder is not None:
            # 기록 중 종료 = 데이터 유실이므로, 정상 마감(요약 파일까지)하고 닫는다
            self._stop_record()
        self.worker.stop()
        self.root.destroy()

    # ---------- 시각화 ----------

    def _draw_all(self, path, rows):
        import matplotlib
        matplotlib.use("TkAgg")
        from matplotlib import font_manager

        # 한글 폰트를 지정하지 않으면 제목/축이 두부(□)로 깨진다
        installed = {f.name for f in font_manager.fontManager.ttflist}
        for name in ("Malgun Gothic", "NanumGothic", "AppleGothic", "Gulim"):
            if name in installed:
                matplotlib.rcParams["font.family"] = name
                break
        matplotlib.rcParams["axes.unicode_minus"] = False   # 유니코드 마이너스가 없으면 깨짐

        tag = os.path.basename(path)
        t       = [r["t_s"] for r in rows]
        pct     = [r["batt_pct"] for r in rows]
        batt_a  = [r["amp"] for r in rows]
        phase_a = [r["phase"] for r in rows]
        power   = [r["power_w"] for r in rows]
        rpm     = [r["rpm"] for r in rows]

        def plot_pct(ax):
            ax.plot(t, pct, color="tab:green")
            ax.set_ylim(0, 100)
            ax.set_xlabel("시간 (초)")
            ax.set_ylabel("배터리 잔량 (%)")
            ax.set_title(f"시간별 배터리 퍼센트  [{tag}]")
            ax.grid(True, alpha=0.3)

        def plot_currents(ax):
            ax.plot(t, batt_a, color="tab:blue", label="버스전류 (배터리)")
            ax.plot(t, phase_a, color="tab:red", label="상전류 (모터)")
            ax.axhline(0, color="#999", lw=0.8)
            ax.set_xlabel("시간 (초)")
            ax.set_ylabel("전류 (A)")
            ax.set_title(f"실시간 버스전류와 상전류  [{tag}]")
            ax.legend()
            ax.grid(True, alpha=0.3)

        def plot_power(ax):
            ax.plot(t, power, color="tab:orange")
            ax.axhline(0, color="#999", lw=0.8)
            ax.set_xlabel("시간 (초)")
            ax.set_ylabel("소모전력 (W)")
            ax.set_title(f"실시간 소모전력 (배터리전압 x 버스전류)  [{tag}]")
            ax.grid(True, alpha=0.3)

        def plot_rpm(ax):
            ax.plot(t, rpm, color="tab:purple")
            ax.set_xlabel("시간 (초)")
            ax.set_ylabel("모터 회전수 (rpm)")
            ax.set_title(f"시간별 RPM  [{tag}]")
            ax.grid(True, alpha=0.3)

        for i, (title, fn) in enumerate([
            ("시간별 배터리 퍼센트", plot_pct),
            ("실시간 버스전류와 상전류", plot_currents),
            ("실시간 소모전력", plot_power),
            ("시간별 RPM", plot_rpm),
        ]):
            self._plot_window(f"{title} - {tag}", fn, offset=i * 30)

    def _plot_window(self, title, draw_fn, offset=0):
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

        # pyplot을 쓰면 tkinter mainloop와 이벤트 루프가 얽히므로 Toplevel에 직접 붙인다
        win = tk.Toplevel(self.root)
        win.title(title)
        win.geometry(f"860x480+{80 + offset}+{60 + offset}")

        fig = Figure(figsize=(8.4, 4.6), dpi=100)
        draw_fn(fig.add_subplot(111))
        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.draw()
        NavigationToolbar2Tk(canvas, win).update()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")   # Windows 기본 테마
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
