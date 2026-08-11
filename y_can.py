# -*- coding: utf-8 -*-
# ============================================================
#  y_can.py  -  y_can.ino가 SD카드에 남긴 CAN 로그 CSV를 불러와 요약/시각화
#
#  - 이 스크립트는 '파일을 불러와 보는' 기능만 한다. 시리얼 실시간 수신은 없다.
#    아두이노를 PC에 연결할 필요도, 아두이노가 켜져 있을 필요도 없다.
#
#  - 읽는 파일 : y_can.ino가 SD카드에 쓴 CANLOG 16열 CSV.
#    부팅마다 1.csv, 2.csv, 3.csv ... 로 번호가 올라간다(아두이노에 시계가 없어
#    날짜 대신 번호로 구분한다). SD카드에서 그 파일을 PC로 옮겨 열면 된다.
#
#  - 16열 양식 (아두이노 SD 출력과 글자 하나까지 같다) :
#      millis, batteryVoltage_V, batteryCurrent_A, phaseCurrent_A, motorSpeed_rpm,
#      controllerTemp_C, motorTemp_C, accelPct, gear, brake, opMode, dcContactor,
#      err1_hex, err2_hex, err3_hex, lifeCounter
#
#  - 배터리% / 전력량 / 평균온도 같은 파생값은 CSV에 없다. 파일을 읽고 나서
#    compute_derived()가 그 자리에서 계산한다.
#
#  - 온도(controllerTemp_C / motorTemp_C)는 SD 로그에만 들어 있다. 예전에
#    PC가 시리얼로 받아 쓴 CSV에는 그 열이 공란이므로, 그런 파일을 열면
#    온도 표시와 온도 그래프는 자동으로 비활성된다.
# ============================================================

import csv
import os
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


# ====================== 설정값 ======================

# ---- 배터리 스펙 ([1] 배터리 퍼센트 환산에만 쓰인다) ----
# 실제 팩 : 52V 80Ah, 14INR21/70, 14S16P, Samsung SDI INR21700-50S
#
# 58V를 100%, 40V를 0%로 잡는다(실측 만충 기준).
#   58.0V / 14S = 4.14 V/셀  - 충전기가 4.2V까지 채우지 않는 실측 만충
#   40.0V / 14S = 2.86 V/셀  - 셀 데이터시트 방전종지(2.5V)보다는 위, 실용 하한
#
# ★ 전압-잔량을 직선으로 잇는 근사다. 리튬 방전곡선은 중간이 평평해 실제로는
#   50% 부근에서 실물보다 낮게, 양 끝에서 높게 나온다. 게다가 주행 중에는
#   내부저항 전압강하 때문에 더 낮게 찍힌다 - 정지 상태 값이 가장 믿을 만하다.
BATT_V_FULL  = 58.0
BATT_V_EMPTY = 40.0

# 파일 대화상자가 처음 열어 보여줄 위치
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 이 열들이 없으면 CANLOG 양식이 아니라고 본다
REQUIRED_COLS = ("millis", "batteryVoltage_V", "batteryCurrent_A",
                 "phaseCurrent_A", "motorSpeed_rpm")

# 온도 열. 없거나 공란이어도 파일은 정상으로 읽고, 온도 기능만 꺼진다
COL_MOTOR_TEMP = "motorTemp_C"
COL_CTRL_TEMP  = "controllerTemp_C"

# 아두이노 SD는 500ms 간격으로 쓴다. 이보다 긴 간격은 전원이 끊겼다 붙은
# 구간으로 보고 전력량 적분에서 뺀다
DT_GAP_MAX_FILE = 3.0


def batt_pct(v):
    """전압 -> 배터리 잔량(%). 부하 중엔 내부저항 전압강하로 실제보다 낮게 나오는 근사치다."""
    if BATT_V_FULL <= BATT_V_EMPTY:
        return 0.0
    pct = (v - BATT_V_EMPTY) / (BATT_V_FULL - BATT_V_EMPTY) * 100.0
    return max(0.0, min(100.0, pct))


def _opt_float(raw, key):
    """공란/누락/비숫자면 None. 온도처럼 '있을 수도 없을 수도' 있는 열에 쓴다."""
    text = (raw.get(key) or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


# ====================== CSV 읽기 + 파생값 계산 ======================

def load_canlog(path):
    """CANLOG 16열 CSV를 읽어 (rows, 오류메시지) 반환."""
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
                             "motor_temp": _opt_float(raw, COL_MOTOR_TEMP),
                             "ctrl_temp": _opt_float(raw, COL_CTRL_TEMP),
                             "gear": (raw.get("gear") or "").strip()})
    except OSError as e:
        return None, f"파일을 열 수 없습니다.\n{e}"

    if not rows:
        return None, "읽을 수 있는 데이터 줄이 없습니다."
    return rows, None


def _mean(values):
    """None을 뺀 평균. 남은 표본이 없으면 None."""
    nums = [v for v in values if v is not None]
    return sum(nums) / len(nums) if nums else None


def compute_derived(rows):
    """millis 간격으로 전력량을 적분하고 각 행에 t_s/power_w/energy_wh/batt_pct를 채운다.

    파생값은 CSV에 없으므로 읽을 때마다 여기서 만든다.
    시간축은 첫 줄을 0으로 맞춘다 - 아두이노 millis는 부팅 후 경과라 1048처럼 시작한다.
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
        # 온도는 SD 로그에만 있다. 없으면 None이고 화면에서 '—'로 표시된다
        "motor_temp_avg": _mean([r["motor_temp"] for r in rows]),
        "ctrl_temp_avg": _mean([r["ctrl_temp"] for r in rows]),
    }


# ====================== GUI ======================

class App:
    def __init__(self, root):
        self.root = root
        root.title("y_can - CAN 로그 뷰어")
        root.geometry("940x300")
        root.minsize(820, 280)

        self.loaded_csv = None      # 현재 불러온 CSV 경로

        self._build_ui()
        root.protocol("WM_DELETE_WINDOW", self.on_quit)
        root.after(250, self.on_load)   # 창이 그려진 뒤에 파일 선택을 띄운다

    # ---------- UI 구성 ----------

    def _build_ui(self):
        pad = 10
        head_font = ("Malgun Gothic", 12, "bold")

        ttk.Label(self.root, text="불러온 로그 요약", font=head_font).pack(
            anchor="w", padx=pad, pady=(pad, 2))
        stat = ttk.Frame(self.root)
        stat.pack(fill=tk.X, padx=pad)
        self.v_pct        = self._cell(stat, 0, "[1] 배터리 잔량", "%")
        self.v_energy     = self._cell(stat, 1, "[2] 총 사용 전력량", "Wh")
        self.v_motor_temp = self._cell(stat, 2, "[3] 모터 평균온도", "°C")
        self.v_ctrl_temp  = self._cell(stat, 3, "[4] 컨트롤러 평균온도", "°C")

        btns = ttk.Frame(self.root)
        btns.pack(fill=tk.X, padx=pad, pady=(pad + 4, 4))
        ttk.Button(btns, text="불러오기", width=18, command=self.on_load).pack(
            side=tk.LEFT, padx=(0, 6))
        ttk.Button(btns, text="시각화", width=14, command=self.on_visualize).pack(
            side=tk.LEFT, padx=6)
        ttk.Button(btns, text="프로그램 종료", width=14, command=self.on_quit).pack(
            side=tk.LEFT, padx=6)

        self.status = tk.StringVar(value="불러온 파일 없음")
        bar = ttk.Frame(self.root)
        bar.pack(fill=tk.X, side=tk.BOTTOM, padx=pad, pady=(0, 6))
        ttk.Label(bar, textvariable=self.status, foreground="#555").pack(side=tk.LEFT)

    def _cell(self, parent, col, title, unit):
        var = tk.StringVar(value="—")
        text = f"{title} ({unit})" if unit else title
        box = ttk.LabelFrame(parent, text=text)
        box.grid(row=0, column=col, padx=4, pady=2, sticky="nsew")
        parent.columnconfigure(col, weight=1, uniform="cells")
        ttk.Label(box, textvariable=var, font=("Consolas", 19, "bold"),
                  anchor="center").pack(fill=tk.BOTH, expand=True, padx=6, pady=8)
        return var

    # ---------- 불러오기 ----------

    def on_load(self):
        path = filedialog.askopenfilename(
            title="CAN 로그 CSV 선택 (SD카드의 1.csv, 2.csv ...)",
            initialdir=BASE_DIR,
            filetypes=[("CSV 파일", "*.csv")],
            defaultextension=".csv")
        if not path:
            if self.loaded_csv is None:
                self.status.set("불러온 파일 없음")
            return

        rows, err = load_canlog(path)
        if rows is None:
            messagebox.showwarning("불러오기", f"{os.path.basename(path)}\n\n{err}")
            if self.loaded_csv is None:
                self.status.set("불러오기 실패")
            return

        summary = compute_derived(rows)
        self.loaded_csv = path
        self._show_summary(summary)

        note = "" if summary["motor_temp_avg"] is not None or \
                     summary["ctrl_temp_avg"] is not None else "  (온도 열 없음)"
        self.status.set(f"불러옴: {os.path.basename(path)} "
                        f"({summary['n']}샘플 / {summary['duration']:.0f}초){note}")

    def _show_summary(self, s):
        def temp(v):
            return "—" if v is None else f"{v:.1f}"

        self.v_pct.set(f"{s['pct']:.1f}")
        self.v_energy.set(f"{s['energy_wh']:.1f}")
        self.v_motor_temp.set(temp(s["motor_temp_avg"]))
        self.v_ctrl_temp.set(temp(s["ctrl_temp_avg"]))

    # ---------- 시각화 ----------

    def on_visualize(self):
        if self.loaded_csv is None:
            messagebox.showinfo("시각화", "표시할 데이터가 없습니다.\n"
                                          "먼저 '불러오기'로 CSV를 선택하세요.")
            return

        # 파일을 다시 읽고 나서 파생값을 계산한다 (CSV에는 원시 열만 들어 있다)
        rows, err = load_canlog(self.loaded_csv)
        if rows is None:
            messagebox.showerror("시각화", f"{os.path.basename(self.loaded_csv)}\n\n{err}")
            return
        if len(rows) < 2:
            messagebox.showinfo("시각화", "그래프를 그리기에 데이터가 부족합니다.")
            return
        compute_derived(rows)

        try:
            self._draw_all(self.loaded_csv, rows)
        except ImportError:
            messagebox.showerror("시각화", "matplotlib이 설치되어 있지 않습니다.\n"
                                           "pip install matplotlib")
        except Exception as e:
            messagebox.showerror("시각화", f"그래프 생성 중 오류\n{e}")

    def on_quit(self):
        self.root.destroy()

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

        # 온도는 공란인 줄이 섞일 수 있으므로 값이 있는 표본만 시간과 짝지어 모은다
        def temp_series(key):
            pairs = [(r["t_s"], r[key]) for r in rows if r[key] is not None]
            return [p[0] for p in pairs], [p[1] for p in pairs]

        t_motor, motor_temp = temp_series("motor_temp")
        t_ctrl,  ctrl_temp  = temp_series("ctrl_temp")

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
            ax.set_title(f"시간별 버스전류와 상전류  [{tag}]")
            ax.legend()
            ax.grid(True, alpha=0.3)

        def plot_power(ax):
            ax.plot(t, power, color="tab:orange")
            ax.axhline(0, color="#999", lw=0.8)
            ax.set_xlabel("시간 (초)")
            ax.set_ylabel("소모전력 (W)")
            ax.set_title(f"시간별 소모전력 (배터리전압 x 버스전류)  [{tag}]")
            ax.grid(True, alpha=0.3)

        def plot_rpm(ax):
            ax.plot(t, rpm, color="tab:purple")
            ax.set_xlabel("시간 (초)")
            ax.set_ylabel("모터 회전수 (rpm)")
            ax.set_title(f"시간별 RPM  [{tag}]")
            ax.grid(True, alpha=0.3)

        def plot_temps(ax):
            # 둘 중 하나만 기록돼 있으면 그것만 그린다
            if motor_temp:
                ax.plot(t_motor, motor_temp, color="tab:red", label="모터 온도")
            if ctrl_temp:
                ax.plot(t_ctrl, ctrl_temp, color="tab:blue", label="컨트롤러 온도")
            ax.set_xlabel("시간 (초)")
            ax.set_ylabel("온도 (°C)")
            ax.set_title(f"시간별 모터·컨트롤러 온도  [{tag}]")
            ax.legend()
            ax.grid(True, alpha=0.3)

        plots = [
            ("시간별 배터리 퍼센트", plot_pct),
            ("시간별 버스전류와 상전류", plot_currents),
            ("시간별 소모전력", plot_power),
            ("시간별 RPM", plot_rpm),
        ]
        if motor_temp or ctrl_temp:
            plots.append(("시간별 모터·컨트롤러 온도", plot_temps))
        else:
            # PC가 시리얼로 받아 쓴 예전 CSV에는 온도 열이 비어 있다
            self.status.set(f"불러옴: {tag}  (온도 열이 비어 있어 온도 그래프는 생략)")

        for i, (title, fn) in enumerate(plots):
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
