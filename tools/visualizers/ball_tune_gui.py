#!/usr/bin/env python3
"""H3 滚球控制的实时整定台：左侧改参数，右侧看波形，同一根串口。

固件侧对应 `app/app_ball_tune.c`。协议是 ASCII 行：

    ?              列出全部参数
    s <名> <值>    改一个
    g <名>         读一个
    d              恢复编译期默认值

回显以 `[TUNE]` 起头，遥测以 `[SCV]` 起头，本工具按前缀分流。

为什么值得做：整定一轮原本要「改 .c → 编译 → 烧录 → 摆球 → 抓数」约 5 分钟，
热更之后是「拖一下滑块 → 摆球 → 看曲线」约 20 秒。整定的瓶颈从来不是想法，
是试验轮次。

⚠ 固件只改内存，不写 flash。掉电即恢复源码默认值——**整定出来的数必须回填
  源码并提交才算数**，本工具右下角的「导出为 C」就是为此准备的。

用法：
    python ball_tune_gui.py --port COM16
    python ball_tune_gui.py --list

依赖：pip install pyserial matplotlib
"""
from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import font_manager
import serial
from serial.tools import list_ports

_CJK = next((f for f in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC")
             if f in {x.name for x in font_manager.fontManager.ttflist}), None)
if _CJK:
    matplotlib.rcParams["font.sans-serif"] = [_CJK, "sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False

SCV = re.compile(r"(\w+)=(-?[\d.]+)")
TUNE_P = re.compile(r"\[TUNE\] p name=(\S+) val=(\S+) min=(\S+) max=(\S+) unit=(\S+)")
TUNE_OK = re.compile(r"\[TUNE\] ok name=(\S+) val=(\S+) old=(\S+)")
TUNE_ERR = re.compile(r"\[TUNE\] err (.*)")

WINDOW_S = 20.0

# 面板分组：(标题, [参数名])。名字必须与固件参数表一致。
GROUPS = [
    ("反馈", ["kp", "kd", "holdkd", "fblim"]),
    ("积分 (PID for MOVE)", ["ki", "kilim", "kileak", "kiminv"]),
    ("MOVE/HOLD 调度", ["henter", "hentv", "hdwell", "hexit", "htau"]),
    ("抖动 (破静摩擦)", ["dith", "dithhz", "dithe", "dithv", "dithd"]),
    ("前馈与标定", ["bias", "roll", "carff"]),
    ("剖面与执行器", ["amax", "vmax", "arate", "servokp"]),
]

PLOTS = [
    ("球位置 (mm)", [("x", "x"), ("xref", "参考"), ("tgt", "目标")]),
    ("球速度 (mm/s)", [("v", "v")]),
    ("水管角 (deg)", [("u", "指令 u"), ("beam", "实际 beam")]),
    ("控制分量 (deg)", [("fb", "fb"), ("ki", "积分"), ("dith", "抖动")]),
]


class Link:
    """串口读写线程。读到的行按前缀分流进两个队列。"""

    def __init__(self, port: str, baud: int = 115200):
        self.serial = serial.Serial(port, baud, timeout=0.2)
        self.telemetry: queue.Queue[dict[str, float]] = queue.Queue(maxsize=4000)
        self.replies: queue.Queue[str] = queue.Queue(maxsize=2000)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        buffer = b""
        while self.running:
            try:
                chunk = self.serial.read(4096)
            except serial.SerialException:
                break
            if chunk:
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    line = raw.decode("ascii", "ignore").strip()
                    if line.startswith("[SCV]") and "drop=" in line:
                        row = {m.group(1): float(m.group(2)) for m in SCV.finditer(line)}
                        if not self.telemetry.full():
                            self.telemetry.put(row)
                    elif line.startswith("[TUNE]") or line.startswith("[SCVCFG]"):
                        if not self.replies.full():
                            self.replies.put(line)
            else:
                time.sleep(0.005)

    def send(self, text: str) -> None:
        try:
            self.serial.write((text + "\n").encode("ascii"))
        except serial.SerialException:
            pass

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=0.5)
        try:
            self.serial.close()
        except Exception:
            pass


class Row:
    """一个参数：标签 + 滑块 + 输入框。滑块用于粗调，输入框用于精确值。"""

    def __init__(self, parent: tk.Widget, name: str, app: "TuneApp"):
        self.name = name
        self.app = app
        self.min_value = 0.0
        self.max_value = 1.0
        self.unit = ""
        self.known = False
        self._suppress = False

        self.frame = ttk.Frame(parent)
        self.frame.pack(fill="x", padx=2, pady=1)
        self.label = ttk.Label(self.frame, text=name, width=9, anchor="w")
        self.label.pack(side="left")
        self.scale = ttk.Scale(self.frame, from_=0.0, to=1.0, orient="horizontal",
                               command=self._on_scale, length=170)
        self.scale.pack(side="left", padx=3)
        self.entry = ttk.Entry(self.frame, width=9, justify="right")
        self.entry.pack(side="left")
        self.entry.bind("<Return>", self._on_entry)
        self.hint = ttk.Label(self.frame, text="", width=13, anchor="w",
                              foreground="#777")
        self.hint.pack(side="left", padx=2)

    def describe(self, value: float, low: float, high: float, unit: str) -> None:
        self.min_value, self.max_value, self.unit = low, high, unit
        self.known = True
        self.scale.configure(from_=low, to=high)
        self.hint.configure(text=f"[{low:g},{high:g}]")
        self.show(value)

    def show(self, value: float) -> None:
        self._suppress = True
        self.scale.set(value)
        self.entry.delete(0, "end")
        self.entry.insert(0, f"{value:.5g}")
        self._suppress = False

    def _on_scale(self, raw: str) -> None:
        if self._suppress or not self.known:
            return
        # 滑块连续拖动会刷屏，交给 app 做节流。
        self.app.request_set(self.name, float(raw), throttle=True)
        self._suppress = True
        self.entry.delete(0, "end")
        self.entry.insert(0, f"{float(raw):.5g}")
        self._suppress = False

    def _on_entry(self, _event: object) -> None:
        try:
            value = float(self.entry.get())
        except ValueError:
            return
        self.app.request_set(self.name, value, throttle=False)


class TuneApp:
    def __init__(self, root: tk.Tk, link: Link):
        self.root = root
        self.link = link
        self.rows: dict[str, Row] = {}
        self.history: list[dict[str, float]] = []
        self.t0: float | None = None
        self._pending: dict[str, float] = {}
        self._last_flush = 0.0

        root.title("H3 滚球实时整定台")
        root.geometry("1380x860")

        outer = ttk.Frame(root)
        outer.pack(fill="both", expand=True)

        # ---- 左：参数面板 ----
        left = ttk.Frame(outer, width=430)
        left.pack(side="left", fill="y", padx=6, pady=6)
        left.pack_propagate(False)

        bar = ttk.Frame(left)
        bar.pack(fill="x", pady=(0, 4))
        ttk.Button(bar, text="重新读取", command=lambda: link.send("?")).pack(side="left")
        ttk.Button(bar, text="恢复默认", command=self._defaults).pack(side="left", padx=3)
        ttk.Button(bar, text="导出为 C", command=self._export).pack(side="left")

        canvas = tk.Canvas(left, highlightthickness=0)
        scrollbar = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        holder = ttk.Frame(canvas)
        holder.bind("<Configure>",
                    lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=holder, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for title, names in GROUPS:
            box = ttk.LabelFrame(holder, text=title)
            box.pack(fill="x", padx=2, pady=3)
            for name in names:
                self.rows[name] = Row(box, name, self)

        self.status = tk.Text(left, height=7, wrap="none", font=("Consolas", 8))
        self.status.pack(fill="x", pady=(4, 0))

        # ---- 右：波形 ----
        right = ttk.Frame(outer)
        right.pack(side="left", fill="both", expand=True, padx=(0, 6), pady=6)
        self.figure = Figure(figsize=(9, 8), dpi=96)
        self.axes = self.figure.subplots(len(PLOTS), 1, sharex=True)
        self.lines: dict[str, object] = {}
        for axis, (title, series) in zip(self.axes, PLOTS):
            axis.set_ylabel(title, fontsize=9)
            axis.grid(alpha=0.3)
            axis.tick_params(labelsize=8)
            for key, label in series:
                (line,) = axis.plot([], [], linewidth=1.2, label=label)
                self.lines[key] = line
            axis.legend(fontsize=7, loc="upper right", ncol=3)
        self.axes[-1].set_xlabel("时间 (s)", fontsize=9)
        self.figure.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        link.send("?")
        root.after(60, self._tick)
        root.protocol("WM_DELETE_WINDOW", self._close)

    # ---- 参数写入：滑块拖动做节流，避免刷爆串口 ----
    def request_set(self, name: str, value: float, throttle: bool) -> None:
        self._pending[name] = value
        if not throttle:
            self._flush()

    def _flush(self) -> None:
        for name, value in self._pending.items():
            self.link.send(f"s {name} {value:.6g}")
        self._pending.clear()
        self._last_flush = time.time()

    def _defaults(self) -> None:
        self.link.send("d")
        self.root.after(150, lambda: self.link.send("?"))

    def _export(self) -> None:
        """把当前值导出成可直接粘回 app_ball_scurve_task.c 的片段。"""
        lines = ["    /* 由 ball_tune_gui.py 导出，"
                 f"{time.strftime('%Y-%m-%d %H:%M')} */"]
        for _, names in GROUPS:
            for name in names:
                row = self.rows.get(name)
                if row is None or not row.known:
                    continue
                try:
                    value = float(row.entry.get())
                except ValueError:
                    continue
                lines.append(f"    /* {name:<9} */ {value:.6g}f,")
        self._log("\n".join(lines))

    def _log(self, text: str) -> None:
        self.status.insert("end", text + "\n")
        self.status.see("end")

    def _tick(self) -> None:
        # 滑块节流：最多 10 Hz 发一次。
        if self._pending and (time.time() - self._last_flush) > 0.1:
            self._flush()

        while True:
            try:
                line = self.link.replies.get_nowait()
            except queue.Empty:
                break
            match = TUNE_P.match(line)
            if match:
                name, value, low, high, unit = match.groups()
                row = self.rows.get(name)
                if row is not None:
                    row.describe(float(value), float(low), float(high), unit)
                continue
            match = TUNE_OK.match(line)
            if match:
                name, value, old = match.groups()
                self._log(f"{name} : {old} → {value}")
                continue
            match = TUNE_ERR.match(line)
            if match:
                self._log("✗ " + match.group(1))

        while True:
            try:
                row = self.link.telemetry.get_nowait()
            except queue.Empty:
                break
            if self.t0 is None:
                self.t0 = row.get("t", 0.0)
            row["_t"] = (row.get("t", 0.0) - self.t0) / 1000.0
            self.history.append(row)
        if len(self.history) > 6000:
            del self.history[:-6000]

        if self.history:
            now = self.history[-1]["_t"]
            window = [r for r in self.history if r["_t"] >= now - WINDOW_S]
            times = [r["_t"] for r in window]
            for key, line_obj in self.lines.items():
                line_obj.set_data(times, [r.get(key, 0.0) for r in window])
            for axis in self.axes:
                axis.relim()
                axis.autoscale_view()
                axis.set_xlim(max(0.0, now - WINDOW_S), now + 0.5)
            self.canvas.draw_idle()

        self.root.after(60, self._tick)

    def _close(self) -> None:
        self.link.close()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    arguments = parser.parse_args()

    if arguments.list or not arguments.port:
        for info in list_ports.comports():
            print(f"{info.device}  {info.description}")
        if not arguments.port:
            sys.exit("需要 --port")
        return

    root = tk.Tk()
    TuneApp(root, Link(arguments.port, arguments.baud))
    root.mainloop()


if __name__ == "__main__":
    main()
