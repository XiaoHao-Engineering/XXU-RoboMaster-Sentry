# -*- coding: utf-8 -*-
"""XXU-RoboMaster-Sentry 串口绘图 / 在线调参上位机

用法:
    python tools/serialplot.py            # 自动列出串口
    python tools/serialplot.py COM4       # 直接指定
    python tools/serialplot.py COM4 8     # 指定串口和通道数

协议（和固件 user_serialplot.c 一致）:
    板子 -> PC :  0xAB + N x float32(小端) + 1 字节累加和
    PC -> 板子 :  name=value#        例如  wheel_kp=18.5#

依赖: pyserial, matplotlib
"""
import sys, struct, threading, time
from collections import deque
import tkinter as tk

import serial
from serial.tools import list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ------------------------------------------------------------------ 配置
HEADER   = 0xAB
BAUD     = 115200
MAX_PTS  = 500          # 每条曲线保留的点数

# 8 通道的名字（改这里就行）
NAMES = ["FL tgt", "FL fdb", "FR tgt", "FR fdb",
         "RL tgt", "RL fdb", "RR tgt", "RR fdb"]
# 若固件里 CHASSIS_SERIALPLOT_MOTOR = 0，改成 4 通道：
# NAMES = ["yaw", "pitch", "omega", "vx"]


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        print("没有找到任何串口，检查 USB 线")
        sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    if len(sys.argv) > 1:
        return sys.argv[1]
    s = input("选一个（填 COM 号或序号）: ").strip()
    if s.isdigit() and int(s) < len(ports):
        return ports[int(s)].device
    return s


class App:
    def __init__(self, root, ser, nch):
        self.ser, self.nch = ser, nch
        self.frame_len = 1 + 4 * nch + 1
        self.lock = threading.Lock()
        self.ch = [deque([0.0] * MAX_PTS, maxlen=MAX_PTS) for _ in range(nch)]
        self.n_frames = 0
        self.n_bad = 0
        self.running = True

        root.title("XXU-RoboMaster-Sentry  SerialPlot  @%s" % ser.port)
        root.geometry("1000x700")

        # ---- 图 ----
        self.fig = Figure(figsize=(9, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("samples")
        self.ax.set_ylabel("value")
        self.lines = [self.ax.plot([], [], lw=1, label=NAMES[i])[0]
                      for i in range(nch)]
        self.ax.legend(loc="upper right", fontsize=8, ncol=2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # ---- 调参输入 ----
        bar = tk.Frame(root)
        bar.pack(fill="x", padx=6, pady=4)
        tk.Label(bar, text="在线调参 :").pack(side="left")
        self.entry = tk.Entry(bar, width=34)
        self.entry.pack(side="left", padx=4)
        self.entry.bind("<Return>", lambda e: self.send())
        tk.Button(bar, text="发送", command=self.send).pack(side="left")
        tk.Label(bar, text="例:  wheel_kp=18.5#   omega_dir=-1#   test_wheel=1#").pack(side="left", padx=10)

        self.status = tk.Label(root, text="", anchor="w")
        self.status.pack(fill="x", padx=6)

        threading.Thread(target=self.reader, daemon=True).start()
        root.after(50, self.refresh)

    # ----------------------------------------------------- 收
    def reader(self):
        buf = bytearray()
        while self.running:
            try:
                n = self.ser.in_waiting
            except Exception:
                break
            if n == 0:
                time.sleep(0.004)
                continue
            buf += self.ser.read(n)

            while len(buf) >= self.frame_len:
                i = buf.find(HEADER)
                if i < 0:
                    buf.clear(); break
                if i > 0:
                    del buf[:i]; continue
                if len(buf) < self.frame_len:
                    break
                frame = bytes(buf[:self.frame_len])
                if (sum(frame[1:-1]) & 0xFF) == frame[-1]:
                    vals = struct.unpack_from("<%df" % self.nch, frame, 1)
                    with self.lock:
                        for k in range(self.nch):
                            self.ch[k].append(vals[k])
                        self.n_frames += 1
                    del buf[:self.frame_len]
                else:
                    self.n_bad += 1
                    del buf[:1]

    # ----------------------------------------------------- 发
    def send(self):
        cmd = self.entry.get().strip()
        if not cmd:
            return
        if not cmd.endswith("#"):
            cmd += "#"
        try:
            self.ser.write(cmd.encode("ascii"))
            self.entry.delete(0, "end")
        except Exception as e:
            print("发送失败:", e)

    # ----------------------------------------------------- 画
    def refresh(self):
        with self.lock:
            data = [list(c) for c in self.ch]
            nf, nb = self.n_frames, self.n_bad
        x = range(len(data[0]))
        for i, ln in enumerate(self.lines):
            ln.set_data(x, data[i])
        if nf > 0:
            self.ax.set_xlim(0, len(data[0]))
            lo = min(min(d) for d in data)
            hi = max(max(d) for d in data)
            pad = (hi - lo) * 0.1 or 1.0
            self.ax.set_ylim(lo - pad, hi + pad)
        self.status.config(text="帧 %d   校验错 %d   %s" %
                                (nf, nb, " ".join("%s=%.0f" % (NAMES[i].replace(" ", ""), data[i][-1])
                                                  for i in range(min(self.nch, 4)))))
        self.canvas.draw_idle()
        if self.running:
            self.root.after(50, self.refresh)

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    nch = int(sys.argv[2]) if len(sys.argv) > 2 else len(NAMES)
    port = pick_port()
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except Exception as e:
        print("打开 %s 失败: %s" % (port, e))
        print("（串口被别的程序占用？关掉串口助手/SerialPlot 再试）")
        sys.exit(1)

    print("已连接 %s @%d，通道数 %d，帧长 %d 字节" % (port, BAUD, nch, 1 + 4 * nch + 1))
    print("板子端要发 100Hz，波形应该动起来。不动就看下面几条。")

    root = tk.Tk()
    app = App(root, ser, nch)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
