#!/usr/bin/env python3
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
firmware_gui.py - Minimal GUI for TetherIA Aero Hand (16-byte serial protocol)

A simple Tkinter GUI to control the TetherIA Aero Hand via serial port:
- "Start Homing": sends a HOMING command 16 byte packet to the ESP.
- "Set-ID Servo": asks for an integer ID (0..250) and sends a REID command + the integer.
- "Trim Servo": asks for the servo id and the degrees +360/-360.
- "Upload Firmware" (select .bin and flash with esptool).
- 7 sliders (0..65535) to control joints  by sending 16 Bytes CTRL_POS Command.
- RX log window to show incoming parsed serial data and status messages.
- Status bar for connection status and info.
- Adjustable TX rate (default 40 Hz).
- Auto-detects merged vs app-only .bin files for flashing.
- Handles esptool installation if missing.
- Uses pyserial for serial communication.
"""
import sys
import os
import threading
import time
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog

from serial.tools import list_ports

from aero_open_sdk.aero_hand import AeroHand


# ---- operation codes ------------
HOMING_MODE = 0x01
SET_ID_MODE = 0x03
TRIM_MODE   = 0x04

CTRL_POS = 0x11

GET_ALL  = 0x21
GET_POS  = 0x22
GET_VEL  = 0x23
GET_CURR = 0x24
GET_TEMP = 0x25
# ---- GUI ---------------------------------------------------------------------
BAUDS = [
    9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
]

SLIDER_LABELS = [
    "拇指外展",
    "拇指弯曲",
    "拇指肌腱",
    "食指",
    "中指",
    "无名指",
    "小指",
]

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("TetherIA – Aero Hand Open 灵巧手控制器")
        self.geometry("900x620")
        self.minsize(860, 560)
        if sys.platform.startswith("win"):
            self.state("zoomed")
        elif sys.platform == "darwin":
            self.update_idletasks()
            self.geometry(f"{self.winfo_screenwidth()}x{self.winfo_screenheight()}+0+0")
        else:
            self.attributes("-zoomed", True)

        try:
            icon_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "logo.png")
            icon_img = tk.PhotoImage(file=icon_path)
            self.iconphoto(True, icon_img)
        except Exception as e:
            print(f"无法设置窗口图标：{e}")

        # runtime state
        self.hand: AeroHand | None = None
        self.tx_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.control_paused = False  # pause streaming during blocking ops
        self.tx_rate_hz = 50.0       # streaming rate for CTRL_POS
        self.slider_vars: list[tk.DoubleVar] = []  # Use DoubleVar for normalized 0.0-1.0 range
        self.port_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=921600)

        self._build_ui()
        self._refresh_ports()

    # ---------------- UI ----------------
    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)

        # Port + refresh
        ttk.Label(top, text="串口：").pack(side=tk.LEFT)
        self.port_cmb = ttk.Combobox(top, textvariable=self.port_var, width=20, state="readonly")
        self.port_cmb.pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side=tk.LEFT, padx=(0, 12))

        # Baud select
        ttk.Label(top, text="波特率：").pack(side=tk.LEFT)
        self.baud_cmb = ttk.Combobox(top, width=10, state="readonly",
                                     values=[str(b) for b in BAUDS], textvariable=self.baud_var)
        self.baud_cmb.set(str(self.baud_var.get()))
        self.baud_cmb.pack(side=tk.LEFT, padx=(4, 12))

        # Connect / Disconnect
        self.btn_connect = ttk.Button(top, text="连接", command=self.on_connect)
        self.btn_connect.pack(side=tk.LEFT, padx=(0, 8))
        self.btn_disc = ttk.Button(top, text="断开", command=self.on_disconnect, state=tk.DISABLED)
        self.btn_disc.pack(side=tk.LEFT, padx=(0, 16))
        
        # Streaming rate
        ttk.Label(top, text="发送频率 (Hz)：").pack(side=tk.LEFT)
        self.rate_spin = ttk.Spinbox(top, from_=1, to=200, width=6)
        self.rate_spin.delete(0, tk.END)
        self.rate_spin.insert(0, "50")
        self.rate_spin.pack(side=tk.LEFT, padx=(4, 0))

        # ---- Commands row
        cmd = ttk.Frame(self, padding=(10, 4))
        cmd.pack(side=tk.TOP, fill=tk.X)

        self.btn_homing = ttk.Button(cmd, text="执行归位", command=self.on_homing, state=tk.DISABLED)
        self.btn_homing.pack(side=tk.LEFT, padx=(0, 10))

        self.btn_setid = ttk.Button(cmd, text="设置电机 ID", command=self.on_set_id, state=tk.DISABLED)
        self.btn_setid.pack(side=tk.LEFT, padx=(0, 10))

        self.btn_trim = ttk.Button(cmd, text="校准电机", command=self.on_trim, state=tk.DISABLED)
        self.btn_trim.pack(side=tk.LEFT, padx=(0, 10))

        self.btn_flash = ttk.Button(cmd, text="烧录固件", command=self.on_flash)
        self.btn_flash.pack(side=tk.LEFT, padx=(0, 10))

        # Zero All Button
        self.btn_zero = ttk.Button(cmd, text="设为张开姿态", command=self.on_zero_all, state=tk.DISABLED)
        self.btn_zero.pack(side=tk.LEFT, padx=(0, 10))

        #Set Speed Button
        self.btn_set_speed = ttk.Button(cmd, text="设置速度", command=self.on_set_speed, state=tk.DISABLED)
        self.btn_set_speed.pack(side=tk.LEFT, padx=(0, 10))

        #Set Torque Button
        self.btn_set_torque = ttk.Button(cmd, text="设置扭矩", command=self.on_set_torque, state=tk.DISABLED)
        self.btn_set_torque.pack(side=tk.LEFT, padx=(0, 10))

        # GET buttons
        self.btn_get_pos  = ttk.Button(cmd, text="读取位置", command=self.on_get_pos,  state=tk.DISABLED)
        self.btn_get_vel  = ttk.Button(cmd, text="读取速度", command=self.on_get_vel,  state=tk.DISABLED)
        self.btn_get_cur  = ttk.Button(cmd, text="读取电流", command=self.on_get_cur,  state=tk.DISABLED)
        self.btn_get_temp = ttk.Button(cmd, text="读取温度", command=self.on_get_temp, state=tk.DISABLED)
        self.btn_get_all  = ttk.Button(cmd, text="读取全部", command=self.on_get_all,  state=tk.DISABLED)
        self.btn_get_pos.pack(side=tk.LEFT, padx=(20, 6))
        self.btn_get_vel.pack(side=tk.LEFT, padx=6)
        self.btn_get_cur.pack(side=tk.LEFT, padx=6)
        self.btn_get_temp.pack(side=tk.LEFT, padx=6)
        self.btn_get_all.pack(side=tk.LEFT, padx=6)

        # ---- Sliders (7)
        self.grp = ttk.LabelFrame(self, text="关节位置控制（发送 CTRL_POS 数据）", padding=10)
        self.grp.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=(6, 10))

        self.slider_vars = []
        self.slider_widgets = []
        mono_font = ("Consolas", 10)
        for i, name in enumerate(SLIDER_LABELS):
            row = ttk.Frame(self.grp)
            row.pack(fill=tk.X, pady=5)
            ttk.Label(row, text=f"{i} – {name}", width=22).pack(side=tk.LEFT, padx=(0, 8))
            min_lbl = ttk.Label(row, text="0.000", width=8, font=mono_font)
            min_lbl.pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.slider_vars.append(var)
            scale = tk.Scale(row, from_=0.0, to=1.0, orient=tk.HORIZONTAL, length=600,
                              resolution=0.001, variable=var, showvalue=True, font=mono_font)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
            self.slider_widgets.append(scale)
            max_lbl = ttk.Label(row, text="1.000", width=8, font=mono_font)
            max_lbl.pack(side=tk.LEFT)

        # Torque slider (hidden by default)
        self.torque_frame = ttk.Frame(self)
        self.torque_slider_var = tk.DoubleVar(value=0.0)
        self.torque_slider = tk.Scale(self.torque_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL, length=600,
                                      resolution=0.001, variable=self.torque_slider_var, showvalue=True,
                                      label="扭矩（0.000 - 1.000）", command=self._on_torque_slider)
        self.torque_slider.pack(side=tk.LEFT, padx=20, pady=10)
        self.btn_back_joint = ttk.Button(self.torque_frame, text="返回位置控制", command=self.disable_torque_control)
        self.btn_back_joint.pack(side=tk.LEFT, padx=20, pady=10)
        self.torque_frame.pack_forget()

        # Torque Control Button below sliders
        self.btn_torque_control = ttk.Button(self, text="扭矩控制", command=self.on_torque_control, state=tk.NORMAL)
        self.btn_torque_control.pack(side=tk.TOP, pady=(0, 10))

        # ---- RX log
        rx = ttk.LabelFrame(self, text="收发日志", padding=10)
        rx.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        self.rx_text = tk.Text(rx, height=10)
        self.rx_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(rx, command=self.rx_text.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.rx_text.configure(yscrollcommand=sb.set)

        # ---- statusbar
        self.status_var = tk.StringVar(value="未连接")
        status_bar = ttk.Frame(self)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(0, 6))

        ttk.Label(status_bar, textvariable=self.status_var, anchor="w").pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(status_bar, text="清空日志", command=self._clear_rx).pack(side=tk.RIGHT)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------- helpers -------------
    def log(self, s: str):
        self.rx_text.insert(tk.END, s + ("\n" if not s.endswith("\n") else ""))
        self.rx_text.see(tk.END)

    def set_status(self, s: str):
        self.status_var.set(s)

    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cmb["values"] = ports
        # auto-select the first port if none chosen
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        if not ports and not self.port_var.get():
            # plausible defaults
            if sys.platform.startswith("win"):
                self.port_var.set("COM12")
            else:
                self.port_var.set("/dev/ttyUSB0")

    def on_torque_control(self):
        # Stop CTRL_POS streaming and disable joint sliders
        self.control_paused = True
        self.set_status("已进入扭矩控制模式，CTRL_POS 位置数据发送已停止")
        for scale in self.slider_widgets:
            scale.configure(state=tk.DISABLED)
        self.grp.configure(text="关节位置控制（CTRL_POS 已禁用）")
        # Show torque slider
        self.torque_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(6, 10))
        self.torque_slider.configure(state=tk.NORMAL)

    def _on_torque_slider(self, val):
        t_val = int(float(val) * 1000)
        try:
            self.hand.ctrl_torque([t_val]*7)
            self.log(f"[发送] CTRL_TOR 扭矩值：{t_val}")
            self.set_status(f"扭矩已设置为 {t_val}")
        except Exception as e:
            self.log(f"[错误] 扭矩设置失败：{e}")
            self.set_status("扭矩设置失败")

    def disable_torque_control(self):
        # Hide torque slider and re-enable joint sliders
        self.control_paused = False
        self.torque_frame.pack_forget()
        for scale in self.slider_widgets:
            scale.configure(state=tk.NORMAL)
        self.grp.configure(text="关节位置控制（发送 CTRL_POS 数据）")

    # ------------- connect/disconnect -------------
    def on_connect(self):
        if self.hand is not None:
            return False  # Already connected
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("错误", "请选择串口。")
            return False
        try:
            self.tx_rate_hz = float(self.rate_spin.get().strip())
            if self.tx_rate_hz <= 0:
                raise ValueError
        except Exception:
            messagebox.showerror("错误", "发送频率必须是正数。")
            return False

        baud = int(self.baud_var.get())
        try:
            self.hand = AeroHand(port, baudrate=baud)

            self.stop_event.clear()
            self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self.tx_thread.start()

            self.btn_connect.configure(state=tk.DISABLED)
            self.btn_disc.configure(state=tk.NORMAL)
            for b in (self.btn_zero,self.btn_homing, self.btn_setid, self.btn_trim,self.btn_set_speed,self.btn_set_torque,
                      self.btn_get_pos, self.btn_get_vel, self.btn_get_cur, self.btn_get_temp, self.btn_get_all):
                b.configure(state=tk.NORMAL)

            self.set_status(f"已连接到 {port}，波特率 {baud}")
            self.log(f"[信息] 已连接 {port}，波特率 {baud}")
            return True  # Success!
        except Exception as e:
            self.hand = None
            messagebox.showerror("连接失败", str(e))
            return False  # Failure

    def on_disconnect(self):
        self._shutdown_serial()

    def _shutdown_serial(self):
        self.control_paused = True
        self.stop_event.set()
        if self.tx_thread and self.tx_thread.is_alive():
            try:
                self.tx_thread.join(timeout=0.5)
            except Exception:
                pass
        if self.hand:
            try:
                self.hand.close()
            except Exception:
                pass
        self.hand = None
        self.tx_thread = None
        self.control_paused = False

        self.btn_connect.configure(state=tk.NORMAL)
        self.btn_disc.configure(state=tk.DISABLED)
        for b in (self.btn_zero,self.btn_homing, self.btn_setid, self.btn_trim,self.btn_set_speed,self.btn_set_torque,
                  self.btn_get_pos, self.btn_get_vel, self.btn_get_cur, self.btn_get_temp, self.btn_get_all):
            b.configure(state=tk.DISABLED)
        self.set_status("未连接")
        self.log("[信息] 连接已断开")

    # ------------- TX streaming (CTRL_POS) -------------
    def _tx_loop(self):
        period = 1.0 / max(1e-3, self.tx_rate_hz)
        next_t = time.perf_counter()
        while not self.stop_event.is_set():
            if self.hand is not None and not self.control_paused:
                # ## Unnormalize to joint limits
                j_ll = self.hand.joint_lower_limits
                j_ul = self.hand.joint_upper_limits
                joint_values = [j_ll[i] + (j_ul[i] - j_ll[i]) * self.slider_vars[i].get() for i in range(7)]
                try:
                    self.hand.set_joint_positions(joint_values)
                except Exception as e:
                    self.log(f"[发送错误] {e}")
            # pacing
            next_t += period
            to_sleep = next_t - time.perf_counter()
            if to_sleep > 0:
                time.sleep(to_sleep)
            else:
                next_t = time.perf_counter()

    def on_homing(self):
        if not self.hand:
            return

        def worker():
            try:
                self.control_paused = True
                self.set_status("正在执行归位…等待应答")
                self.log("[发送] 已发送 HOMING (0x01)，等待 16 字节应答…")
                ok = self.hand.send_homing(timeout_s=175.0)
                if ok:
                    self.log("[应答] 归位完成。")
                    self.set_status("归位完成")
            except Exception as e:
                self.log(f"[错误] 归位失败：{e}")
                self.set_status("归位失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_set_id(self):
        if not self.hand:
            return
        new_id = simpledialog.askinteger("设置电机 ID", "请输入新 ID（0..253）：", minvalue=0, maxvalue=253, parent=self)
        if new_id is None:
            return
        cur_lim = simpledialog.askinteger("电流限制", "请输入电流限制（0..1023）：",
                                          minvalue=0, maxvalue=1023, initialvalue=1023, parent=self)
        if cur_lim is None:
            return

        def worker():
            try:
                self.control_paused = True
                self.set_status("正在设置电机 ID…等待应答")
                self.log(f"[发送] SET_ID（ID={new_id}，电流限制={cur_lim}）")
                ack = self.hand.set_id(new_id, cur_lim)  # dict with Old_id, New_id, Current_limit
                self.log(f"[应答] SET_ID：原 ID={ack['Old_id']}，新 ID={ack['New_id']}，电流限制={ack['Current_limit']}")
                self.set_status("电机 ID 设置完成")
            except Exception as e:
                self.log(f"[错误] SET_ID 失败：{e}")
                self.set_status("电机 ID 设置失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_set_speed(self):
        if not self.hand:
            return
        ch = simpledialog.askinteger("设置速度", "电机 ID / 通道（0..6）：",
                                     minvalue=0, maxvalue=6, parent=self)
        if ch is None:
            return
        speed = simpledialog.askinteger("设置速度", "速度（0..32766）：",
                                       minvalue=0, maxvalue=32766, initialvalue=32766, parent=self)
        if speed is None:
            return

        def worker():
            try:
                self.control_paused = True
                self.set_status("正在设置速度…等待应答")
                self.log(f"[发送] SET_SPEED（通道={ch}，速度={speed}）")
                ack = self.hand.set_speed(ch, speed)  # dict with Servo ID, Speed
                self.log(f"[应答] SET_SPEED：ID={ack['Servo ID']}，速度={ack['Speed']}")
                self.set_status("速度设置完成")
            except Exception as e:
                self.log(f"[错误] SET_SPEED 失败：{e}")
                self.set_status("速度设置失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_set_torque(self):
        if not self.hand:
            return
        ch = simpledialog.askinteger("设置扭矩", "电机 ID / 通道（0..6）：",
                                     minvalue=0, maxvalue=6, parent=self)
        if ch is None:
            return
        torque = simpledialog.askinteger("设置扭矩", "扭矩（0..1000）：",
                                        minvalue=0, maxvalue=1000, initialvalue=1023, parent=self)
        if torque is None:
            return

        def worker():
            try:
                self.control_paused = True
                self.set_status("正在设置扭矩…等待应答")
                self.log(f"[发送] SET_TORQUE（通道={ch}，扭矩={torque}）")
                ack = self.hand.set_torque(ch, torque)  # dict with Servo ID, Torque
                self.log(f"[应答] SET_TORQUE：ID={ack['Servo ID']}，扭矩={ack['Torque']}")
                self.set_status("扭矩设置完成")
            except Exception as e:
                self.log(f"[错误] SET_TORQUE 失败：{e}")
                self.set_status("扭矩设置失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_zero_all(self):
        if not self.hand:
            return

        def worker():
            try:
                self.control_paused = True
                for var in self.slider_vars:
                    var.set(0.0)  # Set to 0.0 for normalized slider
                joint_pos = list(self.hand.joint_lower_limits)
                self.hand.set_joint_positions(joint_pos)
                self.log("[发送] 通过 CTRL_POS 执行 ZERO_ALL（关节下限）")
                self.set_status("已设为张开姿态（已发送关节下限并重置滑块）")
            except Exception as e:
                self.log(f"[错误] ZERO_ALL：{e}")
                self.set_status("设置张开姿态失败")
            finally:
                time.sleep(0.05)
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_trim(self):
        if not self.hand:
            return
        ch = simpledialog.askinteger("校准电机", "电机 ID / 通道（0..6）：",
                                     minvalue=0, maxvalue=6, parent=self)
        if ch is None:
            return
        deg = simpledialog.askinteger("校准电机", "校准角度（-360..360）：",
                                      minvalue=-360, maxvalue=360, initialvalue=0, parent=self)
        if deg is None:
            return

        def worker():
            try:
                self.control_paused = True
                self.set_status("正在校准电机…等待应答")
                self.log(f"[发送] TRIM（通道={ch}，角度={deg}）")
                ack = self.hand.trim_servo(ch, deg)  # dict with Servo ID, Extend Count
                self.log(f"[应答] TRIM：ID={ack['Servo ID']}，扩展计数={ack['Extend Count']}")
                self.set_status("电机校准完成")
            except Exception as e:
                self.log(f"[错误] TRIM 失败：{e}")
                self.set_status("电机校准失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    # ---- GET_* buttons (request + show parsed reply) ----
    def on_get_pos(self):
        if not self.hand:
            return
        try:
            vals = self.hand.get_actuations()
            j_ll = self.hand.actuation_lower_limits
            j_ul = self.hand.actuation_upper_limits
            # Convert to normalized 0.0-1.0 range for display
            norm_vals = [(vals[i] - j_ll[i]) / (j_ul[i] - j_ll[i]) for i in range(len(vals))]
            ## Format to 3 decimal places
            norm_vals_fmt = [round(v, 3) for v in norm_vals]
            self.log(f"[GET_POS] {norm_vals_fmt}")
        except Exception as e:
            self.log(f"[错误] GET_POS：{e}")

    def on_get_vel(self):
        if not self.hand:
            return
        try:
            vals = self.hand.get_actuator_speeds()
            self.log(f"[GET_VEL] {list(vals)}")
        except Exception as e:
            self.log(f"[错误] GET_VEL：{e}")

    def on_get_cur(self):
        if not self.hand:
            return
        try:
            vals = self.hand.get_actuator_currents()
            self.log(f"[GET_CURR] {list(vals)}")
        except Exception as e:
            self.log(f"[错误] GET_CURR：{e}")

    def on_get_temp(self):
        if not self.hand:
            return
        try:
            vals = self.hand.get_actuator_temperatures()
            self.log(f"[GET_TEMP] {list(vals)}")
        except Exception as e:
            self.log(f"[错误] GET_TEMP：{e}")
    
    def on_get_all(self):
        if not self.hand:
            return
        try:
            pos = self.hand.get_actuations()
            vel = self.hand.get_actuator_speeds()
            curr = self.hand.get_actuator_currents()
            temp = self.hand.get_actuator_temperatures()
            norm_pos = [round(v / 65535, 3) for v in pos]  # Normalize for display
            self.log(f"[GET_ALL] 位置：{norm_pos} | 速度：{list(vel)} | 电流：{list(curr)} | 温度：{list(temp)}")
        except Exception as e:
            self.log(f"[错误] GET_ALL：{e}")

    # ---- Flashing (esptool) ----
    def on_flash(self):
        bin_path = filedialog.askopenfilename(
            parent=self, title="选择 ESP32 固件（.bin）",
            filetypes=[("BIN 固件", "*.bin"), ("所有文件", "*.*")]
        )
        if not bin_path:
            return
        # pick a port (use connected one if available)
        port = self.port_var.get().strip() or simpledialog.askstring("串口", "请输入串口：", parent=self)
        if not port:
            return

        chip = "auto"
        offset = "0x10000"

        if self.hand:
            self.log("[烧录] 正在关闭串口连接…")
            self.on_disconnect()

        def worker():
            cmd = [sys.executable, "-m", "esptool",
                   "--chip", chip, "-p", port, "-b", "921600",
                   "write-flash", offset, bin_path]
            self.log("> " + " ".join(cmd))
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                for line in proc.stdout:
                    self.log(line.rstrip("\n"))
                rc = proc.wait()
                if rc == 0:
                    self.log("[烧录] 固件烧录完成。")
                    self.after(0, lambda: messagebox.showinfo("成功", "固件烧录成功。"))
                else:
                    self.log(f"[烧录] esptool 退出码：{rc}")
                    self.after(0, lambda rc=rc: messagebox.showerror("烧录失败", f"esptool 退出码：{rc}"))
            except Exception as e:
                self.log(f"[烧录错误] {e}")
                self.after(0, lambda e=e: messagebox.showerror("烧录失败", str(e)))

            max_attempts = 3
            for attempt in range(1, max_attempts + 1):
                time.sleep(1.5) 
                self.log(f"[烧录] 第 {attempt} 次尝试重新连接…")
                result = {'ok': False}
                done = threading.Event()
                def _do_connect():
                    try:
                        ok = bool(self.on_connect()) 
                        result['ok'] = ok
                    except Exception as e:
                        self.log(f"[烧录] 重新连接异常：{e}")
                        result['ok'] = False
                    finally:
                        done.set()
                self.after(0, _do_connect)
                if not done.wait(3.0): 
                    self.log("[烧录] 重新连接超时")
                    continue                   
                if result['ok']:
                    self.log("[烧录] 已重新连接 ✅")
                    break                     
            else:
                self.after(0, lambda: self.set_status("固件烧录后重新连接失败"))
                self.after(0, lambda: messagebox.showerror("重新连接失败", "固件烧录完成，但无法重新连接串口。"))

        threading.Thread(target=worker, daemon=True).start()

    # ---- to clear RX window 
    def _clear_rx(self):
        """Clear the RX log text box."""
        try:
            self.rx_text.delete("1.0", tk.END)
        except Exception:
            pass
    # ---- teardown ----
    def _on_close(self):
        try:
            self._shutdown_serial()
        finally:
            self.destroy()

def main():
    app = App()
    app.mainloop()

if __name__ == "__main__":
    main()
