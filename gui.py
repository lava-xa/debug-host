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
import queue
import threading
import time
import subprocess
import shutil
import tempfile
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk, messagebox, simpledialog, filedialog

from serial.tools import list_ports

from aero_hand import AeroHand


# ---- operation codes ------------
HOMING_MODE = 0x01
SET_ID_MODE = 0x02
TRIM_MODE   = 0x03

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
        # Use a dedicated X11 window class so desktop environments do not
        # group this GUI with the terminal/Codex process that launched it.
        super().__init__(className="AeroHandControl")
        self._configure_fonts()
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
            icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logo.png")
            icon_source = tk.PhotoImage(file=icon_path)

            # Tk/X11 on this system rejects _NET_WM_ICON when any supplied
            # image is larger than 128px, so only publish desktop-safe sizes.
            source_size = max(icon_source.width(), icon_source.height())
            scale_factors = {
                max(1, round(source_size / target_size))
                for target_size in (128, 64, 48, 32, 16)
            }
            self.icon_images = [
                icon_source.subsample(factor, factor)
                for factor in sorted(scale_factors)
            ]
            # Keep all PhotoImage objects for the lifetime of the window.
            self.iconphoto(True, *self.icon_images)
        except Exception as e:
            print(f"无法设置窗口图标：{e}")

        # runtime state
        self.hand: AeroHand | None = None
        self.tx_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self._main_thread = threading.current_thread()
        self._ui_queue = queue.Queue()
        self.control_paused = False  # pause streaming during blocking ops
        self.active_control_mode = "position"
        self.signed_batch_control_available = False
        self.tx_rate_hz = 50.0       # streaming rate for CTRL_POS
        self.slider_vars: list[tk.DoubleVar] = []  # Use DoubleVar for normalized 0.0-1.0 range
        self.slider_values = [0.0] * 7
        self.speed_values = [0] * 7
        self.torque_values = [0] * 7
        self.port_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=921600)

        self._build_ui()
        self._refresh_ports()
        self.after(25, self._drain_ui_queue)

    def _configure_fonts(self):
        """为不同操作系统选择清晰的中文字体。"""
        self._register_linux_cjk_font()
        available_families = {
            family.casefold(): family for family in tkfont.families(self)
        }

        if sys.platform.startswith("win"):
            ui_candidates = (
                "Microsoft YaHei UI", "Microsoft YaHei", "Noto Sans CJK SC", "Segoe UI"
            )
        elif sys.platform == "darwin":
            ui_candidates = (
                "PingFang SC", "Hiragino Sans GB", "Noto Sans CJK SC", "Helvetica Neue"
            )
        else:
            ui_candidates = (
                "Noto Sans CJK SC", "WenQuanYi Micro Hei", "Droid Sans Fallback", "DejaVu Sans"
            )

        mono_candidates = (
            "Noto Sans Mono CJK SC", "WenQuanYi Micro Hei Mono", "Microsoft YaHei UI",
            "PingFang SC", "Consolas", "DejaVu Sans Mono"
        )

        default_family = tkfont.nametofont("TkDefaultFont").actual("family")
        self.ui_font_family = next(
            (
                available_families[family.casefold()]
                for family in ui_candidates
                if family.casefold() in available_families
            ),
            default_family,
        )
        self.mono_font_family = next(
            (
                available_families[family.casefold()]
                for family in mono_candidates
                if family.casefold() in available_families
            ),
            self.ui_font_family,
        )

        font_sizes = {
            "TkDefaultFont": 11,
            "TkTextFont": 11,
            "TkMenuFont": 11,
            "TkHeadingFont": 11,
            "TkCaptionFont": 11,
            "TkSmallCaptionFont": 10,
            "TkIconFont": 11,
            "TkTooltipFont": 10,
        }
        for font_name, font_size in font_sizes.items():
            try:
                tkfont.nametofont(font_name).configure(
                    family=self.ui_font_family,
                    size=font_size,
                )
            except tk.TclError:
                pass

        try:
            tkfont.nametofont("TkFixedFont").configure(
                family=self.mono_font_family,
                size=10,
            )
        except tk.TclError:
            pass

        # ttk 组件和下拉列表有各自的字体设置，需要单独统一。
        ttk.Style(self).configure(".", font=(self.ui_font_family, 11))
        self.option_add("*TCombobox*Listbox.font", (self.ui_font_family, 11))

    def _register_linux_cjk_font(self):
        """为未启用 Fontconfig 的 Linux Tk 注册中文无衬线字体。"""
        if not sys.platform.startswith("linux") or not os.environ.get("DISPLAY"):
            return

        current_families = {
            family.casefold() for family in tkfont.families(self)
        }
        if "wenquanyi micro hei" in current_families:
            return

        font_candidates = (
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/wqy-microhei/wqy-microhei.ttc",
        )
        font_path = next((path for path in font_candidates if os.path.isfile(path)), None)
        if not font_path:
            return
        if not all(shutil.which(command) for command in ("mkfontscale", "mkfontdir", "xset")):
            return

        try:
            cache_dir = os.path.join(
                tempfile.gettempdir(),
                f"aero-open-gui-fonts-{os.getuid()}",
            )
            os.makedirs(cache_dir, exist_ok=True)
            font_link = os.path.join(cache_dir, "wqy-microhei.ttc")
            if not os.path.exists(font_link):
                os.symlink(font_path, font_link)

            quiet = {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL}
            subprocess.run(["mkfontscale", cache_dir], check=True, **quiet)
            subprocess.run(["mkfontdir", cache_dir], check=True, **quiet)
            subprocess.run(["xset", "+fp", cache_dir], check=True, **quiet)
            subprocess.run(["xset", "fp", "rehash"], check=True, **quiet)
        except (OSError, subprocess.CalledProcessError):
            # 字体注册失败时仍可使用 Tk 默认字体启动程序。
            pass

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
        mono_font = (self.mono_font_family, 10)
        for i, name in enumerate(SLIDER_LABELS):
            row = ttk.Frame(self.grp)
            row.pack(fill=tk.X, pady=5)
            ttk.Label(row, text=f"{i} – {name}", width=22).pack(side=tk.LEFT, padx=(0, 8))
            min_lbl = ttk.Label(row, text="0.000", width=8, font=mono_font)
            min_lbl.pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.slider_vars.append(var)
            scale = tk.Scale(row, from_=0.0, to=1.0, orient=tk.HORIZONTAL, length=600,
                              resolution=0.001, variable=var, showvalue=True, font=mono_font,
                              command=lambda value, index=i: self._on_position_slider(index, value))
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
            self.slider_widgets.append(scale)
            max_lbl = ttk.Label(row, text="1.000", width=8, font=mono_font)
            max_lbl.pack(side=tk.LEFT)

        # Seven-motor settings are placed directly below the last joint.
        motor_settings = ttk.LabelFrame(self.grp, text="7 路电机参数", padding=(10, 8))
        motor_settings.pack(fill=tk.X, pady=(10, 0))
        self.btn_set_speed = ttk.Button(
            motor_settings,
            text="设置 7 路速度",
            command=self.on_set_speed,
            state=tk.DISABLED,
        )
        self.btn_set_speed.pack(side=tk.LEFT, padx=(0, 10))
        self.btn_set_torque = ttk.Button(
            motor_settings,
            text="设置 7 路扭矩",
            command=self.on_set_torque,
            state=tk.DISABLED,
        )
        self.btn_set_torque.pack(side=tk.LEFT, padx=(0, 12))
        self.btn_stop_direct = ttk.Button(
            motor_settings,
            text="停止并返回位置控制",
            command=self.on_stop_direct_control,
            state=tk.DISABLED,
        )
        self.btn_stop_direct.pack(side=tk.LEFT)

        # Torque slider (hidden by default)
        self.torque_frame = ttk.Frame(self)
        self.torque_slider_var = tk.DoubleVar(value=0.0)
        self.torque_slider = tk.Scale(self.torque_frame, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, length=600,
                                      resolution=0.001, variable=self.torque_slider_var, showvalue=True,
                                      label="扭矩（-1.000 – 1.000；正值逆时针）", command=self._on_torque_slider)
        self.torque_slider.pack(side=tk.LEFT, padx=20, pady=10)
        self.btn_back_joint = ttk.Button(self.torque_frame, text="返回位置控制", command=self.disable_torque_control)
        self.btn_back_joint.pack(side=tk.LEFT, padx=20, pady=10)
        self.torque_frame.pack_forget()

        # Torque Control Button below sliders
        self.btn_torque_control = ttk.Button(
            self, text="扭矩控制", command=self.on_torque_control, state=tk.DISABLED
        )
        self.btn_torque_control.pack(side=tk.TOP, pady=(0, 10))

        # ---- RX log
        rx = ttk.LabelFrame(self, text="收发日志", padding=10)
        rx.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        self.rx_text = tk.Text(rx, height=10, font=(self.mono_font_family, 10))
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
        if threading.current_thread() is not self._main_thread:
            self._ui_queue.put(("log", s))
            return
        self._append_log(s)

    def _append_log(self, s: str):
        self.rx_text.insert(tk.END, s + ("\n" if not s.endswith("\n") else ""))
        self.rx_text.see(tk.END)

    def set_status(self, s: str):
        if threading.current_thread() is not self._main_thread:
            self._ui_queue.put(("status", s))
            return
        self.status_var.set(s)

    def call_in_ui(self, callback):
        if threading.current_thread() is self._main_thread:
            callback()
        else:
            self._ui_queue.put(("call", callback))

    def _drain_ui_queue(self):
        try:
            while True:
                action, value = self._ui_queue.get_nowait()
                if action == "log":
                    self._append_log(value)
                elif action == "status":
                    self.status_var.set(value)
                elif action == "call":
                    value()
        except queue.Empty:
            pass
        try:
            self.after(25, self._drain_ui_queue)
        except tk.TclError:
            pass

    def _on_position_slider(self, index: int, value: str):
        self.slider_values[index] = float(value)

    def _ask_motor_values(
        self,
        title: str,
        parameter_name: str,
        current_values: list[int],
        minimum: int,
        maximum: int,
        direction_hint: str = "",
    ) -> list[int] | None:
        """在一个对话框中编辑 7 路电机参数。"""
        dialog = tk.Toplevel(self, class_="AeroHandControl")
        dialog.title(title)
        dialog.transient(self)
        dialog.resizable(False, False)

        content = ttk.Frame(dialog, padding=16)
        content.pack(fill=tk.BOTH, expand=True)
        range_text = f"{parameter_name}范围：{minimum} – {maximum}"
        if direction_hint:
            range_text += f"；{direction_hint}"
        ttk.Label(
            content,
            text=range_text,
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))

        value_vars = []
        for index, (motor_name, current_value) in enumerate(
            zip(SLIDER_LABELS, current_values)
        ):
            ttk.Label(
                content,
                text=f"{index} – {motor_name}：",
                width=18,
            ).grid(row=index + 1, column=0, sticky="e", padx=(0, 8), pady=4)
            value_var = tk.StringVar(value=str(current_value))
            value_vars.append(value_var)
            ttk.Spinbox(
                content,
                from_=minimum,
                to=maximum,
                textvariable=value_var,
                width=14,
            ).grid(row=index + 1, column=1, sticky="ew", pady=4)

        result = None

        def copy_first_to_all():
            first_value = value_vars[0].get()
            for value_var in value_vars[1:]:
                value_var.set(first_value)

        def apply_values():
            nonlocal result
            try:
                values = [int(value_var.get()) for value_var in value_vars]
            except ValueError:
                messagebox.showerror("参数错误", "所有参数必须是整数。", parent=dialog)
                return

            invalid = [
                index for index, value in enumerate(values)
                if not minimum <= value <= maximum
            ]
            if invalid:
                invalid_text = "、".join(str(index) for index in invalid)
                messagebox.showerror(
                    "参数超出范围",
                    f"电机 {invalid_text} 的{parameter_name}必须介于 {minimum} 和 {maximum} 之间。",
                    parent=dialog,
                )
                return

            result = values
            dialog.destroy()

        actions = ttk.Frame(content)
        actions.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        ttk.Button(
            actions,
            text="将 0 号值复制到全部",
            command=copy_first_to_all,
        ).pack(side=tk.LEFT)
        ttk.Button(actions, text="取消", command=dialog.destroy).pack(side=tk.RIGHT)
        ttk.Button(actions, text="应用到 7 路电机", command=apply_values).pack(
            side=tk.RIGHT, padx=(0, 8)
        )

        dialog.protocol("WM_DELETE_WINDOW", dialog.destroy)
        dialog.bind("<Return>", lambda _event: apply_values())
        dialog.bind("<Escape>", lambda _event: dialog.destroy())
        dialog.update_idletasks()
        x = self.winfo_rootx() + (self.winfo_width() - dialog.winfo_width()) // 2
        y = self.winfo_rooty() + (self.winfo_height() - dialog.winfo_height()) // 2
        dialog.geometry(f"+{max(0, x)}+{max(0, y)}")
        dialog.grab_set()
        self.wait_window(dialog)
        return result

    def _update_control_mode_ui(self):
        """根据当前控制模式更新位置控件和停止按钮。"""
        if threading.current_thread() is not self._main_thread:
            self.call_in_ui(self._update_control_mode_ui)
            return

        is_position = self.active_control_mode == "position"
        for scale in self.slider_widgets:
            scale.configure(state=tk.NORMAL if is_position else tk.DISABLED)

        titles = {
            "position": "关节位置控制（发送 CTRL_POS 数据）",
            "speed": "7 路有符号速度控制（CTRL_POS 已禁用）",
            "torque": "7 路有符号扭矩控制（CTRL_POS 已禁用）",
        }
        self.grp.configure(text=titles[self.active_control_mode])
        stop_state = (
            tk.NORMAL
            if self.hand is not None and not is_position
            else tk.DISABLED
        )
        self.btn_stop_direct.configure(state=stop_state)

    @staticmethod
    def _send_direct_stop(hand: AeroHand, mode: str):
        if mode == "speed":
            hand.ctrl_speeds([0] * 7)
        elif mode == "torque":
            hand.ctrl_torque([0] * 7)

    @staticmethod
    def _port_rank(port):
        identity = " ".join(
            str(value or "")
            for value in (port.device, port.description, port.manufacturer, port.hwid)
        ).lower()
        if (
            (port.vid, port.pid) == (0x303A, 0x1001)
            or "espressif" in identity
            or "usb jtag/serial debug unit" in identity
        ):
            return 0
        device = port.device.lower()
        if "ttyacm" in device:
            return 1
        if "ttyusb" in device:
            return 2
        if device.startswith("com"):
            return 3
        if "ttys" in device:
            return 20
        return 10

    def _refresh_ports(self):
        ports = sorted(list_ports.comports(), key=self._port_rank)
        devices = [port.device for port in ports]
        self.port_cmb["values"] = devices

        if not ports:
            self.port_var.set("")
            self.set_status("未检测到可用串口")
            return

        preferred = ports[0]
        current = self.port_var.get()
        if current not in devices or self._port_rank(preferred) == 0:
            self.port_var.set(preferred.device)

        if self._port_rank(preferred) == 0:
            self.set_status(f"已检测到 ESP32：{preferred.device}")
        else:
            self.set_status("未识别到 ESP32，请检查 USB 连接")

    def on_torque_control(self):
        if not self.hand:
            return
        if not self.signed_batch_control_available:
            messagebox.showerror(
                "固件版本过低",
                "当前 ESP32 固件不支持有符号扭矩控制，请先烧录最新固件。",
                parent=self,
            )
            return
        self.control_paused = True
        self.torque_values = [int(self.torque_slider_var.get() * 1000)] * 7
        self.active_control_mode = "torque"
        self._update_control_mode_ui()
        self.torque_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(6, 10))
        self.torque_slider.configure(state=tk.NORMAL)
        try:
            self.hand.ctrl_torque(list(self.torque_values))
            self.set_status("已进入扭矩控制模式，CTRL_POS 位置数据发送已停止")
        except Exception as e:
            self.log(f"[错误] 扭矩控制失败：{e}")
        finally:
            self.control_paused = False

    def _on_torque_slider(self, val):
        if not self.hand:
            return
        t_val = int(float(val) * 1000)
        self.torque_values = [t_val] * 7
        try:
            self.hand.ctrl_torque(list(self.torque_values))
            self.log(f"[发送] CTRL_TOR 扭矩值：{t_val}")
            self.set_status(f"扭矩已设置为 {t_val}")
        except Exception as e:
            self.log(f"[错误] 扭矩设置失败：{e}")
            self.set_status("扭矩设置失败")

    def disable_torque_control(self):
        self.torque_frame.pack_forget()
        self.on_stop_direct_control()

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
        candidate = None
        try:
            self.set_status(f"正在连接 {port}…")
            self.update_idletasks()
            candidate = AeroHand(port, baudrate=baud)
            capabilities = candidate.get_capabilities(timeout_s=2.0)

            # Match the sliders to the current hand pose before streaming so
            # connecting does not cause an unexpected jump to zero.
            current_joints = candidate.get_joint_positions_compact()
            if len(current_joints) != 7:
                raise RuntimeError("固件返回了无效的关节位置数据")
            normalized = []
            for index, value in enumerate(current_joints):
                lower = candidate.joint_lower_limits[index]
                upper = candidate.joint_upper_limits[index]
                ratio = (value - lower) / (upper - lower)
                normalized.append(max(0.0, min(1.0, ratio)))
            self.slider_values = normalized
            for var, value in zip(self.slider_vars, normalized):
                var.set(value)

            self.speed_values = [0] * 7
            self.torque_values = [0] * 7
            self.active_control_mode = "position"
            self.signed_batch_control_available = capabilities[
                "signed_batch_control"
            ]

            self.hand = candidate

            self.stop_event.clear()
            self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self.tx_thread.start()

            self.btn_connect.configure(state=tk.DISABLED)
            self.btn_disc.configure(state=tk.NORMAL)
            for b in (self.btn_zero, self.btn_homing, self.btn_setid, self.btn_trim,
                      self.btn_set_speed, self.btn_set_torque, self.btn_torque_control,
                      self.btn_get_pos, self.btn_get_vel, self.btn_get_cur, self.btn_get_temp, self.btn_get_all):
                b.configure(state=tk.NORMAL)
            self._update_control_mode_ui()

            if not self.signed_batch_control_available:
                self.btn_set_speed.configure(state=tk.DISABLED)
                self.btn_set_torque.configure(state=tk.DISABLED)
                self.btn_torque_control.configure(state=tk.DISABLED)
                self.log("[警告] 当前固件不支持 7 路有符号速度/扭矩控制，请烧录最新 firmware_new 固件。")

            protocol_version = capabilities["protocol_version"]
            self.set_status(f"已连接 ESP32（固件协议 v{protocol_version}）：{port}")
            self.log(f"[信息] ESP32 通信验证通过：{port}，波特率 {baud}")
            self.log(f"[信息] 固件协议版本：{protocol_version}")
            return True  # Success!
        except Exception as e:
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
            self.hand = None
            detail = (
                f"已打开串口 {port}，但未收到 Aero Hand 固件应答。\n\n"
                f"详细信息：{e}\n\n"
                "请确认选中的是 ESP32 串口，并且已烧录 firmware_new 固件。"
            )
            self.set_status("连接失败：ESP32 固件无应答")
            messagebox.showerror("连接失败", detail)
            return False  # Failure

    def on_disconnect(self):
        self._shutdown_serial()

    def _shutdown_serial(self):
        self.control_paused = True
        previous_mode = self.active_control_mode
        self.stop_event.set()
        if self.tx_thread and self.tx_thread.is_alive():
            try:
                self.tx_thread.join(timeout=0.5)
            except Exception:
                pass
        if self.hand:
            try:
                self._send_direct_stop(self.hand, previous_mode)
            except Exception:
                pass
            try:
                self.hand.close()
            except Exception:
                pass
        self.hand = None
        self.tx_thread = None
        self.active_control_mode = "position"
        self.signed_batch_control_available = False
        self.control_paused = False

        self.btn_connect.configure(state=tk.NORMAL)
        self.btn_disc.configure(state=tk.DISABLED)
        for b in (self.btn_zero, self.btn_homing, self.btn_setid, self.btn_trim,
                  self.btn_set_speed, self.btn_set_torque, self.btn_torque_control,
                  self.btn_get_pos, self.btn_get_vel, self.btn_get_cur, self.btn_get_temp, self.btn_get_all):
            b.configure(state=tk.DISABLED)
        self.torque_frame.pack_forget()
        self._update_control_mode_ui()
        self.set_status("未连接")
        self.log("[信息] 连接已断开")

    # ------------- TX streaming (CTRL_POS) -------------
    def _tx_loop(self):
        period = 1.0 / max(1e-3, self.tx_rate_hz)
        next_t = time.perf_counter()
        while not self.stop_event.is_set():
            if self.hand is not None and not self.control_paused:
                try:
                    if self.active_control_mode == "speed":
                        self.hand.ctrl_speeds(list(self.speed_values))
                    elif self.active_control_mode == "torque":
                        self.hand.ctrl_torque(list(self.torque_values))
                    else:
                        # Unnormalize to joint limits for CTRL_POS.
                        j_ll = self.hand.joint_lower_limits
                        j_ul = self.hand.joint_upper_limits
                        slider_values = list(self.slider_values)
                        joint_values = [
                            j_ll[i] + (j_ul[i] - j_ll[i]) * slider_values[i]
                            for i in range(7)
                        ]
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
        if not self.signed_batch_control_available:
            messagebox.showerror(
                "固件版本过低",
                "当前 ESP32 固件不支持 7 路有符号速度控制，请先烧录最新固件。",
                parent=self,
            )
            return
        values = self._ask_motor_values(
            title="设置 7 路电机速度",
            parameter_name="速度",
            current_values=self.speed_values,
            minimum=-32766,
            maximum=32766,
            direction_hint="正值=逆时针，负值=顺时针，0=停止",
        )
        if values is None:
            return

        self.control_paused = True
        hand = self.hand

        def worker():
            try:
                self.set_status("正在启用 7 路有符号速度控制…")
                hand.ctrl_speeds(values)
                self.speed_values = list(values)
                self.active_control_mode = "speed"
                self._update_control_mode_ui()
                summary = "，".join(
                    f"{index}={value}" for index, value in enumerate(values)
                )
                self.log(f"[发送] 7 路有符号速度：{summary}")
                self.set_status("7 路速度控制中：正值逆时针，负值顺时针")
            except Exception as e:
                self.log(f"[错误] 7 路速度控制失败：{e}")
                self.set_status("7 路电机速度控制失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_set_torque(self):
        if not self.hand:
            return
        if not self.signed_batch_control_available:
            messagebox.showerror(
                "固件版本过低",
                "当前 ESP32 固件不支持 7 路有符号扭矩控制，请先烧录最新固件。",
                parent=self,
            )
            return
        values = self._ask_motor_values(
            title="设置 7 路电机扭矩",
            parameter_name="扭矩",
            current_values=self.torque_values,
            minimum=-1000,
            maximum=1000,
            direction_hint="正值=逆时针，负值=顺时针，0=停止",
        )
        if values is None:
            return

        self.control_paused = True
        hand = self.hand

        def worker():
            try:
                self.set_status("正在启用 7 路有符号扭矩控制…")
                hand.ctrl_torque(values)
                self.torque_values = list(values)
                self.active_control_mode = "torque"
                self._update_control_mode_ui()
                summary = "，".join(
                    f"{index}={value}" for index, value in enumerate(values)
                )
                self.log(f"[发送] 7 路有符号扭矩：{summary}")
                self.set_status("7 路扭矩控制中：正值逆时针，负值顺时针")
            except Exception as e:
                self.log(f"[错误] 7 路扭矩控制失败：{e}")
                self.set_status("7 路电机扭矩控制失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_stop_direct_control(self):
        if not self.hand:
            return

        self.control_paused = True
        hand = self.hand
        previous_mode = self.active_control_mode

        def worker():
            try:
                self._send_direct_stop(hand, previous_mode)
                if previous_mode == "speed":
                    self.speed_values = [0] * 7
                elif previous_mode == "torque":
                    self.torque_values = [0] * 7
                self.active_control_mode = "position"
                self.call_in_ui(self.torque_frame.pack_forget)
                self._update_control_mode_ui()
                self.log("[发送] 7 路电机已停止，恢复 CTRL_POS 位置控制")
                self.set_status("已停止速度/扭矩控制，返回位置控制")
            except Exception as e:
                self.log(f"[错误] 停止 7 路控制失败：{e}")
                self.set_status("停止 7 路控制失败")
            finally:
                self.control_paused = False

        threading.Thread(target=worker, daemon=True).start()

    def on_zero_all(self):
        if not self.hand:
            return

        self.control_paused = True
        hand = self.hand
        previous_mode = self.active_control_mode
        self.slider_values = [0.0] * 7
        for var in self.slider_vars:
            var.set(0.0)

        def worker():
            try:
                self._send_direct_stop(hand, previous_mode)
                self.active_control_mode = "position"
                self.call_in_ui(self.torque_frame.pack_forget)
                self._update_control_mode_ui()
                joint_pos = list(hand.joint_lower_limits)
                hand.set_joint_positions(joint_pos)
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
            lower = self.hand.actuation_lower_limits
            upper = self.hand.actuation_upper_limits
            norm_pos = [
                round((pos[i] - lower[i]) / (upper[i] - lower[i]), 3)
                for i in range(7)
            ]
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
                    self.call_in_ui(lambda: messagebox.showinfo("成功", "固件烧录成功。"))
                else:
                    self.log(f"[烧录] esptool 退出码：{rc}")
                    self.call_in_ui(lambda rc=rc: messagebox.showerror("烧录失败", f"esptool 退出码：{rc}"))
            except Exception as e:
                self.log(f"[烧录错误] {e}")
                self.call_in_ui(lambda e=e: messagebox.showerror("烧录失败", str(e)))

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
                self.call_in_ui(_do_connect)
                if not done.wait(3.0): 
                    self.log("[烧录] 重新连接超时")
                    continue                   
                if result['ok']:
                    self.log("[烧录] 已重新连接 ✅")
                    break                     
            else:
                self.set_status("固件烧录后重新连接失败")
                self.call_in_ui(lambda: messagebox.showerror(
                    "重新连接失败",
                    "固件烧录完成，但无法重新连接串口。",
                ))

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
