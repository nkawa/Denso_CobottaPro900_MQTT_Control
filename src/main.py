import json
import tkinter as tk
from tkinter import scrolledtext

from cobotta_control.cobotta_pro_mqtt_control import ProcessManager


class MQTTWin:
    def __init__(self, root):
        self.pm = ProcessManager()
        self.pm.startDebug()
 
        self.root = root
        self.root.title("MQTT-CobottaPro900 Controller")
        self.root.geometry("600x800")

        # NOTE: デモ用
        self.tool_id = 1
        
        for col in range(4):
            self.root.grid_columnconfigure(col, weight=1, uniform="equal")
        
        self.button_ConnectRobot = \
            tk.Button(self.root, text="ConnectRobot", padx=5,
                      command=self.ConnectRobot, state="normal")
        self.button_ConnectRobot.grid(row=0,column=0,padx=2,pady=10,sticky="ew")

        self.button_ConnectMQTT = \
            tk.Button(self.root, text="ConnectMQTT", padx=5,
                             command=self.ConnectMQTT, state="normal")
        self.button_ConnectMQTT.grid(row=0,column=1,padx=2,pady=10,sticky="ew")

        self.button_EnableRobot = \
            tk.Button(self.root, text="EnableRobot", padx=5,
                      command=self.EnableRobot, state="disabled")
        self.button_EnableRobot.grid(row=1,column=0,padx=2,pady=10,sticky="ew")

        self.button_DisableRobot = \
            tk.Button(self.root, text="DisableRobot", padx=5,
                      command=self.DisableRobot, state="disabled")
        self.button_DisableRobot.grid(row=1,column=1,padx=2,pady=10,sticky="ew")

        self.button_ReleaseHand = \
            tk.Button(self.root, text="ReleaseHand", padx=5,
                      command=self.ReleaseHand, state="disabled")
        self.button_ReleaseHand.grid(row=1,column=2,padx=2,pady=10,sticky="ew")

        self.button_DefaultPose = \
            tk.Button(self.root, text="DefaultPose", padx=5,
                      command=self.DefaultPose, state="disabled")
        self.button_DefaultPose.grid(row=2,column=0,padx=2,pady=10,sticky="ew")

        self.button_TidyPose = \
            tk.Button(self.root, text="TidyPose", padx=5,
                      command=self.TidyPose, state="disabled")
        self.button_TidyPose.grid(row=2,column=1,padx=2,pady=10,sticky="ew")

        self.button_ClearError = \
            tk.Button(self.root, text="ClearError", padx=5,
                      command=self.ClearError, state="disabled")
        self.button_ClearError.grid(row=2,column=2,padx=2,pady=10,sticky="ew")

        self.button_StartMQTTControl = \
            tk.Button(self.root, text="StartMQTTControl", padx=5,
                      command=self.StartMQTTControl, state="disabled")
        self.button_StartMQTTControl.grid(
            row=3,column=0,padx=2,pady=10,sticky="ew")
        
        self.button_StopMQTTControl = \
            tk.Button(self.root, text="StopMQTTControl", padx=5,
                      command=self.StopMQTTControl, state="disabled")
        self.button_StopMQTTControl.grid(
            row=3,column=1,padx=2,pady=10,sticky="ew")

        self.button_ToolChange = \
            tk.Button(self.root, text="ToolChange", padx=5,
                      command=self.ToolChange, state="disabled")
        self.button_ToolChange.grid(
            row=3,column=2,padx=2,pady=10,sticky="ew")

        self.frame_enabled = tk.Frame(self.root)
        self.frame_enabled.grid(row=1,column=3,padx=2,pady=10,sticky="w")
        self.canvas_enabled = \
            tk.Canvas(self.frame_enabled, width=10, height=10)
        self.canvas_enabled.pack(side="left",padx=10)
        self.light_enabled = \
            self.canvas_enabled.create_oval(1, 1, 9, 9, fill="gray")
        self.label_enabled = \
            tk.Label(self.frame_enabled, text="Enabled")
        self.label_enabled.pack(side="left",padx=2)

        self.frame_error = tk.Frame(self.root)
        self.frame_error.grid(row=2,column=3,padx=2,pady=10,sticky="w")
        self.canvas_error = \
            tk.Canvas(self.frame_error, width=10, height=10)
        self.canvas_error.pack(side="left",padx=10)
        self.light_error = \
            self.canvas_error.create_oval(1, 1, 9, 9, fill="gray")
        self.label_error = \
            tk.Label(self.frame_error, text="Error")
        self.label_error.pack(side="left",padx=2)

        self.frame_mqtt_control = tk.Frame(self.root)
        self.frame_mqtt_control.grid(row=3,column=3,padx=2,pady=10,sticky="w")
        self.canvas_mqtt_control = \
            tk.Canvas(self.frame_mqtt_control, width=10, height=10)
        self.canvas_mqtt_control.pack(side="left",padx=10)
        self.light_mqtt_control = \
            self.canvas_mqtt_control.create_oval(1, 1, 9, 9, fill="gray")
        self.label_mqtt_control = \
            tk.Label(self.frame_mqtt_control, text="MQTTControl")
        self.label_mqtt_control.pack(side="left",padx=2)

        self.log_monitor = scrolledtext.ScrolledText(
            self.root, width=70, height=60)
        self.log_monitor.grid(row=4,column=0,padx=10,pady=10,columnspan=7)
        
        self.update_monitor()

    def ConnectRobot(self):
        if self.pm.state_control and self.pm.state_monitor:
            return
        self.pm.startControl()
        self.pm.startMonitor()
        self.button_ConnectRobot.config(state="disabled")
        self.button_ClearError.config(state="normal")
        self.button_DefaultPose.config(state="normal")
        self.button_DisableRobot.config(state="normal")
        self.button_EnableRobot.config(state="normal")
        self.button_ReleaseHand.config(state="normal")
        self.button_TidyPose.config(state="normal")
        self.button_ToolChange.config(state="normal")
        if self.pm.state_recv_mqtt:
            self.button_StartMQTTControl.config(state="normal")
            self.button_StopMQTTControl.config(state="normal")

    def ConnectMQTT(self):
        if self.pm.state_recv_mqtt:
            return
        self.pm.startRecvMQTT()
        self.button_ConnectMQTT.config(state="disabled")
        self.button_ClearError.config(state="normal")
        self.button_DefaultPose.config(state="normal")
        self.button_DisableRobot.config(state="normal")
        self.button_EnableRobot.config(state="normal")
        self.button_ReleaseHand.config(state="normal")
        self.button_TidyPose.config(state="normal")
        if self.pm.state_control and self.pm.state_monitor:
            self.button_StartMQTTControl.config(state="normal")
            self.button_StopMQTTControl.config(state="normal")

    def EnableRobot(self):
        if not self.pm.state_control:
            return
        self.pm.enable()

    def DisableRobot(self):
        if not self.pm.state_control:
            return
        self.pm.disable()

    def DefaultPose(self):
        if not self.pm.state_control:
            return
        self.pm.default_pose()
    
    def TidyPose(self):
        if not self.pm.state_control:
            return
        self.pm.tidy_pose()

    def ClearError(self):
        if not self.pm.state_control:
            return
        self.pm.clear_error()

    def StartMQTTControl(self):
        if ((not self.pm.state_control) or
            (not self.pm.state_monitor) or
            (not self.pm.state_recv_mqtt)):
            return
        self.pm.start_mqtt_control()

    def StopMQTTControl(self):
        if ((not self.pm.state_control) or
            (not self.pm.state_monitor) or
            (not self.pm.state_recv_mqtt)):
            return
        self.pm.stop_mqtt_control()

    def ReleaseHand(self):
        if not self.pm.state_control:
            return
        self.pm.release_hand()

    def ToolChange(self):
        if not self.pm.state_control:
            return
        # NOTE: 開発用
        self.tool_id = 2 if (not getattr(self, "tool_id") or self.tool_id == 1) else 1
        self.pm.tool_change(self.tool_id)

    def update_monitor(self):
        if not self.pm.state_monitor:
            self.root.after(100, self.update_monitor)  # 100ms間隔で表示を更新
            return
        log = self.pm.get_current_monitor_log()
        color = "lime" if log.get("enabled") else "gray"
        self.canvas_enabled.itemconfig(self.light_enabled, fill=color)
        color = "lime" if log.get("mqtt_control") == "ON" else "gray"
        self.canvas_mqtt_control.itemconfig(
            self.light_mqtt_control, fill=color)
        color = "red" if "error" in log else "gray"
        self.canvas_error.itemconfig(self.light_error, fill=color)
        log_str = json.dumps(log, ensure_ascii=False)
        trim_logs = False
        # 古いログを全て削除
        if not trim_logs:
            self.log_monitor.delete("1.0", tk.END)
        self.log_monitor.insert(tk.END, log_str + "\n")  # ログを表示
        if trim_logs:
            # 古いログを一部削除
            self.trim_logs(self.log_monitor, max_lines=100)
            self.log_monitor.see(tk.END)  # 最新ログにスクロール
        self.root.after(100, self.update_monitor)  # 100ms間隔で表示を更新

    def trim_logs(self, log, max_lines: int) -> None:
        """最大行数を超えたログを削除。"""
        current_lines = int(log.index('end-1c').split('.')[0])  # 現在の行数を取得
        if current_lines > max_lines:
            excess_lines = current_lines - max_lines
            self.log_area.delete("1.0", f"{excess_lines}.0")  # 超過分の行を削除

root = tk.Tk()
mqwin = MQTTWin(root)
mqwin.root.lift()
root.mainloop()
