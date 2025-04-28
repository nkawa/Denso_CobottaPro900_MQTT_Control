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

        self.button_ConnectRobot = \
            tk.Button(self.root, text="ConnectRobot", padx=5,
                      command=self.ConnectRobot)
        self.button_ConnectRobot.grid(row=0,column=0,padx=2,pady=10)

        self.button_ConnectMQTT = \
            tk.Button(self.root, text="ConnectMQTT", padx=5,
                             command=self.ConnectMQTT)
        self.button_ConnectMQTT.grid(row=0,column=1,padx=2,pady=10)

        self.button_EnableRobot = \
            tk.Button(self.root, text="EnableRobot", padx=5,
                      command=self.EnableRobot)
        self.button_EnableRobot.grid(row=1,column=0,padx=2,pady=10)

        self.button_DefaultPose = \
            tk.Button(self.root, text="DefaultPose", padx=5,
                      command=self.DefaultPose)
        self.button_DefaultPose.grid(row=1,column=1,padx=2,pady=10)

        self.button_TidyPose = \
            tk.Button(self.root, text="TidyPose", padx=5,
                      command=self.TidyPose)
        self.button_TidyPose.grid(row=1,column=2,padx=2,pady=10)

        self.button_ClearError = \
            tk.Button(self.root, text="ClearError", padx=5,
                      command=self.ClearError)
        self.button_ClearError.grid(row=1,column=3,padx=2,pady=10)

        self.button_StartMQTTControl = \
            tk.Button(self.root, text="StartMQTTControl", padx=5,
                      command=self.StartMQTTControl)
        self.button_StartMQTTControl.grid(row=2,column=0,padx=2,pady=10)
        
        self.button_StopMQTTControl = \
            tk.Button(self.root, text="StopMQTTControl", padx=5,
                      command=self.StopMQTTControl)
        self.button_StopMQTTControl.grid(row=2,column=1,padx=2,pady=10)

        self.button_ReleaseHand = \
            tk.Button(self.root, text="ReleaseHand", padx=5,
                      command=self.ReleaseHand)
        self.button_ReleaseHand.grid(row=2,column=2,padx=2,pady=10)

        self.log_monitor = scrolledtext.ScrolledText(
            self.root, width=70, height=60)
        self.log_monitor.grid(row=3,column=0,padx=10,pady=10,columnspan=7)

        self.update_log_monitor()

    def log_txt(self,str):
        self.text_log.insert(tk.END,str)

    def resetRobot(self):
        self.text_log.delete('1.0', 'end-1c')

    def ConnectRobot(self):
        self.pm.startControl()
        self.pm.startMonitor()

    def ConnectMQTT(self):
        self.pm.startRecvMQTT()

    def EnableRobot(self):
        self.pm.enable()
    
    def DefaultPose(self):
        self.pm.default_pose()
    
    def TidyPose(self):
        self.pm.tidy_pose()

    def ClearError(self):
        self.pm.clear_error()

    def StartMQTTControl(self):
        self.pm.start_mqtt_control()

    def StopMQTTControl(self):
        self.pm.stop_mqtt_control()

    def ReleaseHand(self):
        self.pm.release_hand()

    def update_log_monitor(self):
        if not self.pm.state_monitor:
            self.root.after(100, self.update_log_monitor)  # 100ms間隔で表示を更新
            return
        current_monitor_log = json.dumps(
            self.pm.get_current_monitor_log(), ensure_ascii=False)
        trim_logs = False
        # 古いログを全て削除
        if not trim_logs:
            self.log_monitor.delete("1.0", tk.END)
        self.log_monitor.insert(tk.END, current_monitor_log + "\n")  # ログを表示
        if trim_logs:
            # 古いログを一部削除
            self.trim_logs(self.log_monitor, max_lines=100)
            self.log_monitor.see(tk.END)  # 最新ログにスクロール
        self.root.after(100, self.update_log_monitor)  # 100ms間隔で表示を更新

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
