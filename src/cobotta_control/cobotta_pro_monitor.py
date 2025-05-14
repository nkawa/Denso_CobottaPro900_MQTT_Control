# Cobotta Pro の状態をモニタリングする

from typing import Literal
from paho.mqtt import client as mqtt
from denso_robot import DensoRobot

import datetime
import time

import os
import sys
import json
import psutil

import multiprocessing as mp

import numpy as np

from dotenv import load_dotenv

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.5.45")
HAND_IP = os.getenv("HAND_IP", "192.168.5.46")
ROBOT_UUID = os.getenv("ROBOT_UUID","cobotta-pro-real")
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")+"/"+ROBOT_UUID
MQTT_FORMAT = os.getenv("MQTT_FORMAT", "Denso-Cobotta-Pro-Control-IK")
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")
SAVE = os.getenv("SAVE", "true") == "true"

# 基本的に運用時には固定するパラメータ
t_intv = 0.008
use_hand = True
hand_mode: Literal["b-CAP", "xmlrpc"] = "xmlrpc"
save_state = SAVE
if save_state:
    save_path = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + "_status.jsonl"
    f = open(save_path, "w")

class Cobotta_Pro_MON:
    def __init__(self):
        pass

    def init_robot(self):
        self.robot = DensoRobot(
            host=ROBOT_IP,
            use_hand=use_hand,
            hand_mode=hand_mode,
            hand_host=HAND_IP,
        )
        self.robot.start()
        assert self.robot.wait_until_set_tool(timeout=60)
        if self.robot._use_hand:
            assert self.robot.setup_hand()

    def init_realtime(self):
        os_used = sys.platform
        process = psutil.Process(os.getpid())
        if os_used == "win32":  # Windows (either 32-bit or 64-bit)
            process.nice(psutil.REALTIME_PRIORITY_CLASS)
        elif os_used == "linux":  # linux
            rt_app_priority = 80
            param = os.sched_param(rt_app_priority)
            try:
                os.sched_setscheduler(0, os.SCHED_FIFO, param)
            except OSError:
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)

    def on_connect(self,client, userdata, flag, rc,proc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示

    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def connect_mqtt(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_start()   # 通信処理開始

    def monitor_start(self):
        last = 0
        last_error_monitored = 0
        while True:
            now = time.time()
            if last == 0:
                last = now
            if last_error_monitored == 0:
                last_error_monitored = now

            # TCP姿勢
            actual_tcp_pose = self.robot.get_current_pose()
            # 関節
            actual_joint = self.robot.get_current_joint()
            if MQTT_FORMAT == 'UR-realtime-control-MQTT':        
                joints = ['j1','j2','j3','j4','j5','j6']
                actual_joint_js = {
                    k: v for k, v in zip(joints, actual_joint)}
            elif MQTT_FORMAT == 'Denso-Cobotta-Pro-Control-IK':
                # 7要素送る必要があるのでダミーの[0]を追加
                actual_joint_js = {"joints": list(actual_joint) + [0]}
                # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
                actual_joint_js["joints"][4] = actual_joint_js["joints"][4] - 90
            else:
                raise ValueError
            
            # 型: 整数、単位: ms
            time_ms = int(now * 1000)
            actual_joint_js["time"] = time_ms
            # [X, Y, Z, RX, RY, RZ]: センサ値の力[N]とモーメント[Nm]
            forces = self.robot.ForceValue()
            actual_joint_js["forces"] = forces

            if use_hand:
                if hand_mode == "b-CAP":
                    width = self.robot.TwofgGetWidth()
                    force = self.robot.TwofgGetForce()
                else:
                    width = self.robot.TwofgGetWidthXmlRpc()
                    force = self.robot.TwofgGetForceXmlRpc()
            else:
                width = None
                force = None
            
            # モータがONか
            enabled = self.robot.is_enabled()
            actual_joint_js["enabled"] = enabled

            error = {}
            # スレーブモード中にエラー情報を取得しようとすると、
            # スレーブモードが切断される。
            # 本プロセスでロボットがスレーブモードでないと判断した直後に、
            # 別プロセスでスレーブモードに入る可能性があるので、
            # 通常モードの場合のみ呼び出す
            with self.slave_mode_lock:
                if self.pose[14] == 0:
                    actual_joint_js["servo_mode"] = False
                    errors = self.robot.get_cur_error_info_all()  
                    # 制御プロセスのエラー検出と方法が違うので、
                    # 直後は状態プロセスでエラーが検出されないことがある
                    # その場合は次のループに検出を持ち越す
                    if len(errors) > 0:
                        error = {"errors": errors}
                        # 自動復帰可能エラー
                        if self.robot.are_all_errors_stateless(errors):
                            error["auto_recoverable"] = True
                        # 復帰にユーザーの対応を求めるエラー
                        else:
                            error["auto_recoverable"] = False
                else:
                    actual_joint_js["servo_mode"] = True
            if self.pose[15] == 0:
                actual_joint_js["mqtt_control"] = "OFF"
            else:
                actual_joint_js["mqtt_control"] = "ON"

            if error:
                actual_joint_js["error"] = error

            self.pose[:len(actual_joint)] = actual_joint

            if now-last > 0.3:
                jss = json.dumps(actual_joint_js)
                self.client.publish(MQTT_ROBOT_STATE_TOPIC, jss)
                with self.monitor_lock:
                    self.monitor_dict.clear()
                    self.monitor_dict.update(actual_joint_js)
                last = now

            if save_state:
                datum = dict(
                    kind="state",
                    joint=actual_joint,
                    pose=actual_tcp_pose,
                    width=width,
                    force=force,
                    forces=forces,
                    error=error,
                    time=now,
                    enabled=enabled,
                )
                js = json.dumps(datum, ensure_ascii=False)
                f.write(js + "\n")

            t_elapsed = time.time() - now
            t_wait = t_intv - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)

    def run_proc(self, monitor_dict, monitor_lock, slave_mode_lock):
        self.sm = mp.shared_memory.SharedMemory("cobotta_pro")
        self.pose = np.ndarray((32,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.monitor_dict = monitor_dict
        self.monitor_lock = monitor_lock
        self.slave_mode_lock = slave_mode_lock

        self.init_realtime()
        self.init_robot()
        self.connect_mqtt()
        try:
            self.monitor_start()
        except KeyboardInterrupt:
            print("Stop! Cobotta Pro monitor")
            self.robot.disable()
            self.robot.stop()

if __name__ == '__main__':
    cp = Cobotta_Pro_MON()
    cp.init_realtime()
    cp.init_robot()
    cp.connect_mqtt()

    try:
        cp.monitor_start()
    except KeyboardInterrupt:
        print("Monitor Main Stopped")
        cp.robot.disable()
        cp.robot.stop()
