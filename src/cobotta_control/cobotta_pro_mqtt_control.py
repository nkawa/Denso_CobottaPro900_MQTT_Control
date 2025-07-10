# Cobotta ProをMQTTで制御する

import json
import logging
from paho.mqtt import client as mqtt
import multiprocessing as mp
import multiprocessing.shared_memory

from multiprocessing import Process

import os
from datetime import datetime
import numpy as np
import time
import sys

## ここでUUID を使いたい
import uuid

package_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(package_dir)
from cobotta_control.config import SHM_NAME, SHM_SIZE
from cobotta_pro_control import Cobotta_Pro_CON
from cobotta_pro_monitor import Cobotta_Pro_MON
from cobotta_pro_monitor_gui import run_joint_monitor_gui

from dotenv import load_dotenv

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
ROBOT_UUID = os.getenv("ROBOT_UUID","cobotta-pro-real")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","cobotta-pro-real")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "mgr")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
MQTT_FORMAT = os.getenv("MQTT_FORMAT", "Denso-Cobotta-Pro-Control-IK")
MQTT_MODE = os.getenv("MQTT_MODE", "metawork")

class Cobotta_Pro_MQTT:
    def __init__(self):
        self.gripState = False
        self.mqtt_ctrl_topic = None

    def on_connect(self,client, userdata, flag, rc):
        # ロボットのメタ情報の中身はとりあえず
        date = datetime.now().strftime('%c')
        if MQTT_MODE == "metawork":
            info = {
                "date": date,
                "device": {
                    "agent": "none",
                    "cookie": "none",
                },
                "devType": "robot",
                "type": ROBOT_MODEL,
                "version": "none",
                "devId": ROBOT_UUID,
            }
            self.client.publish(MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
            with self.mqtt_control_lock:
                info["topic_type"] = "mgr/register"
                info["topic"] = MQTT_MANAGE_TOPIC + "/register"
                self.mqtt_control_dict.clear()
                self.mqtt_control_dict.update(info)
            self.logger.info("publish to: " + MQTT_MANAGE_TOPIC + "/register")
            self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
            self.logger.info("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)
        else:
            self.logger.info("MQTT:Connected with result code " + str(rc),
                             "subscribe ctrl", MQTT_CTRL_TOPIC)
            self.mqtt_ctrl_topic = MQTT_CTRL_TOPIC
            self.client.subscribe(self.mqtt_ctrl_topic)

    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            self.logger.warning("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
        if msg.topic == self.mqtt_ctrl_topic:
            js = json.loads(msg.payload)

            if MQTT_FORMAT == "UR-realtime-control-MQTT":
                joints=['j1','j2','j3','j4','j5','j6']
                rot =[js[x]  for x in joints]    
                joint_q = [x for x in rot]
            elif MQTT_FORMAT == "Denso-Cobotta-Pro-Control-IK":
                # 7要素入っているが6要素でよいため
                rot = js["joints"][:6]
                joint_q = [x for x in rot]
                # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
                # NOTE(20250530): 現状はこれでうまくいくがVR側との意思疎通が必要
                # joint_q[4] = joint_q[4] + 90
                # NOTE(20250604): 一時的な対応。VR側で修正され次第削除。
                # joint_q[0] = joint_q[0] - 180
            else:
                raise ValueError
            self.pose[6:12] = joint_q 

            if "grip" in js:
                if js['grip']:
                    if not self.gripState:
                        self.gripState = True
                        self.pose[13] = 1

                else:
                    if self.gripState:
                        self.gripState = False
                        self.pose[13] = 2
            
            if "tool_change" in js:
                if self.pose[17] == 0:
                    tool = js["tool_change"]
                    self.pose[16] = 1
                    self.pose[17] = tool
            
            if "put_down_box" in js:
                if self.pose[21] == 0:
                    if js["put_down_box"]:
                        self.pose[16] = 1
                        self.pose[21] = 1

            self.pose[20] = 1
            with self.mqtt_control_lock:
                js["topic_type"] = "control"
                js["topic"] = msg.topic
                self.mqtt_control_dict.clear()
                self.mqtt_control_dict.update(js)

        elif msg.topic == MQTT_MANAGE_RCV_TOPIC:
            if MQTT_MODE == "metawork":
                js = json.loads(msg.payload)
                goggles_id = js["devId"]
                mqtt_ctrl_topic = MQTT_CTRL_TOPIC + "/" + goggles_id
                if mqtt_ctrl_topic != self.mqtt_ctrl_topic:
                    if self.mqtt_ctrl_topic is not None:
                        self.client.unsubscribe(self.mqtt_ctrl_topic)    
                    self.mqtt_ctrl_topic = mqtt_ctrl_topic
                self.client.subscribe(self.mqtt_ctrl_topic)
                self.logger.info("subscribe to: " + self.mqtt_ctrl_topic)
                with self.mqtt_control_lock:
                    js["topic_type"] = "dev"
                    js["topic"] = msg.topic
                    self.mqtt_control_dict.clear()
                    self.mqtt_control_dict.update(js)
        else:
            self.logger.warning("not subscribe msg" + msg.topic)

    def connect_mqtt(self):
        self.client = mqtt.Client()  
        # MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_forever()   # 通信処理開始

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("MQTT")
        if log_queue is not None:
            handler = logging.handlers.QueueHandler(log_queue)
        else:
            handler = logging.StreamHandler()
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

    def run_proc(self, mqtt_control_dict, mqtt_control_lock, log_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory("cobotta_pro")
        self.pose = np.ndarray((32,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.mqtt_control_dict = mqtt_control_dict
        self.mqtt_control_lock = mqtt_control_lock
        self.connect_mqtt()


class Cobotta_Pro_Debug:
    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("DBG")
        if log_queue is not None:
            handler = logging.handlers.QueueHandler(log_queue)
        else:
            handler = logging.StreamHandler()
        self.logger.addHandler(handler)
        self.logger.setLevel(logging.INFO)

    def run_proc(self, monitor_dict, log_queue):
        self.setup_logger(log_queue)
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.ar = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        while True:
            diff = self.ar[6:12]-self.ar[0:6]
            diff *=1000
            diff = diff.astype('int')
            self.logger.debug(f"State: {self.ar[0:6]}, Target: {self.ar[6:12]}")
            self.logger.debug(f"Diff: {diff}")
            sm = ",".join(str(int(round(x))) for x in self.ar)
            self.logger.debug(f"SharedMemory (rounded): {sm}")
            self.logger.debug(f"Monitor: {monitor_dict}")
            time.sleep(2)


class ProcessManager:
    def __init__(self):
        # mp.set_start_method('spawn')
        sz = SHM_SIZE * np.dtype('float32').itemsize
        try:
            self.sm = mp.shared_memory.SharedMemory(create=True,size = sz, name=SHM_NAME)
        except FileExistsError:
            self.sm = mp.shared_memory.SharedMemory(size = sz, name=SHM_NAME)
        # self.arの要素の説明
        # [0:6]: 関節の状態値
        # [6:12]: 関節の目標値
        # [13]: ハンドの目標値
        # [14]: 0: 必ず通常モード。1: 基本的にスレーブモード（通常モードになっている場合もある）
        # [15]: 0: mqtt_control実行中でない。1: mqtt_control実行中
        # [16]: 1: リアルタイム制御停止命令（mqtt_control停止命令ではないことに注意）
        # [17]: ツールチェンジの実行フラグ。0: 終了。0以外: 開始。次のツール番号
        # [18]: ツールチェンジ完了状態。0: 未定義。1: 成功。2: 失敗
        # [19]: 制御開始後の状態値の受信フラグ
        # [20]: 制御開始後の目標値の受信フラグ
        # [21]: 棚の上の箱を作業台に置くデモの実行フラグ。0: 終了。1: 開始
        # [22]: 棚の上の箱を作業台に置くデモの完了状態。0: 未定義。1: 成功。2: 失敗
        # [23]: 現在のツール番号
        self.ar = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf) # 共有メモリ上の Array
        self.ar[:] = 0
        self.manager = multiprocessing.Manager()
        self.monitor_dict = self.manager.dict()
        self.monitor_lock = self.manager.Lock()
        self.mqtt_control_dict = self.manager.dict()
        self.mqtt_control_lock = self.manager.Lock()
        self.slave_mode_lock = multiprocessing.Lock()
        self.main_pipe, self.control_pipe = multiprocessing.Pipe()
        self.state_recv_mqtt = False
        self.state_monitor = False
        self.state_control = False
        self.log_queue = multiprocessing.Queue()

    def startRecvMQTT(self):
        self.recv = Cobotta_Pro_MQTT()
        self.recvP = Process(
            target=self.recv.run_proc,
            args=(self.mqtt_control_dict,
                  self.mqtt_control_lock,
                  self.log_queue),
            name="MQTT-recv")
        self.recvP.start()
        self.state_recv_mqtt = True

    def startMonitor(self):
        self.mon = Cobotta_Pro_MON()
        self.monP = Process(
            target=self.mon.run_proc,
            args=(self.monitor_dict,
                  self.monitor_lock,
                  self.slave_mode_lock,
                  self.log_queue),
            name="Cobotta-Pro-monitor")
        self.monP.start()
        self.state_monitor = True

    def startControl(self):
        self.ctrl = Cobotta_Pro_CON()
        self.ctrlP = Process(
            target=self.ctrl.run_proc,
            args=(self.control_pipe, self.slave_mode_lock, self.log_queue),
            name="Cobotta-Pro-control")
        self.ctrlP.start()
        self.state_control = True

    def startDebug(self):
        self.debug = Cobotta_Pro_Debug()
        self.debugP = Process(
            target=self.debug.run_proc,
            args=(self.monitor_dict, self.log_queue),
            name="Cobotta-Pro-debug")
        self.debugP.start()

    def _send_command_to_control(self, command):
        self.main_pipe.send(command)

    def enable(self):
        self._send_command_to_control({"command": "enable"})

    def disable(self):
        self._send_command_to_control({"command": "disable"})

    def default_pose(self):
        self._send_command_to_control({"command": "default_pose"})

    def tidy_pose(self):
        self._send_command_to_control({"command": "tidy_pose"})

    def clear_error(self):
        self._send_command_to_control({"command": "clear_error"})

    def release_hand(self):
        self._send_command_to_control({"command": "release_hand"})

    def start_mqtt_control(self):
        self._send_command_to_control({"command": "start_mqtt_control"})

    def stop_mqtt_control(self):
        # mqtt_control中のみシグナルを出す
        if self.ar[15] == 1:
            self.ar[16] = 1

    def tool_change(self, tool_id: int):
        self.ar[17] = tool_id
        self._send_command_to_control({"command": "tool_change"})

    def jog_joint(self, joint, direction):
        self._send_command_to_control({"command": "jog_joint", "params": {"joint": joint, "direction": direction}})

    def jog_tcp(self, axis, direction):
        self._send_command_to_control({"command": "jog_tcp", "params": {"axis": axis, "direction": direction}})

    def demo_put_down_box(self):
        self._send_command_to_control({"command": "demo_put_down_box"})

    def get_current_monitor_log(self):
        with self.monitor_lock:
            monitor_dict = self.monitor_dict.copy()
        return monitor_dict
    
    def get_current_mqtt_control_log(self):
        with self.mqtt_control_lock:
            mqtt_control_dict = self.mqtt_control_dict.copy()
        return mqtt_control_dict

    def __del__(self):
        self.sm.close()
        self.sm.unlink()
        self.manager.shutdown()
        self.main_pipe.close()
        self.control_pipe.close()
