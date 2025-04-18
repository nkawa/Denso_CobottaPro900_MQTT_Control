# Cobotta ProをMQTTで制御する

import json
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
from cobotta_pro_control import Cobotta_Pro_CON
from cobotta_pro_monitor import Cobotta_Pro_MON

from dotenv import load_dotenv

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
MQTT_SERVER = os.getenv("MQTT_SERVER", "sora2.uclab.jp")
MQTT_CTRL_TOPIC = os.getenv("MQTT_CTRL_TOPIC", "control")
ROBOT_UUID = os.getenv("ROBOT_UUID","no-uuid")
ROBOT_MODEL = os.getenv("ROBOT_MODEL","no-model")
MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "dev")
MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev")+"/"+ROBOT_UUID
MQTT_FORMAT = os.getenv("MQTT_FORMAT", "UR-realtime-control-MQTT")
MQTT_MODE = os.getenv("MQTT_MODE", "local")

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
                "type": "cobotta-pro-real",
                "version": "none",
                "devId": ROBOT_UUID,
            }
            self.client.publish(MQTT_MANAGE_TOPIC + "/register", json.dumps(info))
            print("publish to: " + MQTT_MANAGE_TOPIC + "/register")
            self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)
            print("subscribe to: " + MQTT_MANAGE_RCV_TOPIC)
        else:
            print("MQTT:Connected with result code " + str(rc), "subscribe ctrl", MQTT_CTRL_TOPIC)
            self.mqtt_ctrl_topic = MQTT_CTRL_TOPIC
            self.client.subscribe(self.mqtt_ctrl_topic)

    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
        if msg.topic == self.mqtt_ctrl_topic:
            js = json.loads(msg.payload)

            if MQTT_FORMAT == "UR-realtime-control-MQTT":
                joints=['j1','j2','j3','j4','j5','j6']
                rot =[js[x]  for x in joints]    
            elif MQTT_FORMAT == "Denso-Cobotta-Pro-Control-IK":
                # 7要素入っているが6要素でよいため
                rot = js["joints"][:6]
            else:
                raise ValueError
            joint_q = [x for x in rot]
            # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
            joint_q[4] = joint_q[4] + 90
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

        elif msg.topic == MQTT_MANAGE_RCV_TOPIC:
            if MQTT_MODE == "metawork":
                js = json.loads(msg.payload)
                goggles_id = js["devId"]
                print(f"Connected to goggles: {goggles_id}")
                self.mqtt_ctrl_topic = MQTT_CTRL_TOPIC + "/" + goggles_id
                self.client.subscribe(self.mqtt_ctrl_topic)
                print("subscribe to: " + self.mqtt_ctrl_topic)
        else:
            print("not subscribe msg", msg.topic)

    def connect_mqtt(self):
        self.client = mqtt.Client()  
        # MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect(MQTT_SERVER, 1883, 60)
        self.client.loop_forever()   # 通信処理開始

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("cobotta_pro")
        self.pose = np.ndarray((16,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.connect_mqtt()

class ProcessManager:
    def __init__(self):
        # mp.set_start_method('spawn')
        sz = 32* np.dtype('float').itemsize
        try:
            self.sm = mp.shared_memory.SharedMemory(create=True,size = sz, name='cobotta_pro')
        except FileExistsError:
            self.sm = mp.shared_memory.SharedMemory(size = sz, name='cobotta_pro')
        self.ar = np.ndarray((12,), dtype=np.dtype("float32"), buffer=self.sm.buf) # 共有メモリ上の Array

    def startRecvMQTT(self):
        self.recv = Cobotta_Pro_MQTT()
        self.recvP = Process(target=self.recv.run_proc, args=(),name="MQTT-recv")
        self.recvP.start()

    def startMonitor(self):
        self.mon = Cobotta_Pro_MON()
        self.monP = Process(target=self.mon.run_proc, args=(),name="Cobotta-Pro-monitor")
        self.monP.start()

    def startControl(self):
        self.ctrl = Cobotta_Pro_CON()
        self.ctrlP = Process(target=self.ctrl.run_proc, args=(),name="Cobotta-Pro-control")
        self.ctrlP.start()
 
    def checkSM(self):
        while True:
            diff = self.ar[6:]-self.ar[:6]
            diff *=1000
            diff = diff.astype('int')
            print(self.ar[:6],self.ar[6:])
            print(diff)
            time.sleep(2)
    

if __name__ == '__main__':
    pm = ProcessManager()
    try:
        print("Monitor!")
        pm.startMonitor()
        print("MQTT!")
        pm.startRecvMQTT()
        print("Control")
        pm.startControl()
        print("Check!")
        pm.checkSM()
    except KeyboardInterrupt:
        print("Stop!")
        # self.sm.close()
        # self.sm.unlink()
