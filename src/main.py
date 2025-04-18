
from cobotta_control.cobotta_pro_mqtt_control import ProcessManager

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
