
import datetime
import logging
import json
import logging.handlers
import os
import queue
import time
import multiprocessing
import multiprocessing.shared_memory
from typing import Optional

import numpy as np

from cobotta_control.cobotta_pro_mqtt_control import ProcessManager
from cobotta_control.config import SHM_NAME, SHM_SIZE
from cobotta_control.tools import tool_infos


tool_ids = [tool_info["id"] for tool_info in tool_infos]


class MicrosecondFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        dt = datetime.datetime.fromtimestamp(record.created)
        if datefmt:
            s = dt.strftime(datefmt)
            # %f をマイクロ秒で置換
            s = s.replace('%f', f"{dt.microsecond:06d}")
            return s
        else:
            return super().formatTime(record, datefmt)


class MQTTTargetReplayer:
    def __init__(
        self,
        target_path: str,
        log_dir: str,
        use_joint_monitor_plot: bool = False,
        start: Optional[float] = None,
        stop: Optional[float] = None,
    ):
        self.target_path = target_path
        self.start = start
        self.stop = stop
        self.use_joint_monitor_plot = use_joint_monitor_plot
        self.pm = ProcessManager()
        log_queue = self.pm.log_queue
        self.logging_dir = self.get_logging_dir(log_dir)
        self.setup_logging(log_queue=log_queue, logging_dir=self.logging_dir)
        self.setup_logger(log_queue=log_queue)
        self.gui_log_queue = queue.Queue()
        self.logger.info("Starting Process!")

    def get_logging_dir(self, log_dir: str = "log") -> str:
        now = datetime.datetime.now()
        os.makedirs(log_dir, exist_ok=True)
        date_str = now.strftime("%Y-%m-%d")
        os.makedirs(os.path.join(log_dir, date_str), exist_ok=True)
        time_str = now.strftime("%H-%M-%S")
        logging_dir = os.path.join(log_dir, date_str, time_str)
        os.makedirs(logging_dir, exist_ok=True)
        return logging_dir

    def setup_logging(
        self,
        log_queue: Optional[multiprocessing.Queue] = None,
        logging_dir: Optional[str] = None,
    ) -> None:
        """複数プロセスからのログを集約する方法を設定する"""
        if logging_dir is None:
            logging_dir = self.get_logging_dir()
        handlers = [
            logging.FileHandler(os.path.join(logging_dir, "log.txt")),
            logging.StreamHandler(),
        ]
        formatter = MicrosecondFormatter(
            "[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S.%f",
        )
        for handler in handlers:
            handler.setFormatter(formatter)
        self.listener = logging.handlers.QueueListener(log_queue, *handlers)
        # listener.startからroot.mainloopまでのわずかな間は、
        # GUIの更新がないが、root.mainloopが始まると溜まっていたログも表示される
        self.listener.start()

    def setup_logger(
        self, log_queue: Optional[multiprocessing.Queue] = None,
    ) -> None:
        """GUIプロセスからのログの設定"""
        self.logger = logging.getLogger("Replayer")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def _replay(self):
        self.logger.info(f"Replaying from {self.target_path}")
        self.sm = multiprocessing.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        try:
            with open(self.target_path, "r") as f:
                i = 0
                j = 0
                ts = []
                joints = []
                for line in f:
                    js = json.loads(line)
                    if js["kind"] != "target":    
                        continue
                    t = js["time"]
                    joint = js["joint"]
                    if i == 0:         
                        t0 = t
                    i += 1
                    t = t - t0
                    if self.start is not None and t < self.start:
                        continue
                    if self.stop is not None and t > self.stop:
                        break
                    if j == 0:
                        t_start = t
                    j += 1
                    t = t - t_start
                    ts.append(t)
                    joints.append(joint)
            self.logger.info("Move to initial position")
            self.pm.move_joint(joints[0], wait=True)
            self.logger.info("Start replay")
            self.pm.start_mqtt_control()
            # スレーブモード切り替え処理などで時間がかかるので待つ
            time.sleep(3)
            t0 = time.time()
            i = 0
            while True:
                t = time.time() - t0
                if t >= ts[i]:
                    self.pose[6:12] = joints[i]
                    i += 1
                if i >= len(ts):
                    break
                self.pose[20] = 1
            self.logger.info("Finish replay")
            self.pm.stop_mqtt_control()
        except Exception as e:
            self.logger.error(f"Error during replay: {e}")

    def run(self):
        self.pm.startControl(logging_dir=self.logging_dir)
        self.pm.startMonitor(logging_dir=self.logging_dir, disable_mqtt=True)
        if self.use_joint_monitor_plot:
            self.pm.startMonitorGUI()
        # self.pm.startRecvMQTT()
        self.pm.enable()
        self._replay()
        self.pm.disable()
        self.sm.close()
        self.pm.stop_all_processes()
        # TODO: Hanging here
        self.logger.info("Process stopped (except logging)")
        self.listener.stop()
        self.handler.close()
        logging.shutdown()


if __name__ == '__main__':
    # Freeze Support for Windows
    multiprocessing.freeze_support()


    # NOTE: 現在ロボットに付いているツールが何かを管理する方法がないので
    # ロボット制御コードの使用者に指定してもらう
    # ツールによっては、ツールとの通信が不要なものがあるので、通信の成否では判定できない
    # 現在のツールの状態を常にファイルに保存しておき、ロボット制御コードを再起動するときに
    # そのファイルを読み込むようにすれば管理はできるが、エラーで終了したときに
    # ファイルの情報が正確かいまのところ保証できないので、指定してもらう
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--target-path",
        type=str,
        required=True,
        help="再生する目標値ファイルのパス",
    )
    parser.add_argument(
        "--log-dir",
        type=str,
        default="log_replay",
        help="ログを保存するディレクトリのパス",
    )
    parser.add_argument(
        "--tool-id",
        type=int,
        required=True,
        choices=tool_ids,
        help="現在ロボットに付いているツールのID",
    )
    parser.add_argument(
        "--use-joint-monitor-plot",
        action="store_true",
        help="関節角度のモニタープロットを使用する",
    )
    parser.add_argument(
        "--start",
        type=float,
        default=None,
        help="再生開始時間（秒）。指定しない場合はファイルの最初から再生する",
    )
    parser.add_argument(
        "--stop",
        type=float,
        default=None,
        help="再生終了時間（秒）。指定しない場合はファイルの最後まで再生する",
    )
    args = parser.parse_args()
    kwargs = vars(args)
    import os
    # HACK: コードの変化を少なくするため、
    # ロボット制御プロセスに引数で渡すのではなく環境変数で渡す
    os.environ["TOOL_ID"] = str(kwargs.pop("tool_id"))

    replayer = MQTTTargetReplayer(**kwargs)
    replayer.run()
