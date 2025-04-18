# Cobotta Proを制御する

from typing import Literal
import datetime
import time

import os
import sys
import json
import psutil

import multiprocessing as mp
import threading

import numpy as np
from dotenv import load_dotenv

package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
sys.path.append(package_dir)

from bcap_python.orinexception import ORiNException
from denso_robot import E_ACCEL_AUTO_RECOVERABLE_SET, E_AUTO_RECOVERABLE_SET, E_VEL_AUTO_RECOVERABLE_SET, DensoRobot, original_error_to_python_error

from filter import SMAFilter
from interpolate import DelayedInterpolator

# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.5.45")
HAND_IP = os.getenv("HAND_IP", "192.168.5.46")
SAVE = os.getenv("SAVE", "false") == "true"
MOVE = os.getenv("MOVE", "false") == "true"

# 基本的に運用時には固定するパラメータ
# 実際にロボットを制御するかしないか (VRとの結合時のデバッグ用)
move_robot = MOVE
# 平滑化の方法
# NOTE: 実際のVRコントローラとの結合時に、
# 遅延などを考慮すると改良が必要かもしれない。
# そのときのヒントとして残している
filter_kind: Literal[
    "original",
    "target",
    "state_and_target_diff",
    "moveit_servo_humble",
    "control_and_target_diff",
] = "original"
speed_limits = np.array([15, 15, 15, 15, 20, 15])
speed_limit_ratio = 0.9
# NOTE: 加速度制限。特に明確な値は調整していない
accel_limits = np.array([10000, 10000, 10000, 10000, 10000, 10000])
accel_limit_ratio = 0.9
stopped_velocity_eps = 1e-4
servo_mode = 0x202
use_interp = True
n_windows = 10
if filter_kind == "original":
    n_windows = 10
elif filter_kind == "target":
    n_windows = 100
if servo_mode == 0x102:
    t_intv = 0.004
else:
    t_intv = 0.008
n_windows *= int(0.008 / t_intv)
use_hand = True
hand_mode: Literal["b-CAP", "xmlrpc"] = "xmlrpc"
reset_default_state = True
# TCPが台の中心の上に来る初期位置
# default_joint = [4.7031, -0.6618, 105.5149, 0.0001, 75.1440, 94.7038]
# NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
default_joint = [159.3784, 10.08485, 122.90902, 151.10866, -43.20116 + 90, 20.69275]
abs_joint_limit = [270, 150, 150, 270, 150, 360]
abs_joint_limit = np.array(abs_joint_limit)
abs_joint_soft_limit = abs_joint_limit - 10

save_control = SAVE
if save_control:
    save_path = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + "_control.jsonl"
    f = open(save_path, "w")

class Cobotta_Pro_CON:
    def __init__(self):
        pass

    def init_robot(self):
        self.robot = DensoRobot(
            host=ROBOT_IP,
            default_servo_mode=servo_mode,
            use_hand=use_hand,
            hand_mode=hand_mode,
            hand_host=HAND_IP,
        )
        self.robot.start()
        self.robot.enable()
        if reset_default_state:
            self.default_joint = default_joint
            self.robot.move_joint_until_completion(self.default_joint)
            if hand_mode == "b-CAP":
                self.robot.TwofgRelease()
            else:
                self.robot.TwofgReleaseXmlRpc()
            time.sleep(1)
        self.robot.enter_servo_mode()

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

    def control_loop(self):
        self.last = 0
        print("[CNT]Start Main Loop")
        while self.loop:
            # NOTE: テスト用データなど、時間が経つにつれて
            # targetの値がstateの値によらずにどんどん
            # 変化していく場合は、以下で待ちすぎると
            # 制御値のもとになる最初のtargetの値が
            # stateから大きく離れるので、t_intv秒と短い時間だけ
            # 待っている。もしもっと待つと最初に
            # ガッとロボットが動いてしまう。実際のシステムでは
            # targetはstateに依存するのでまた別に考える

            # 現在情報を取得しているかを確認
            if self.pose[0:6].sum() == 0:
                time.sleep(t_intv)
                # print("[CNT]Wait for monitoring..")
                continue

            # 目標値を取得しているかを確認
            if self.pose[6:12].sum() == 0:
                time.sleep(t_intv)
                # print("[CNT]Wait for target..")
                continue 

            # 関節の状態値
            state = self.pose[:6].copy()

            now = time.time()
            if self.last == 0:
                print("[CNT]Starting to Control!",self.pose)
                self.last = now
                target = self.pose[6:12].copy()

                # j4の角度の制限値を±270に緩める用
                self.last_target = target

                # 目標値を遅延を許して極力線形補間するためのセットアップ
                if use_interp:
                    di = DelayedInterpolator(delay=0.1)
                    di.reset(now, target)
                    target_delayed = di.read(now, target)
                else:
                    target_delayed = target
                self.last_target_delayed = target_delayed
                
                # 移動平均フィルタのセットアップ（t_intv秒間隔）
                if filter_kind == "original":
                    self.last_control = state
                    _filter = SMAFilter(n_windows=n_windows)
                    _filter.reset(state)
                elif filter_kind == "target":
                    _filter = SMAFilter(n_windows=n_windows)
                    _filter.reset(target)
                elif filter_kind == "state_and_target_diff":
                    self.last_control = state
                    _filter = SMAFilter(n_windows=n_windows)
                    _filter.reset(state)
                elif filter_kind == "moveit_servo_humble":
                    _filter = SMAFilter(n_windows=n_windows)
                    _filter.reset(state)
                elif filter_kind == "control_and_target_diff":
                    self.last_control = state
                    _filter = SMAFilter(n_windows=n_windows)
                    _filter.reset(state)

                self.last_control_velocity = None
                continue

            # target_delayedは、delay秒前の目標値を前後の値を
            # 使って線形補間したもの
            target = self.pose[6:12].copy()
            target_raw = target

            # HACK: 現状、例えば、関節の角度が175度から180度をまたぎ185度に変化するとき、
            # VRの関節の角度の値は175度から-175度に変化するが、
            # ロボットの関節の角度の変化が-350度になり大きい
            # 代わりにロボットの関節の角度の変化が10度になるように規格化する
            # ロボットの関節の角度は185度になるが、ロボットの関節の角度の範囲内であれば
            # 問題ない。限界値を超えたら限界値を送るようにする
            # 本来はVR（IK）側で対応すべき
            target_norm = self.last_target + (target - self.last_target + 180) % 360 - 180
            target = target_norm
            self.last_target = target

            target_th = np.maximum(target, -abs_joint_soft_limit)
            if (target == target_th).any():
                pass
                # print("[CNT]: Warning: target reached minimum threshold")
            target_th = np.minimum(target_th, abs_joint_soft_limit)
            if (target == target_th).any():
                pass
                # print("[CNT]: Warning: target reached maximum threshold")
            target = target_th

            if use_interp:
                target_delayed = di.read(now, target)
            else:
                target_delayed = target

            # 平滑化
            if filter_kind == "original":
                # 成功している方法
                # 速度制限済みの制御値で平滑化をしており、
                # moveit servoなどでは見られない処理
                target_filtered = _filter.predict_only(target_delayed)
                target_diff = target_filtered - self.last_control
            elif filter_kind == "target":
                # 成功することもあるが平滑化窓を増やす必要あり
                # 状態値を無視した目標値の値をロボットに送る
                last_target_filtered = _filter.previous_filtered_measurement
                target_filtered = _filter.filter(target_delayed)
                target_diff = target_filtered - last_target_filtered
            elif filter_kind == "state_and_target_diff":
                # 失敗する
                # 状態値に目標値の差分を足したものを平滑化する
                # moveit servo (少なくともhumble版)ではこのようにしているが、
                # 速度がどんどん大きくなっていって（正のフィードバック）
                # 制限に引っかかる
                # stateを含む移動平均を取ると、stateが速度を持つと
                # その速度を保持し続けようとするので、そこに差分を足すと
                # どんどん加速していくのでは。
                # 遅延があることも影響しているかも。
                target_diff = target_delayed - self.last_target_delayed
                target_aligned = state + target_diff
                last_target_filtered = _filter.previous_filtered_measurement
                target_filtered = _filter.filter(target_aligned)
                target_diff = target_filtered - last_target_filtered
            elif filter_kind == "moveit_servo_humble":
                # 失敗する
                # 停止はしないがかなりゆっくり動き、目標軌跡も追従しなくなる
                # v = (target_filtered - state) / t_intv
                # target_filteredとstateの差は、
                # - 制御値を送ってからその値にstateがなるまで0.1s程度の遅延があること
                # - テストなどであらかじめ決まっているtargetを逐次送り、
                #   targetの速度がロボットの速度制限より大きいとき、
                #   targetがstateからどんどん離れていくこと
                # などの理由からt_intv秒で移動できる距離以上になってしまうため
                target_diff = target_delayed - self.last_target_delayed
                target_aligned = state + target_diff
                last_target_filtered = _filter.previous_filtered_measurement
                target_filtered = _filter.filter(target_aligned)
                target_diff = target_filtered - state
            elif filter_kind == "control_and_target_diff":
                # 失敗する
                # 速度制限にひっかかり途中停止する
                # 制御値に目標値の差分を足したものを平滑化する
                # 上記と同様に正のフィードバック的になっている
                # moveit_servo_mainの処理に近い
                target_diff = target_delayed - self.last_target_delayed
                target_aligned = self.last_control + target_diff
                last_target_filtered = _filter.previous_filtered_measurement
                target_filtered = _filter.filter(target_aligned)
                target_diff = target_filtered - last_target_filtered
            else:
                raise ValueError

            # 速度制限
            dt = now - self.last
            v = target_diff / dt
            ratio = np.abs(v) / (speed_limit_ratio * speed_limits)
            max_ratio = np.max(ratio)
            if max_ratio > 1:
                v /= max_ratio
            target_diff_speed_limited = v * dt

            # 加速度制限
            if self.last_control_velocity is not None:
                a = (v - self.last_control_velocity) / dt
                accel_ratio = np.abs(a) / (accel_limit_ratio * accel_limits)
                accel_max_ratio = np.max(accel_ratio)
                if accel_max_ratio > 1:
                    a /= accel_max_ratio
                v = self.last_control_velocity + a * dt
                target_diff_speed_limited = v * dt

            # 速度がしきい値より小さければ静止させ無駄なドリフトを避ける
            # NOTE: スレーブモードを落とさないためには前の速度が十分小さいとき (しきい値は不明) 
            # にしか静止させてはいけない
            if np.all(target_diff_speed_limited / dt < stopped_velocity_eps):
                target_diff_speed_limited = np.zeros_like(
                    target_diff_speed_limited)
                v = target_diff_speed_limited / dt

            self.last_control_velocity = v

            # 平滑化の種類による対応
            if filter_kind == "original":
                control = self.last_control + target_diff_speed_limited
                # 登録するだけ
                _filter.filter(control)
                self.last_control = control
            elif filter_kind == "target":
                control = last_target_filtered + target_diff_speed_limited
            elif filter_kind == "state_and_target_diff":
                control = last_target_filtered + target_diff_speed_limited
            elif filter_kind == "moveit_servo_humble":
                control = state + target_diff_speed_limited
            elif filter_kind == "control_and_target_diff":
                control = last_target_filtered + target_diff_speed_limited
                self.last_control = control
            else:
                raise ValueError

            if save_control:
                # 分析用データ保存
                datum = dict(
                    kind="target",
                    joint=target_raw.tolist(),
                    time=now,
                )
                js = json.dumps(datum)
                f.write(js + "\n")

                datum = dict(
                    kind="target_th",
                    joint=target_th.tolist(),
                    time=now,
                )
                js = json.dumps(datum)
                f.write(js + "\n")

                datum = dict(
                    kind="target_delayed",
                    joint=target_delayed.tolist(),
                    time=now,
                )
                js = json.dumps(datum)
                f.write(js + "\n")

                datum = dict(
                    kind="control",
                    joint=control.tolist(),
                    time=now,
                    max_ratio=max_ratio,
                )
                js = json.dumps(datum)
                f.write(js + "\n")

            if move_robot:
                try:
                    self.robot.move_joint_servo(control.tolist())
                except ORiNException as e:
                    if not self.robot.is_error_level_0(e):
                        return False

                if self.pose[13] == 1:
                    th1 = threading.Thread(target=self.send_grip)
                    th1.start()
                    self.pose[13] = 0

                if self.pose[13] == 2:
                    th2 = threading.Thread(target=self.send_release)
                    th2.start()
                    self.pose[13] = 0

                if servo_mode == 0x102:
                    t_elapsed = time.time() - now
                    t_wait = t_intv - t_elapsed
                    if t_wait > 0:
                        time.sleep(t_wait)

            else:
                # VRコントローラから位置は動かさずにハンドのみ動かしたい場合コメントアウトする
                # if self.pose[13] == 1:
                #     th1 = threading.Thread(target=self.send_grip)
                #     th1.start()
                #     self.pose[13] = 0

                # if self.pose[13] == 2:
                #     th2 = threading.Thread(target=self.send_release)
                #     th2.start()
                #     self.pose[13] = 0

                t_elapsed = time.time() - now
                t_wait = t_intv - t_elapsed
                if t_wait > 0:
                    time.sleep(t_wait)

            self.last = now
        return True

    def send_grip(self, force: int = 20):
        if hand_mode == "b-CAP":
            # NOTE: 呼ぶ度に目標の把持力は変更できるので
            # VRコントローラーからの入力で動的に把持力を
            # 変えることもできる (どういう仕組みを作るかは別)
            self.robot.TwofgStop()
            self.robot.TwofgGrip(force=force, waiting=0)
        else:
            self.robot.TwofgStopXmlRpc()
            self.robot.TwofgGripXmlRpc(waiting=0)

    def send_release(self):
        if hand_mode == "b-CAP":
            self.robot.TwofgStop()
            self.robot.TwofgRelease(waiting=0)
        else:
            self.robot.TwofgStopXmlRpc()
            self.robot.TwofgReleaseXmlRpc(waiting=0)

    def on_control_loop_error(self):
        # エラー検出
        self.pose[14] = 1
        while True:
            # モニタプロセスで自動復帰エラーと判定
            if self.pose[14] == 2:
                # 自動復帰を試行。失敗したらエラーでプログラム終了
                self.robot.recover_automatic_servo(max_trials=1)
                # エラー処理状況以外初期化
                self.pose[:14] = 0
                # 制御プロセスでエラー対応成功
                self.pose[14] = 0
                return True
            # 復帰にユーザーの対応を求めるエラーにユーザーからの対応を検出済みの場合
            # TODO: エラーの種類によって対応を変えるので、4以降の数字も増えるかも
            elif self.pose[14] == 4:
                # とりあえず何もしない
                raise NotImplementedError

    def run_proc(self):
        self.sm = mp.shared_memory.SharedMemory("cobotta_pro")
        self.pose = np.ndarray((16,), dtype=np.dtype("float32"), buffer=self.sm.buf)

        self.loop = True
        self.init_realtime()
        print("[CNT]:start realtime")
        self.init_robot()
        print("[CNT]:init robot")

        while True:
            if self.control_loop():
                sys.exit(0)
            else:            
                if self.on_control_loop_error():
                   continue
                else:
                    sys.exit(1)
