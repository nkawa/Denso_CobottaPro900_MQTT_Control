# Cobotta Proを制御する

import logging
import queue
from typing import Any, Dict, List, Literal, TextIO, Tuple
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

from cobotta_control.config import SHM_NAME, SHM_SIZE, ABS_JOINT_LIMIT, T_INTV

package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
sys.path.append(package_dir)

from bcap_python.orinexception import HResult, ORiNException
from denso_robot import E_ACCEL_AUTO_RECOVERABLE_SET, E_AUTO_RECOVERABLE_SET, E_VEL_AUTO_RECOVERABLE_SET, DensoRobot, original_error_to_python_error

from filter import SMAFilter
from interpolate import DelayedInterpolator
from cobotta_control.tools import tool_infos, tool_classes, tool_base


# パラメータ
load_dotenv(os.path.join(os.path.dirname(__file__),'.env'))
ROBOT_IP = os.getenv("ROBOT_IP", "192.168.5.45")
HAND_IP = os.getenv("HAND_IP", "192.168.5.46")
SAVE = os.getenv("SAVE", "true") == "true"
MOVE = os.getenv("MOVE", "true") == "true"

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
speed_limits = np.array([240, 200, 240, 300, 300, 475])
speed_limit_ratio = 0.35
# NOTE: 加速度制限。スマートTPの最大加速度設定は単位が[rev/s^2]だが、[deg/s^2]とみなして、
# その値をここで設定すると、エラーが起きにくくなる (観測範囲でエラーがなくなった)
accel_limits = np.array([4040, 4033.33, 4040, 5050, 5050, 4860])
accel_limit_ratio = 0.35
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
    t_intv = T_INTV
n_windows *= int(0.008 / t_intv)
reset_default_state = True
default_joints = {
    # TCPが台の中心の上に来る初期位置
    "tidy": [4.7031, -0.6618, 105.5149, 0.0001, 75.1440, 94.7100],
    # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
    "vr": [159.3784, 10.08485, 122.90902, 151.10866, -43.20116 + 90, 20.69275],
    # NOTE: 2025/04/18 19:25の新しい位置?VRとの対応がおかしい気がする
    # 毎回の値も[0, 0, 0, 0, 0, 0]が飛んでくる気がする
    "vr2": [115.55677, 5.86272, 135.70465, 110.53529, -15.55474 + 90, 35.59977],
    # NOTE: 2025/05/30での新しい位置
    "vr3": [113.748, 5.645, 136.098, 109.059, 75.561, 35.82],
    # NOTE: 2025/06/05 での新しい位置
    "vr4": [-46.243, 10.258, 128.201, 125.629, 62.701, 32.618],
    "vr5": [-66.252, 5.645, 136.098, 109.059, 75.561, 35.82],
}
abs_joint_limit = ABS_JOINT_LIMIT
abs_joint_limit = np.array(abs_joint_limit)
abs_joint_soft_limit = abs_joint_limit - 10
# 外部速度。単位は%
speed_normal = 20
speed_tool_change = 2
# 目標値が状態値よりこの制限より大きく乖離した場合はロボットを停止させる
# 設定値は典型的なVRコントローラの動きから決定した
target_state_abs_joint_diff_limit = [30, 30, 40, 40, 40, 60]

save_control = SAVE


class StopWatch:
    def __init__(self):
        self.start_t = None
        self.last_t = None
        self.last_msg = None
        self.laps = []

    def start(self, msg: str = "") -> None:
        t = time.perf_counter()
        self.start_t = t
        self.last_t = t
        self.last_msg = msg
        self.laps = []

    def lap(self, msg: str = "") -> None:
        if ((self.start_t is None) or
            (self.last_t is None) or
            (self.last_msg is None)):
            raise RuntimeError("StopWatch has not been started.")
        t = time.perf_counter()
        self.laps.append({
            "msg": self.last_msg,
            "lap": t - self.last_t,
            "split": t - self.start_t,
        })
        self.last_t = t
        self.last_msg = msg

    def stop(self) -> None:
        self.lap()
        self.start_t = None
        self.last_t = None
        self.last_msg = None
    
    def summary(self) -> str:
        s = "StopWatch summary:\n"
        s += "| No. | Lap (ms) | Split (ms) | Message |\n"
        s += "| - | - | - | - |\n"
        for i, lap in enumerate(self.laps):
            s += f"| {i} | {lap['lap']*1000:.3f} | {lap['split']*1000:.3f} | {lap['msg']} |\n"
        return s


class Cobotta_Pro_CON:
    def __init__(self):
        self.default_joint = default_joints["vr5"]
        self.tidy_joint = default_joints["tidy"]

    def init_robot(self):
        try:
            self.robot = DensoRobot(
                host=ROBOT_IP,
                default_servo_mode=servo_mode,
                logger=self.robot_logger,
            )
            self.robot.start()
            self.robot.clear_error()
            self.robot.take_arm()
            self.robot.SetAreaEnabled(0, True)
            self.pose[31] = 1
            tool_id = int(os.environ["TOOL_ID"])
            self.find_and_setup_hand(tool_id)
        except Exception as e:
            self.logger.error("Error in initializing robot: ")
            self.logger.error(f"{self.robot.format_error(e)}")

    def find_and_setup_hand(self, tool_id):
        connected = False
        tool_info = self.get_tool_info(tool_infos, tool_id)
        name = tool_info["name"]
        hand = tool_classes[name]()
        if tool_id != -1:
            connected = hand.connect_and_setup()
            if not connected:
                raise ValueError(f"Failed to connect to hand: {name}")
        else:
            hand = None
        self.hand_name = name
        self.hand = hand
        self.tool_id = tool_id
        self.pose[23] = tool_id
        if tool_id != -1:
            self.robot.SetToolDef(
                tool_info["id_in_robot"], tool_info["tool_def"])
        self.robot.set_tool(tool_info["id_in_robot"])

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
                self.logger.warning("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                self.logger.info("Process real-time priority set to: %u" % rt_app_priority)

    def control_loop(self, f: TextIO | None = None) -> bool:
        """リアルタイム制御ループ"""
        self.last = 0
        self.logger.info("Start Control Loop")
        # 状態値が最新の値になるようにする
        time.sleep(1)
        self.pose[19] = 0
        self.pose[20] = 0
        target_stop = None
        sw = StopWatch()
        last_target = None

        while True:
            sw.start("Get shared memory")
            now = time.time()

            # NOTE: テスト用データなど、時間が経つにつれて
            # targetの値がstateの値によらずにどんどん
            # 変化していく場合は、以下で待ちすぎると
            # 制御値のもとになる最初のtargetの値が
            # stateから大きく離れるので、t_intv秒と短い時間だけ
            # 待っている。もしもっと待つと最初に
            # ガッとロボットが動いてしまう。実際のシステムでは
            # targetはstateに依存するのでまた別に考える
            stop = self.pose[16]

            # 現在情報を取得しているかを確認
            if self.pose[19] != 1:
                time.sleep(t_intv)
                # self.logger.info("Wait for monitoring")
                # 取得する前に終了する場合即時終了可能
                if stop:
                    return True
                # ロボットにコマンドを送る前は、非常停止が押されているかを
                # スレーブモードが解除されているかで確認する
                if self.pose[37] != 1:
                    raise ValueError("Robot is not in servo mode")
                continue

            # 目標値を取得しているかを確認
            if self.pose[20] != 1:
                time.sleep(t_intv)
                # self.logger.info("Wait for target")
                # 取得する前に終了する場合即時終了可能
                if stop:
                    return True
                # ロボットにコマンドを送る前は、非常停止が押されているかを
                # スレーブモードが解除されているかで確認する
                if self.pose[37] != 1:
                    raise ValueError("Robot is not in servo mode")
                continue

            # NOTE: 最初にVR側でロボットの状態値を取得できていれば追加してもよいかも
            # state = self.pose[:6].copy()
            # target = self.pose[6:12].copy()
            # if np.any(np.abs(state - target) > 0.01):
            #     continue

            # 関節の状態値
            state = self.pose[:6].copy()

            # 目標値
            target = self.pose[6:12].copy()
            sw.lap("Check target")
            target_raw = target

            # 目標値の角度が360度の不定性が許される場合 (1度と-359度を区別しない場合) でも
            # 実機の関節の角度は360度の不定性が許されないので
            # 状態値に最も近い目標値に規格化する
            # TODO: VRと実機の関節の角度が360度の倍数だけずれた状態で、
            # 実機側で制限値を超えると動くVRと動かない実機との間に360度の倍数で
            # ないずれが生じ、急に実機が動く可能性があるので、VR側で
            # 実機との比較をし、実機側で制限値を超えることがないようにする必要がある
            # とりあえず急に動こうとすれば止まる仕組みは入れている
            target = state + (target - state + 180) % 360 - 180

            # TODO: VR側でもソフトリミットを設定したほうが良い
            target_th = np.maximum(target, -abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached minimum threshold")
            target_th = np.minimum(target_th, abs_joint_soft_limit)
            if (target != target_th).any():
                self.logger.warning("target reached maximum threshold")
            target = target_th

            # 目標値が状態値から大きく離れた場合
            # すでに目標値を受け取っていない場合はすぐに、
            # 目標値を受け取っている場合は前回の目標値からも大きく離れた場合に停止させる
            if (
                (np.abs(target - state) > 
                target_state_abs_joint_diff_limit).any()
            ) and (
                last_target is None or
                (np.abs(target - last_target) > 
                target_state_abs_joint_diff_limit).any()
            ):
                # 強制停止する。強制停止しないとエラーメッセージを返すのが複雑になる
                raise ValueError("Target and state are too different")

            last_target = target

            sw.lap("First target")
            if self.last == 0:
                self.logger.info("Start sending control command")
                # 制御する前に終了する場合即時終了可能
                if stop:
                    return True
                self.last = now

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

                # 速度制限をフィルタの手前にも入れてみる
                if True:
                    assert filter_kind == "original"
                    self.last_target_delayed_velocity = np.zeros(6)

                self.last_control_velocity = np.zeros(6)
                # ロボットにコマンドを送る前は、非常停止が押されているかを
                # スレーブモードが解除されているかで確認する
                if self.pose[37] != 1:
                    raise ValueError("Robot is not in servo mode")
                continue

            sw.lap("Check stop")
            # 制御値を送り済みの場合は
            # 目標値を状態値にしてロボットを静止させてから止める
            # 厳密にはここに初めて到達した場合は制御値は送っていないが
            # 簡潔さのため同じように扱う
            if stop:
                if target_stop is None:
                    target_stop = state
                target = target_stop

            sw.lap("Read delayed interpolator")
            # target_delayedは、delay秒前の目標値を前後の値を
            # 使って線形補間したもの
            if use_interp:
                target_delayed = di.read(now, target)
            else:
                target_delayed = target

            sw.lap("1st speed limit")
            # 速度制限をフィルタの手前にも入れてみる
            if True:
                assert filter_kind == "original"
                target_diff = target_delayed - self.last_target_delayed
                # 速度制限
                dt = now - self.last
                v = target_diff / dt
                ratio = np.abs(v) / (speed_limit_ratio * speed_limits)
                max_ratio = np.max(ratio)
                if max_ratio > 1:
                    v /= max_ratio
                target_diff_speed_limited = v * dt

                # 加速度制限
                a = (v - self.last_target_delayed_velocity) / dt
                accel_ratio = np.abs(a) / (accel_limit_ratio * accel_limits)
                accel_max_ratio = np.max(accel_ratio)
                if accel_max_ratio > 1:
                    a /= accel_max_ratio
                v = self.last_target_delayed_velocity + a * dt
                target_diff_speed_limited = v * dt

                # 速度がしきい値より小さければ静止させ無駄なドリフトを避ける
                # NOTE: スレーブモードを落とさないためには前の速度が十分小さいとき (しきい値は不明) 
                # にしか静止させてはいけない
                if np.all(target_diff_speed_limited / dt < stopped_velocity_eps):
                    target_diff_speed_limited = np.zeros_like(
                        target_diff_speed_limited)
                    v = target_diff_speed_limited / dt

                self.last_target_delayed_velocity = v
                target_delayed = self.last_target_delayed + target_diff_speed_limited

            self.last_target_delayed = target_delayed

            sw.lap("Get filtered target")
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
                # NOTE(20250813): targetがstateのフィードバックを受けていない場合は
                # target_alignedは、stateとtargetが乖離するので不適切
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

            sw.lap("2nd speed limit")
            # 速度制限
            dt = now - self.last
            v = target_diff / dt
            ratio = np.abs(v) / (speed_limit_ratio * speed_limits)
            max_ratio = np.max(ratio)
            if max_ratio > 1:
                v /= max_ratio
            target_diff_speed_limited = v * dt

            # 加速度制限
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

            sw.lap("Get control")
            # 平滑化の種類による対応
            if filter_kind == "original":
                control = self.last_control + target_diff_speed_limited
                # 登録するだけ
                _filter.filter(control)
            elif filter_kind == "target":
                control = last_target_filtered + target_diff_speed_limited
            elif filter_kind == "state_and_target_diff":
                control = last_target_filtered + target_diff_speed_limited
            elif filter_kind == "moveit_servo_humble":
                control = state + target_diff_speed_limited
            elif filter_kind == "control_and_target_diff":
                control = last_target_filtered + target_diff_speed_limited
            else:
                raise ValueError

            sw.lap("Put control to shared memory")
            self.pose[24:30] = control

            sw.lap("Save control")
            # 分析用データ保存
            datum = [
                dict(
                    time=now,
                    kind="target",
                    joint=target_raw.tolist(),
                ),
                dict(
                    time=now,
                    kind="target_delayed",
                    joint=target_delayed.tolist(),
                ),
                dict(
                    time=now,
                    kind="control",
                    joint=control.tolist(),
                    max_ratio=max_ratio,
                    accel_max_ratio=accel_max_ratio,
                ),
            ]
            self.control_to_archiver_queue.put(datum)

            sw.lap("Check elapsed before command")
            t_elapsed = time.time() - now
            if t_elapsed > t_intv * 2:
                self.logger.warning(
                    f"Control loop is 2 times as slow as expected before command: "
                    f"{t_elapsed} seconds")
                self.logger.warning(sw.summary())

            if move_robot:
                sw.lap("Send arm command")
                try:
                    self.robot.move_joint_servo(control.tolist())
                except ORiNException as e:
                    if (type(e) is ORiNException and
                        e.hresult == HResult.E_TIMEOUT):
                        raise e
                    is_error_level_0 = self.robot.is_error_level_0(e)
                    if is_error_level_0:
                        self.logger.warning(
                            "Maybe trivial error in move_joint_servo")
                        self.logger.warning(f"{self.robot.format_error(e)}")
                    else:
                        raise e

                sw.lap("Send hand command")
                if self.pose[13] == 1:
                    th1 = threading.Thread(target=self.send_grip)
                    th1.start()
                    self.pose[13] = 0

                if self.pose[13] == 2:
                    th2 = threading.Thread(target=self.send_release)
                    th2.start()
                    self.pose[13] = 0

            sw.lap("Wait control loop")
            t_elapsed = time.time() - now
            t_wait = t_intv - t_elapsed
            if t_wait > 0:
                if (move_robot and servo_mode == 0x102) or (not move_robot):
                    time.sleep(t_wait)

            sw.lap("Check elapsed after command")
            t_elapsed = time.time() - now
            if t_elapsed > t_intv * 2:
                self.logger.warning(
                    f"Control loop is 2 times as slow as expected after command: "
                    f"{t_elapsed} seconds")
                self.logger.warning(sw.summary())

            sw.stop()
            if stop:
                # スレーブモードでは十分低速時に2回同じ位置のコマンドを送ると
                # ロボットを停止させてスレーブモードを解除可能な状態になる
                if (control == self.last_control).all():
                    return True
                
            self.last_control = control
            self.last = now

    def send_grip(self) -> None:
        try:
            if self.tool_id == -1:
                return
            if self.hand_name == "onrobot_2fg7":
                # NOTE: 呼ぶ度に目標の把持力は変更できるので
                # VRコントローラーからの入力で動的に把持力を
                # 変えることもできる (どういう仕組みを作るかは別)
                self.hand.grip(waiting=False)
            elif self.hand_name == "onrobot_vgc10":
                self.hand.grip(waiting=False, vacuumA=80,  vacuumB=80)
        except Exception:
            self.logger.exception("Error gripping hand")
    
    def send_release(self) -> None:
        try:
            if self.tool_id == -1:
                return
            if self.hand_name == "onrobot_2fg7":
                # NOTE: 呼ぶ度に目標の把持力は変更できるので
                # VRコントローラーからの入力で動的に把持力を
                # 変えることもできる (どういう仕組みを作るかは別)
                self.hand.release(waiting=False)
            elif self.hand_name == "onrobot_vgc10":
                self.hand.release(waiting=False)
        except Exception:
            self.logger.exception("Error releasing hand")

    def enable(self) -> None:
        try:
            self.robot.enable_robot(ext_speed=speed_normal)
        except Exception as e:
            self.logger.error("Error enabling robot")
            self.logger.error(f"{self.robot.format_error(e)}")

    def disable(self) -> None:
        try:
            self.robot.disable()
        except Exception as e:
            self.logger.error("Error disabling robot")
            self.logger.error(f"{self.robot.format_error(e)}")

    def set_area_enabled(self, enable: bool) -> None:
        try:
            self.robot.SetAreaEnabled(0, enable=enable)
            self.pose[31] = int(enable)
        except Exception as e:
            self.logger.error("Error setting area enabled")
            self.logger.error(f"{self.robot.format_error(e)}")

    def tidy_pose(self) -> None:
        try:
            self.robot.move_joint_until_completion(self.tidy_joint)
        except Exception as e:
            self.logger.error("Error moving to tidy pose")
            self.logger.error(f"{self.robot.format_error(e)}")

    def clear_error(self) -> None:
        try:
            self.robot.clear_error()
        except Exception as e:
            self.logger.error("Error clearing robot error")
            self.logger.error(f"{self.robot.format_error(e)}")

    def enter_servo_mode(self):
        # self.pose[14]は0のとき必ず通常モード。
        # self.pose[14]は1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        # 順番固定
        with self.slave_mode_lock:
            self.pose[14] = 1
        self.robot.enter_servo_mode()

    def leave_servo_mode(self):
        # self.pose[14]は0のとき必ず通常モード。
        # self.pose[14]は1のとき基本的にスレーブモードだが、
        # 変化前後の短い時間は通常モードの可能性がある。
        # 順番固定
        self.robot.leave_servo_mode()
        while True:
            if not self.robot.is_in_servo_mode():
                break
            time.sleep(0.008)
        self.pose[14] = 0

    def control_loop_w_recover_automatic(self) -> bool:
        """自動復帰を含むリアルタイム制御ループ"""
        self.logger.info("Start Control Loop with Automatic Recover")
        # 自動復帰ループ
        while True:
            try:
                # 制御ループ
                # 停止するのは、ユーザーが要求した場合か、自然に内部エラーが発生した場合
                self.enter_servo_mode()
                self.control_loop()
                self.leave_servo_mode()
                # ここまで正常に終了した場合、ユーザーが要求した場合が成功を意味する
                if self.pose[16] == 1:
                    self.pose[16] = 0
                    self.logger.info("User required stop and succeeded")
                    return True
            except Exception as e:
                # 自然に内部エラーが発生した場合、自動復帰を試みる
                # 自動復帰の前にエラーを確実にモニタするため待機
                time.sleep(1)
                self.logger.error("Error in control loop")
                self.logger.error(f"{self.robot.format_error(e)}")

                # 目標値が状態値から大きく離れた場合は自動復帰しない
                if str(e) == "Target and state are too different":
                    self.pose[16] = 0
                    return False

                # 必ずスレーブモードから抜ける
                try:
                    self.leave_servo_mode()
                except Exception as e_leave:
                    self.logger.error("Error leaving servo mode")
                    self.logger.error(f"{self.robot.format_error(e_leave)}")
                    # タイムアウトの場合はスレーブモードは切れているので
                    # 共有メモリを更新する
                    if ((type(e_leave) is ORiNException and
                        e_leave.hresult == HResult.E_TIMEOUT) or
                        (type(e_leave) is not ORiNException)):
                        self.pose[14] = 0
                    # それ以外は原因不明なのでループは抜ける
                    else:
                        self.pose[16] = 0
                        return False

                # 非常停止ボタンの状態値を最新にするまで待つ必要がある
                time.sleep(1)
                # 非常停止ボタンが押された場合は自動復帰しない
                if self.pose[36] == 1:
                    self.logger.error("Emergency stop is pressed")
                    self.pose[16] = 0
                    return False

                # タイムアウトの場合は接続からやり直す
                if ((type(e) is ORiNException and
                    e.hresult == HResult.E_TIMEOUT) or
                   (type(e) is not ORiNException)):
                        for i in range(1, 11):
                            try:
                                self.robot.start()
                                self.robot.clear_error()

                                # NOTE: タイムアウトした場合の数回に1回、
                                # 制御権が取得できない場合がある。しかし、
                                # このメソッドのこの例外から抜けた後に
                                # GUIでClearError -> Enable -> StartMQTTControl
                                # とすると制御権が取得できる。
                                # ここで制御権を取得しても、GUIから制御権を取得しても
                                # 内部的には同じ関数を呼んでいるので原因不明
                                # (ソケットやbCAPClientのidが両者で同じことも確認済み)
                                # 0. 元
                                self.robot.take_arm()
                                # 1. ここをイネーブルにしても変わらない
                                # self.robot.enable_robot(ext_speed=speed_normal)
                                # 2. manual_resetを追加しても変わらない
                                # self.robot.manual_reset()
                                # self.robot.take_arm()
                                # 3. 待っても変わらない
                                # time.sleep(5)
                                # self.robot.take_arm()
                                # time.sleep(5)

                                self.find_and_setup_hand(self.tool_id)
                                self.logger.info(
                                    "Reconnected to robot successfully"
                                    " after timeout")
                                break
                            except Exception as e_reconnect:
                                self.logger.error(
                                    "Error in reconnecting robot")
                                self.logger.error(
                                    f"{self.robot.format_error(e_reconnect)}")
                                if i == 10:
                                    self.logger.error(
                                        "Failed to reconnect robot after"
                                        " 10 attempts")
                                    self.pose[16] = 0
                                    return False
                            time.sleep(1)
                # ここまでに接続ができている場合
                try:
                    errors = self.robot.get_cur_error_info_all()
                    self.logger.error(f"Errors in teach pendant: {errors}")
                    # 自動復帰可能エラー
                    if self.robot.are_all_errors_stateless(errors):
                        # 自動復帰を試行。失敗またはエラーの場合は通常モードに戻る。
                        # エラー直後の自動復帰処理に失敗しても、
                        # 同じ復帰処理を手動で行うと成功することもあるので
                        # 手動で操作が可能な状態に戻す
                        ret = self.robot.recover_automatic_enable()
                        if not ret:
                            raise ValueError(
                                "Automatic recover failed in enable timeout")
                        self.logger.info("Automatic recover succeeded")                    
                    # 自動復帰不可能エラー
                    else:
                        self.logger.error(
                            "Error is not automatically recoverable")
                        self.pose[16] = 0
                        return False
                except Exception as e_recover:
                    self.logger.error("Error during automatic recover")
                    self.logger.error(f"{self.robot.format_error(e_recover)}")
                    self.pose[16] = 0
                    return False


    def mqtt_control_loop(self) -> None:
        """MQTTによる制御ループ"""
        self.logger.info("Start MQTT Control Loop")
        self.pose[15] = 1
        while True:
            # 停止するのは、ユーザーが要求した場合か、自然に内部エラーが発生した場合
            success_stop = self.control_loop_w_recover_automatic()
            # 停止フラグが成功の場合は、ユーザーが要求した場合のみありうる
            next_tool_id = self.pose[17].copy()
            put_down_box = self.pose[21].copy()
            change_log_file = self.pose[33].copy()
            if success_stop:
                # ツールチェンジが要求された場合
                if next_tool_id != 0:
                    self.logger.info(
                        f"User required tool change to: {next_tool_id}")
                    # ツールチェンジに成功した場合は、ループを継続し
                    # 失敗した場合は、ループを抜ける
                    try:
                        self.tool_change(next_tool_id)
                        # NOTE: より良い方法がないか
                        # VRアニメーションがロボットの動きに追従し終わるのを待つ
                        time.sleep(3)
                        self.pose[18] = 1
                        self.pose[17] = 0
                        # VRのIKで解いた関節角度にロボットの関節角度を合わせるのを待つ
                        time.sleep(3)
                        self.logger.info("Tool change succeeded")
                    except Exception as e:
                        self.logger.error("Error during tool change")
                        self.logger.error(f"{self.robot.format_error(e)}")
                        self.pose[18] = 2
                        self.pose[17] = 0
                        break
                # 棚の上の箱を置くことが要求された場合
                elif put_down_box != 0:
                    self.logger.info("User required put down box")
                    # 成功しても失敗してもループを継続する (ツールを変えることによる
                    # 予測できないエラーは起こらないため)
                    self.demo_put_down_box()
                # ログファイルを変更することが要求された場合
                elif change_log_file != 0:
                    self.logger.info("User required change log file")
                    # ログファイルを変更する
                    self.get_logging_dir_and_change_log_file()
                # 単なる停止が要求された場合は、ループを抜ける
                else:
                    break
            # 停止フラグが失敗の場合は、ユーザーが要求した場合か、
            # 自然に内部エラーが発生した場合
            else:
                # ツールチェンジが要求された場合
                if next_tool_id != 0:
                    # 要求コマンドのみリセット
                    self.pose[18] = 2
                    self.pose[17] = 0
                # 棚の上の箱を置くことが要求された場合
                elif put_down_box != 0:
                    # 要求コマンドのみリセット
                    self.pose[22] = 2
                    self.pose[21] = 0
                # ループを抜ける
                elif change_log_file != 0:
                    # 要求コマンドのみリセット
                    self.pose[33] = 0
                break
        self.pose[15] = 0

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def tool_change(self, next_tool_id: int) -> None:
        if next_tool_id == self.tool_id:
            self.logger.info("Selected tool is current tool.")
            return
        tool_info = self.get_tool_info(tool_infos, self.tool_id)
        next_tool_info = self.get_tool_info(tool_infos, next_tool_id)
        # ツールチェンジはワークから十分離れた場所で行うことを仮定
        current_joint = self.robot.get_current_joint()
        self.robot.move_joint(self.tidy_joint)
        # ツールチェンジの場所が移動可能エリア外なので、エリア機能を無効にする
        self.robot.SetAreaEnabled(0, False)
        self.pose[31] = 0
        # アームの先端の位置で制御する（現在のツールに依存しない）
        self.robot.set_tool(0)

        # 現在のツールとの接続を切る
        # 現在ツールが付いていないとき
        if tool_info["id"] == -1:
            assert next_tool_info["id"] != -1
        # 現在ツールが付いているとき
        else:
            self.hand.disconnect()

        # 現在ツールが付いていないとき
        if tool_info["id"] == -1:
            assert next_tool_info["id"] != -1
            wps = next_tool_info["holder_waypoints"]
            self.robot.move_pose(wps["enter_path"])
            self.robot.move_pose(wps["disengaged"])
            self.robot.ext_speed(speed_tool_change)
            self.robot.move_pose(wps["tool_holder"])
            time.sleep(1)
            self.robot.move_pose(wps["locked"])
            name = next_tool_info["name"]
            hand = tool_classes[name]()
            connected = hand.connect_and_setup()
            # NOTE: 接続できなければ止めたほうが良いと考える
            if not connected:
                raise ValueError(f"Failed to connect to hand: {name}")
            self.hand_name = name
            self.hand = hand
            self.tool_id = next_tool_id
            self.pose[23] = next_tool_id
            self.robot.ext_speed(speed_normal)
            self.robot.move_pose(wps["exit_path_1"])
            self.robot.move_pose(wps["exit_path_2"])
        # 現在ツールが付いているとき
        else:
            wps = tool_info["holder_waypoints"]
            self.robot.move_pose(wps["exit_path_2"])
            self.robot.move_pose(wps["exit_path_1"])
            self.robot.move_pose(wps["locked"])
            self.robot.ext_speed(speed_tool_change)
            self.robot.move_pose(wps["tool_holder"])
            time.sleep(1)
            self.robot.move_pose(wps["disengaged"])
            if next_tool_info["id"] == -1:
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["enter_path"])
            elif tool_info["holder_region"] == next_tool_info["holder_region"]:
                wps = next_tool_info["holder_waypoints"]
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["disengaged"])
                self.robot.ext_speed(speed_tool_change)
                self.robot.move_pose(wps["tool_holder"])
                time.sleep(1)
                self.robot.move_pose(wps["locked"])
                name = next_tool_info["name"]
                hand = tool_classes[name]()
                connected = hand.connect_and_setup()
                # NOTE: 接続できなければ止めたほうが良いと考える
                if not connected:
                    raise ValueError(f"Failed to connect to hand: {name}")
                self.hand_name = name
                self.hand = hand
                self.tool_id = next_tool_id
                self.pose[23] = next_tool_id
                self.robot.ext_speed(speed_normal)
            elif tool_info["holder_region"] != next_tool_info["holder_region"]:
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["enter_path"])
                self.robot.move_pose(tool_base, fig=-3)
                wps = next_tool_info["holder_waypoints"]
                self.robot.ext_speed(speed_normal)
                self.robot.move_pose(wps["disengaged"])
                self.robot.ext_speed(speed_tool_change)
                self.robot.move_pose(wps["tool_holder"])
                time.sleep(1)
                self.robot.move_pose(wps["locked"])
                name = next_tool_info["name"]
                hand = tool_classes[name]()
                connected = hand.connect_and_setup()
                # NOTE: 接続できなければ止めたほうが良いと考える
                if not connected:
                    raise ValueError(f"Failed to connect to hand: {name}")
                self.hand_name = name
                self.hand = hand
                self.tool_id = next_tool_id
                self.pose[23] = next_tool_id
                self.robot.ext_speed(speed_normal)
            self.robot.move_pose(wps["exit_path_1"])
            self.robot.move_pose(wps["exit_path_2"])
                
        # 以下の移動後、ツールチェンジ前後でのTCP位置は変わらない
        # （ツールの大きさに応じてアームの先端の位置が変わる）
        if next_tool_info["id"] != -1:
            self.robot.SetToolDef(
                next_tool_info["id_in_robot"], next_tool_info["tool_def"])
        self.robot.set_tool(next_tool_info["id_in_robot"])
        self.robot.move_joint(self.tidy_joint)
        # エリア機能を有効にする
        self.robot.SetAreaEnabled(0, True)
        self.pose[31] = 1
        if next_tool_info["id"] == 4:
            # tool_baseから箱の手前まで直接行くと台にぶつかりそうなので少し手前に移動
            # pose = [-302.96, -400.32, 831.60, -46.90, 88.37, -136.46]
            # jointで移動したほうが特異点を経由しないので止まりにくい
            self.robot.move_joint(
                [-123.41, -2.78, 60.32, -127.61, -44.68, 136.85]
            )
            # 箱の手前に移動
            # pose = [-302.92, -560.30, 830.96, -49.35, 88.43, -138.85]
            self.robot.move_joint(
                [-110.01, 11.00, 48.76, -142.45, -35.12, 147.03]
            )
        else:
            # ツールチェンジ後に実機をVRに合わせる場合
            # ツールチェンジ前の位置だけでなく関節角度も合わせる必要がある
            # ツールチェンジ後にVRを実機に合わせる場合は必ずしも
            # その限りではないが、関節空間での補間に悪影響があるかもしれないので
            # 関節角度を前後で合わせることを推奨
            self.robot.move_joint(current_joint)
        return

    def tool_change_not_in_rt(self) -> None:
        while True:
            next_tool_id = self.pose[17]
            if next_tool_id != 0:
                try:
                    self.tool_change(next_tool_id)
                    self.pose[18] = 1
                except Exception as e:
                    self.logger.error("Error during tool change")
                    self.logger.error(f"{self.robot.format_error(e)}")
                    self.pose[18] = 2
                finally:
                    self.pose[17] = 0
                    break

    def jog_joint(self, joint: int, direction: float) -> None:
        try:
            self.robot.jog_joint(joint, direction)
        except Exception as e:
            self.logger.error("Error during joint jog")
            self.logger.error(f"{self.robot.format_error(e)}")

    def jog_tcp(self, axis: int, direction: float) -> None:
        try:
            self.robot.jog_tcp(axis, direction)
        except Exception as e:
            self.logger.error("Error during TCP jog")
            self.logger.error(f"{self.robot.format_error(e)}")

    def demo_put_down_box(self) -> None:
        """
        デモ用に棚の上の箱を作業台に下ろす動き
        ロボットと棚の上の箱の位置関係上、ロボットの特異姿勢（ひじ、手首特異姿勢）
        が集まっており、それらをかいくぐってなんとか下ろすようにしている
        したがって棚の上の箱の位置はほぼ同じ位置にあることを前提とする
        この関数を呼ぶ前に、ロボットの先端のホルダーを棚の上の箱に引っ掛けておく
        """
        try:
            if self.tool_id != 4:
                raise ValueError("Tool is not the box holder")

            use_pre_automatic_move = True
            if use_pre_automatic_move:
                # 現状はホルダーへのツールチェンジ後の箱の少し手前の位置から、
                # 箱の位置へと自動で移動するようにしている
                # 本当は手動で移動させたほうが想定に近いが、
                # 手動またはTCP制御で移動しようとすると、関節2より関節3が先に動き、
                # ひじ特異姿勢に近くなるため、このようにしている

                # ホルダーへのツールチェンジ後の箱の少し手前の位置
                # ツールチェンジで移動済みだがもう一度同じ場所にいることを保証させる
                # pose = [-302.92, -560.30, 830.96, -49.35, 88.43, -138.85]
                self.robot.move_joint(
                    [-110.01, 11.00, 48.76, -142.45, -35.12, 147.03]
                )

                # ここから箱の位置へと自動で移動する
                # TCP制御 (これではひじ特異姿勢に近くなる)
                # self.robot.move_pose(
                #     [-302.92, -660.89, 830.96, -49.35, 88.43, -138.84],
                #     interpolation=2, fig=-2
                # )
                # かわりに関節制御する (衝突しないことを確認済み)
                self.robot.ext_speed(5)
                self.robot.move_joint(
                    [-105.27, 22.47, 35.35, -151.32, -34.53, 154.84]
                )
                self.robot.ext_speed(speed_normal)

            # この関数を呼ぶ前にホルダーを箱に引っ掛けておく
            # 棚の上の箱はおおよそこの位置にあることを前提とする
            target = [-302.92, -660.89, 830.96, -49.35, 88.43, -138.84]
            ranges = [
                ("X", 0, target[0] - 50, target[0] + 50),
                ("Y", 1, target[1] - 50, target[1] + 50),
                ("Z", 2, target[2] - 20, target[2] + 80),
            ]
            state = self.robot.get_current_pose()
            violations = []
            for label, idx, low, high in ranges:
                if not (low < state[idx] < high):
                    violations.append(
                        f"{label}: {state[idx]:.2f} "
                        f"(required: {low:.2f} < {label} < {high:.2f})")
            if violations:
                msg = "Position out of range:\n" + "\n".join(violations)
                raise ValueError(msg)
            # 箱を持ち上げる
            up_state = state.copy()
            up_state[2] += 50
            # 形態1
            # 直線移動、形態一定で移動する
            self.robot.ext_speed(5)
            self.robot.move_pose(up_state, interpolation=2, fig=-2)
            self.robot.ext_speed(speed_normal)
            # 箱を棚から出す
            self.robot.move_pose(
                [-302.97, -140.75, 885.68, -49.70, 88.44, -139.19],
                interpolation=2, fig=-2
            )
            # 形態が変わる場所はPTPで移動する
            # 形態5
            self.robot.move_pose(
                [-302.97, -135.03, 885.67, -49.70, 88.44, -139.19],
                interpolation=1, fig=-3
            )
            # 形態69
            self.robot.move_pose(
                [-302.87, -131.23, 885.66, -49.75, 88.45, -139.24],
                interpolation=1, fig=-3
            )
            self.robot.move_pose(
                [-302.97, -22.83, 885.52, -49.60, 88.44, -139.09],
                interpolation=2, fig=-2
            )
            # 箱を作業台の真上に移動させる
            self.robot.move_pose(
                [-457.80, -22.82, 885.51, -49.58, 88.45, -139.08],
                interpolation=2, fig=-2
            )
            # 作業台の上に箱を下ろす
            self.robot.move_pose(
                [-457.80, -22.67, 543.43, -49.78, 88.45, -139.27],
                interpolation=2, fig=-2
            )
            # 形態65
            self.robot.move_pose(
                [-457.80, -22.66, 531.33, -49.83, 88.46, -139.31],
                interpolation=1, fig=-3
            )
            # 作業台にはゆっくりと着地させる
            self.robot.move_pose(
                [-457.80, -22.82, 89.16, -49.58, 88.45, -139.07],
                interpolation=2, fig=-2
            )
            self.robot.ext_speed(5)
            self.robot.move_pose(
                [-457.80, -22.82, 39.16, -49.58, 88.45, -139.07],
                interpolation=2, fig=-2
            )
            self.robot.ext_speed(speed_normal)
            # 箱からホルダーを抜く
            self.robot.move_pose(
                [-457.80, 15.20, 39.16, -49.58, 88.45, -139.07],
                interpolation=2, fig=-2
            )
            # ホルダーを上に引き上げる
            self.robot.move_pose(
                [-457.80, 15.20, 459.68, -49.58, 88.45, -139.05],
                interpolation=2, fig=-2
            )
            # ツール先端を下方向に向ける
            self.robot.move_pose(
                [-457.81, 15.20, 459.67, -178.82, 0.04, 90.50],
                interpolation=1, fig=-3
            )
            # ほぼ同じツール姿勢だが、VRのIKで解いた場合の関節角度に
            # 合わせる (箱下ろし完了後のVR手動操作で合わせるとユーザーが驚くため)
            self.robot.move_joint(
                [-195.75, -7.50, 96.01, -0.89, 74.52, 248.67]
            )
            # NOTE: より良い方法がないか
            # VRアニメーションがロボットの動きに追従し終わるのを待つ
            time.sleep(3)
            self.pose[22] = 1
            # VRのIKで解いた関節角度にロボットの関節角度を合わせるのを待つ
            time.sleep(3)
        except Exception as e:
            self.logger.error("Error during demo put down box")
            self.logger.error(f"{self.robot.format_error(e)}")
            self.pose[22] = 2
        finally:
            self.pose[21] = 0

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("CTRL")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)
        self.robot_logger = logging.getLogger("CTRL-ROBOT")
        if log_queue is not None:
            self.robot_handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.robot_handler = logging.StreamHandler()
        self.robot_logger.addHandler(self.robot_handler)
        self.robot_logger.setLevel(logging.WARNING)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.control_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.change_log_file(logging_dir)

    def change_log_file(self, logging_dir: str) -> None:
        self.logging_dir = logging_dir
        self.pose[33] = 0

    def run_proc(self, control_pipe, slave_mode_lock, log_queue, logging_dir, control_to_archiver_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.slave_mode_lock = slave_mode_lock
        self.control_pipe = control_pipe
        self.logging_dir = logging_dir
        self.control_to_archiver_queue = control_to_archiver_queue

        self.init_robot()
        self.init_realtime()
        while True:
            if control_pipe.poll(timeout=1):
                command = control_pipe.recv()
                if command["command"] == "enable":
                    self.enable()
                elif command["command"] == "disable":
                    self.disable()
                elif command["command"] == "set_area_enabled":
                    self.set_area_enabled(**command["params"])
                elif command["command"] == "tidy_pose":
                    self.tidy_pose()
                elif command["command"] == "release_hand":
                    self.logger.info("Release hand")
                    self.send_release()
                elif command["command"] == "clear_error":
                    self.clear_error()
                elif command["command"] == "start_mqtt_control":
                    self.mqtt_control_loop()
                elif command["command"] == "tool_change":
                    self.tool_change_not_in_rt()
                elif command["command"] == "jog_joint":
                    self.jog_joint(**command["params"])
                elif command["command"] == "jog_tcp":
                    self.jog_tcp(**command["params"])
                elif command["command"] == "demo_put_down_box":
                    self.demo_put_down_box()
                elif command["command"] == "change_log_file":
                    # MQTTControl時以外にログファイルを変更する場合に対応
                    self.logger.info("Change log file")
                    self.change_log_file(**command["params"])
                else:
                    self.logger.warning(
                        f"Unknown command: {command['command']}")
            if self.pose[32] == 1:
                self.sm.close()
                self.control_to_archiver_queue.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                self.robot_handler.close()
                break


class Cobotta_Pro_CON_Archiver:
    def monitor_start(self, f: TextIO | None = None):
        while True:
            # ログファイル変更時
            if self.pose[35] == 1:
                return True
            try:
                datum = self.control_to_archiver_queue.get(
                    block=True, timeout=T_INTV)
            except queue.Empty:
                datum = None
            if ((f is not None) and 
                (datum is not None)):
                s = ""
                for d in datum:
                    s = s + json.dumps(d, ensure_ascii=False) + "\n"
                f.write(s)
            # プロセス終了時
            if self.pose[32] == 1:
                return False

    def setup_logger(self, log_queue):
        self.logger = logging.getLogger("CTRL-ARCV")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def get_logging_dir_and_change_log_file(self) -> None:
        command = self.control_arcv_pipe.recv()
        logging_dir = command["params"]["logging_dir"]
        self.logger.info("Change log file")
        self.change_log_file(logging_dir)

    def change_log_file(self, logging_dir: str) -> None:
        self.logging_dir = logging_dir
        self.pose[35] = 0

    def run_proc(self, control_arcv_pipe, log_queue, logging_dir, control_to_archiver_queue):
        self.setup_logger(log_queue)
        self.logger.info("Process started")
        self.sm = mp.shared_memory.SharedMemory(SHM_NAME)
        self.pose = np.ndarray((SHM_SIZE,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.control_arcv_pipe = control_arcv_pipe
        self.logging_dir = logging_dir
        self.control_to_archiver_queue = control_to_archiver_queue

        while True:
            try:
                # 基本はmonitor_start内のループにいるが、
                # ログファイル変更またはプロセス終了時に
                # monitor_startから抜ける
                if save_control:
                    with open(
                        os.path.join(self.logging_dir, "control.jsonl"), "a"
                    ) as f:
                        will_change_log_file = self.monitor_start(f)
                else:
                    will_change_log_file = self.monitor_start()
                # ログファイル変更時は大きいループを継続
                if will_change_log_file:
                    self.get_logging_dir_and_change_log_file()
            except Exception as e:
                self.logger.error("Error in control archiver")
                self.logger.error(e)
            # プロセス終了時は大きいループを抜ける
            if self.pose[32] == 1:
                self.sm.close()
                self.control_to_archiver_queue.close()
                time.sleep(1)
                self.logger.info("Process stopped")
                self.handler.close()
                break
