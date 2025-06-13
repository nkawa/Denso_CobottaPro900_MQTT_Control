# Cobotta Proを制御する

from typing import Any, Dict, List, Literal, Tuple
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
    t_intv = 0.008
n_windows *= int(0.008 / t_intv)
reset_default_state = True
default_joints = {
    # TCPが台の中心の上に来る初期位置
    "tidy": [4.7031, -0.6618, 105.5149, 0.0001, 75.1440, -85.2900],
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
abs_joint_limit = [270, 150, 150, 270, 150, 360]
abs_joint_limit = np.array(abs_joint_limit)
abs_joint_soft_limit = abs_joint_limit - 10
# 外部速度。単位は%
speed_normal = 10
speed_tool_change = 2
# 目標値が状態値よりこの制限より大きく乖離した場合はロボットを停止させる
# 設定値は典型的なVRコントローラの動きから決定した
target_state_abs_joint_diff_limit = [30, 30, 40, 40, 40, 60]

save_control = SAVE
if save_control:
    save_path = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + "_control.jsonl"
    f = open(save_path, "w")

class Cobotta_Pro_CON:
    def __init__(self):
        self.default_joint = default_joints["vr5"]
        self.tidy_joint = default_joints["tidy"]

    def init_robot(self):
        self.robot = DensoRobot(
            host=ROBOT_IP,
            default_servo_mode=servo_mode,
        )
        self.robot.start()
        self.robot.clear_error()
        self.robot.take_arm()
        self.find_and_setup_hand()

    def find_and_setup_hand(self):
        tool_id = int(os.environ["TOOL_ID"])
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
                print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
            else:
                print("Process real-time priority set to: %u" % rt_app_priority)

    def control_loop(self) -> Tuple[bool, str]:
        self.last = 0
        print("[CNT]Start Main Loop")
        self.pose[19] = 0
        self.pose[20] = 0
        target_stop = None
        code_stop = 0
        message_stop = "Interrupted"
        while True:
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
                # print("[CNT]Wait for monitoring..")
                # 取得する前に終了する場合即時終了可能
                if stop:
                    return True, message_stop
                continue

            # 目標値を取得しているかを確認
            if self.pose[20] != 1:
                time.sleep(t_intv)
                # print("[CNT]Wait for target..")
                # 取得する前に終了する場合即時終了可能
                if stop:
                    return True, message_stop
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
            if (target == target_th).any():
                pass
                # print("[CNT]: Warning: target reached minimum threshold")
            target_th = np.minimum(target_th, abs_joint_soft_limit)
            if (target == target_th).any():
                pass
                # print("[CNT]: Warning: target reached maximum threshold")
            target = target_th

            # 目標値が状態値から大きく離れた場合は制御を停止する
#            if (np.abs(target - state) > 
#                target_state_abs_joint_diff_limit).any():
#                stop = 1
#                code_stop = 1
#                message_stop = "目標値が状態値から離れすぎています"

            now = time.time()
            if self.last == 0:
                print("[CNT]Starting to Control!",self.pose)
                # 制御する前に終了する場合即時終了可能
                if stop:
                    return True, message_stop
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
                continue

            # 制御値を送り済みの場合は
            # 目標値を状態値にしてロボットを静止させてから止める
            # 厳密にはここに初めて到達した場合は制御値は送っていないが
            # 簡潔さのため同じように扱う
            if stop:
                if target_stop is None:
                    target_stop = state
                target = target_stop

            # target_delayedは、delay秒前の目標値を前後の値を
            # 使って線形補間したもの
            if use_interp:
                target_delayed = di.read(now, target)
            else:
                target_delayed = target

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
                        return 2, "Error occurred"

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
                t_elapsed = time.time() - now
                t_wait = t_intv - t_elapsed
                if t_wait > 0:
                    time.sleep(t_wait)

            if stop:
                # スレーブモードでは十分低速時に2回同じ位置のコマンドを送ると
                # ロボットを停止させてスレーブモードを解除可能な状態になる
                if (control == self.last_control).all():
                    return code_stop, message_stop
                
            self.last_control = control
            self.last = now

    def send_grip(self):
        if self.tool_id == -1:
            return
        if self.hand_name == "onrobot_2fg7":
            # NOTE: 呼ぶ度に目標の把持力は変更できるので
            # VRコントローラーからの入力で動的に把持力を
            # 変えることもできる (どういう仕組みを作るかは別)
            self.hand.grip(waiting=False)
        elif self.hand_name == "onrobot_vgc10":
            self.hand.grip(waiting=False, vacuumA = 40,  vacuumB =40)

    def send_release(self):
        if self.tool_id == -1:
            return
        if self.hand_name == "onrobot_2fg7":
            # NOTE: 呼ぶ度に目標の把持力は変更できるので
            # VRコントローラーからの入力で動的に把持力を
            # 変えることもできる (どういう仕組みを作るかは別)
            self.hand.release(waiting=False)
        elif self.hand_name == "onrobot_vgc10":
            self.hand.release(waiting=False)

    def enable(self) -> bool:
        self.robot.enable_robot()
        # self.robot.SetAreaEnabled(0, False)

    def disable(self) -> bool:
        self.robot.disable()

    def default_pose(self) -> bool:
        self.robot.move_joint_until_completion(self.default_joint)

    def tidy_pose(self):
        self.robot.move_joint_until_completion(self.tidy_joint)

    def clear_error(self) -> bool:
        self.robot.clear_error()

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

    def control_loop_w_recover_automatic(self):
        try:
            self.pose[15] = 1
            self.enter_servo_mode()
        # モーターがOFFなどの理由でスレーブモードに入れない場合
        # self.pose[15]を0に戻す
        except ORiNException as e:
            print("[CNT]: Error entering servo mode")
            print(f"[CNT]: {self.robot.format_error(e)}")
            self.leave_servo_mode()
            self.pose[15] = 0
            self.pose[16] = 0
            return

        while True:
            code_stop, message_stop = self.control_loop()
            self.leave_servo_mode()
            if code_stop == 0:
                print("[CNT]: User required stop and successfully done")
                break
            else:
                if code_stop == 1:
                    print(f"[CNT]: {message_stop}")
                    # FIXME: 現状は現在地と目標値の乖離による場合は、
                    # GUIのランプがOFFになる以外に、実験者が耳でも気づくように、
                    # モーターの電源を切ることにする
                    self.disable()
                    break
                # 自動復帰の前にエラーを確実にモニタするため待機
                time.sleep(1)
                errors = self.robot.get_cur_error_info_all()
                print(f"[CNT]: Error in control loop: {errors}")
                # ユーザーが停止させようとしてきちんと停止しなかった場合
                # エラーによらず通常モードに戻る。
                if self.pose[16] == 1:
                    print("[CNT]: User required stop and not successfully done")
                    break
                # 自動復帰可能エラー
                elif self.robot.are_all_errors_stateless(errors):
                    # 自動復帰を試行。失敗またはエラーの場合は通常モードに戻る。
                    try:
                        # エラー直後の自動復帰処理に失敗しても、
                        # 同じ復帰処理を手動で行うと成功することもあるので
                        # 手動で操作が可能な状態に戻す
                        ret = self.robot.recover_automatic_enable()
                        if not ret:
                            print("[CNT]: Cannot automatically recover enable")
                            break
                        self.enter_servo_mode()
                        continue
                    # NOTE: 検証用。エラー処理が確立すれば不要と思われる
                    except ORiNException as e:
                        print("[CNT]: Error during automatic recover")
                        print(f"[CNT]: {self.robot.format_error(e)}")
                        self.leave_servo_mode()
                        break
                # 自動復帰不可能エラー
                else:
                    print("[CNT]: Error is not automatically recoverable")
                    break
        self.pose[15] = 0
        self.pose[16] = 0

    def control_loop_w_tool_change(self):
        # リアルタイム制御中にツールチェンジする
        while True:
            self.control_loop_w_recover_automatic()
            next_tool_id = self.pose[17]
            if next_tool_id != 0:
                try:
                    self.pose[18] = 0
                    self.tool_change(next_tool_id)
                except ORiNException as e:
                    print("[CNT]: Error during tool change")
                    print(f"[CNT]: {self.robot.format_error(e)}")
                finally:
                    self.pose[18] = 0
                    self.pose[17] = 0
            else:
                break

    def get_tool_info(
        self, tool_infos: List[Dict[str, Any]], tool_id: int) -> Dict[str, Any]:
        return [tool_info for tool_info in tool_infos
                if tool_info["id"] == tool_id][0]

    def tool_change(self, next_tool_id: int) -> None:
        self.pose[18] = 1
        while True:
            if self.pose[18] == 2:
                break
            time.sleep(0.008)
        if next_tool_id == self.tool_id:
            print("Selected tool is current tool.")
            self.pose[18] = 0
            return
        tool_info = self.get_tool_info(tool_infos, self.tool_id)
        next_tool_info = self.get_tool_info(tool_infos, next_tool_id)
        # ツールチェンジで行うと予想される仮動作として実装
        # ワークから離れるため、真上のTCP位置を記録しそこへ移動する
        current_pose = self.robot.get_current_pose()
        # diff = [0, 0, 100, 0, 0, 0]
        diff = [0, 0, 0, 0, 0, 0]
        up_pose = (np.asarray(current_pose) + np.asarray(diff)).tolist()
        self.robot.move_pose(up_pose, fig=-3)
        self.robot.move_pose(tool_base, fig=-3)
        # ツールチェンジの場所が移動可能エリア外なので、エリア機能を無効にする
        self.robot.SetAreaEnabled(0, False)
        # アームの先端の位置で制御する（現在のツールに依存しない）
        self.robot.set_tool(0)

        # 現在のツールとの接続を切る
        # 現在ツールが付いていないとき
        if tool_info["id"] == -1:
            assert next_tool_info["id"] != -1
        # 現在ツールが付いているとき
        else:
            self.hand.disconnect()
        self.pose[18] = 3
        while True:
            if self.pose[18] == 4:
                break
            time.sleep(0.008)

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
            self.pose[18] = 5
            while True:
                if self.pose[18] == 6:
                    break
                time.sleep(0.008)
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
                self.pose[18] = 5
                while True:
                    if self.pose[18] == 6:
                        break
                    time.sleep(0.008)
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
                self.pose[18] = 5
                while True:
                    if self.pose[18] == 6:
                        break
                    time.sleep(0.008)
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
                self.pose[18] = 5
                while True:
                    if self.pose[18] == 6:
                        break
                    time.sleep(0.008)
                self.robot.ext_speed(speed_normal)
            self.robot.move_pose(wps["exit_path_1"])
            self.robot.move_pose(wps["exit_path_2"])
                
        # 以下の移動後、ツールチェンジ前後でのTCP位置は変わらない
        # （ツールの大きさに応じてアームの先端の位置が変わる）
        if next_tool_info["id"] != -1:
            self.robot.SetToolDef(
                next_tool_info["id_in_robot"], next_tool_info["tool_def"])
        self.robot.set_tool(next_tool_info["id_in_robot"])
        self.robot.move_pose(tool_base, fig=-3)
        if next_tool_info["id"] == 4:
            # tool_baseから箱の手前まで直接行くと台にぶつかりそうなので少し手前に移動
            self.robot.move_pose(
                [-229.66, -413.48, 703.63, -1.44, 88.95, -90.58], fig=-3)
            # 箱の手前に移動
            self.robot.move_pose(
                [-229.66, -613.48, 703.63, -1.44, 88.95, -90.58], fig=-3)
        else:
            self.robot.move_pose(up_pose, fig=-3)
        # エリア機能を有効にする
        self.robot.SetAreaEnabled(0, True)
        self.hand_name = name
        self.hand = hand
        self.tool_id = next_tool_id
        self.pose[18] = 0
        return

    def run_proc(self, control_pipe, slave_mode_lock):
        self.sm = mp.shared_memory.SharedMemory("cobotta_pro")
        self.pose = np.ndarray((32,), dtype=np.dtype("float32"), buffer=self.sm.buf)
        self.slave_mode_lock = slave_mode_lock
 
        self.init_robot()
        self.init_realtime()
        while True:
            command = control_pipe.recv()
            try:
                if command["command"] == "enable":
                    self.enable()
                elif command["command"] == "disable":
                    self.disable()
                elif command["command"] == "default_pose":
                    self.default_pose()
                elif command["command"] == "tidy_pose":
                    self.tidy_pose()
                elif command["command"] == "release_hand":
                    self.send_release()
                elif command["command"] == "clear_error":
                    self.clear_error()
                elif command["command"] == "start_rt_control":
                    self.control_loop_w_tool_change()
                elif command["command"] == "tool_change":
                    while True:
                        next_tool_id = self.pose[17]
                        if next_tool_id != 0:
                            try:
                                self.pose[18] = 0
                                self.tool_change(next_tool_id)
                            except ORiNException as e:
                                print("[CNT]: Error during tool change")
                                print(f"[CNT]: {self.robot.format_error(e)}")
                            finally:
                                self.pose[18] = 0
                                self.pose[17] = 0
                                break
            except Exception as e:
                self.leave_servo_mode()
                print(f"[CNT]: {self.robot.format_error(e)}")
