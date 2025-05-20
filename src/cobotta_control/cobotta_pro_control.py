# Cobotta Proを制御する

from typing import Any, Dict, List, Literal
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
from cobotta_control.tools import tool_infos, tool_classes


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
reset_default_state = True
default_joints = {
    # TCPが台の中心の上に来る初期位置
    "tidy": [4.7031, -0.6618, 105.5149, 0.0001, 75.1440, 94.7038],
    # NOTE: j5の基準がVRと実機とでずれているので補正。将来的にはVR側で修正?
    "vr": [159.3784, 10.08485, 122.90902, 151.10866, -43.20116 + 90, 20.69275],
    # NOTE: 2025/04/18 19:25の新しい位置?VRとの対応がおかしい気がする
    # 毎回の値も[0, 0, 0, 0, 0, 0]が飛んでくる気がする
    "vr2": [115.55677, 5.86272, 135.70465, 110.53529, -15.55474 + 90, 35.59977],
}
abs_joint_limit = [270, 150, 150, 270, 150, 360]
abs_joint_limit = np.array(abs_joint_limit)
abs_joint_soft_limit = abs_joint_limit - 10
# 外部速度。単位は%
speed_normal = 20
speed_tool_change = 5


save_control = SAVE
if save_control:
    save_path = datetime.datetime.now().strftime("%Y%m%d%H%M%S") + "_control.jsonl"
    f = open(save_path, "w")

class Cobotta_Pro_CON:
    def __init__(self):
        self.default_joint = default_joints["vr"]
        self.tidy_joint = default_joints["tidy"]

    def init_robot(self):
        self.robot = DensoRobot(
            host=ROBOT_IP,
            default_servo_mode=servo_mode,
        )
        self.robot.start()
        self.robot.take_arm()
        self.find_and_setup_hand()

    def find_and_setup_hand(self):
        connected = False
        for tool_info in tool_infos:
            tool_id = tool_info["id"]
            if tool_id == -1:
                continue
            name = tool_info["name"]
            hand = tool_classes[name]()
            connected = hand.connect_and_setup()
            if connected:
                print(f"Connected to hand; id: {tool_id}, name: {name}")
                self.hand_name = name
                self.hand = hand
                self.tool_id = tool_id
                self.robot.SetToolDef(
                    tool_info["id_in_robot"], tool_info["tool_def"])
                self.robot.set_tool(tool_info["id_in_robot"])
                return
        if not connected:
            print("Failed to connect to any hand")
            tool_id = -1
            self.hand_name = "no_tool"
            self.hand = None
            self.tool_id = tool_id
            tool_info = self.get_tool_info(tool_id)
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

    def control_loop(self):
        self.last = 0
        print("[CNT]Start Main Loop")
        while True:
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
                # 取得する前に終了する場合即時終了可能
                if self.pose[16] == 1:
                    return True
                continue

            # 目標値を取得しているかを確認
            if self.pose[6:12].sum() == 0:
                time.sleep(t_intv)
                # print("[CNT]Wait for target..")
                # 取得する前に終了する場合即時終了可能
                if self.pose[16] == 1:
                    return True
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

            # リアルタイム制御を止める場合は入ってくる目標値を同じにして
            # ロボットを静止させる
            if self.pose[16] == 1:
                target = self.last_target
            else:
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

            # target_delayedは、delay秒前の目標値を前後の値を
            # 使って線形補間したもの
            if use_interp:
                target_delayed = di.read(now, target)
            else:
                target_delayed = target
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
                t_elapsed = time.time() - now
                t_wait = t_intv - t_elapsed
                if t_wait > 0:
                    time.sleep(t_wait)

            if self.pose[16] == 1:
                # スレーブモードでは十分低速時に2回同じ位置のコマンドを送ると
                # ロボットを停止させてスレーブモードを解除可能な状態になる
                if (control == self.last_control).all():
                    return True
                
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
        # TODO: ダミー
        elif self.hand_name == "robotiq_epick":
            self.hand.grip(waiting=False)

    def send_release(self):
        if self.tool_id == -1:
            return
        if self.hand_name == "onrobot_2fg7":
            # NOTE: 呼ぶ度に目標の把持力は変更できるので
            # VRコントローラーからの入力で動的に把持力を
            # 変えることもできる (どういう仕組みを作るかは別)
            self.hand.release(waiting=False)
        # TODO: ダミー
        elif self.hand_name == "robotiq_epick":
            self.hand.release(waiting=False)

    def enable(self) -> bool:
        self.robot.enable_robot()

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
            successfully_stopped = self.control_loop()
            self.leave_servo_mode()
            if successfully_stopped:
                print("[CNT]: User required stop and successfully done")
                break
            else:
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
                self.tool_change(next_tool_id)
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
        diff = [0, 0, 100, 0, 0, 0]
        up_pose = (np.asarray(current_pose) + np.asarray(diff)).tolist()
        self.robot.move_pose(up_pose)
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
            self.robot.ext_speed(speed_tool_change)
            self.robot.move_pose(wps["disengaged"])
            self.robot.move_pose(wps["tool_holder"])
            self.robot.move_pose(wps["locked"])
            time.sleep(1)
            name = next_tool_info["name"]
            hand = tool_classes[name]()
            connected = hand.connect_and_setup()
            # NOTE: 接続できなければ止めたほうが良いと考える
            if not connected:
                raise ValueError("Failed to connect to any hand")
            self.pose[18] = 5
            while True:
                if self.pose[18] == 6:
                    break
                time.sleep(0.008)
            self.robot.move_pose(wps["exit_path"])
            self.robot.ext_speed(speed_normal)
        # 現在ツールが付いているとき
        else:
            wps = tool_info["holder_waypoints"]
            self.robot.move_pose(wps["exit_path"])
            self.robot.ext_speed(speed_tool_change)
            self.robot.move_pose(wps["locked"])
            self.robot.move_pose(wps["tool_holder"])
            self.robot.move_pose(wps["disengaged"])
            time.sleep(1)
            if next_tool_info["id"] == -1:
                self.pose[18] = 5
                while True:
                    if self.pose[18] == 6:
                        break
                    time.sleep(0.008)
                self.robot.move_pose(wps["enter_path"])
                self.robot.ext_speed(speed_normal)
            else:
                wps = next_tool_info["holder_waypoints"]
                self.robot.move_pose(wps["disengaged"])
                self.robot.move_pose(wps["tool_holder"])
                self.robot.move_pose(wps["locked"])
                time.sleep(1)
                name = next_tool_info["name"]
                hand = tool_classes[name]()
                connected = hand.connect_and_setup()
                # NOTE: 接続できなければ止めたほうが良いと考える
                if not connected:
                    raise ValueError("Failed to connect to any hand")
                self.pose[18] = 5
                while True:
                    if self.pose[18] == 6:
                        break
                    time.sleep(0.008)
                self.robot.move_pose(wps["exit_path"])
                self.robot.ext_speed(speed_normal)
                
        # 以下の移動後、ツールチェンジ前後でのTCP位置は変わらない
        # （ツールの大きさに応じてアームの先端の位置が変わる）
        self.robot.SetToolDef(
            next_tool_info["id_in_robot"], next_tool_info["tool_def"])
        self.robot.set_tool(next_tool_info["id_in_robot"])
        self.robot.move_pose(up_pose)
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
                            self.tool_change(next_tool_id)
                            self.pose[17] = 0
                            break
            except Exception as e:
                self.leave_servo_mode()
                print(f"[CNT]: {self.robot.format_error(e)}")