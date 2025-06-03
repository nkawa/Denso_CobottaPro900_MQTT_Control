import os
import sys
from typing import Optional

package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
sys.path.append(package_dir)

from on_robot.device import Device
from on_robot.twofg import TWOFG, CONN_ERR, RET_OK, RET_FAIL


class TWOFGControl:
    def __init__(
        self,
        host: str = "192.168.5.46",
    ) -> None:
        device = Device(Global_cbip=host)
        self._twofg = TWOFG(device)

    def connect_and_setup(self) -> bool:
        """
        接続してハンドのパラメータを取得します.
        """
        if not self._twofg.isConnected():
            return False
        ret = self._twofg.get_min_ext_width()
        if ret == CONN_ERR:
            return False
        self._default_grip_width = ret
        ret = self._twofg.get_max_ext_width()
        if ret == CONN_ERR:
            return False
        self._default_release_width = ret
        return True
    
    def disconnect(self) -> None:
        """
        接続を切断します
        """
        pass

    def grip(
        self,
        width: Optional[float] = None,
        force: int = 20,
        speed: int = 10,
        waiting: int = 1,
    ) -> None:
        """
        指定した設定で把持動作を実行します．

        TwofgGrip(<fWidth>, <lForce>, <lSpeed>, <lWaiting>)
        <fWidth> ： 目標となフィンガー間の幅[mm]（VT_R4）
        <lForce> ： ワークを把持する力[N]（VT_I4）
        <lSpeed> ： フィンガーの開閉速度[%]（VT_I4）
        <lWaiting> ： 待機設定（VT_I4）
                        0：待機しない
                        1：目標ワーク幅に達するまで待機する

        OR_2FG_GRIP(instance, width, force, speed, waiting)
        width シングル 現在のモードで適切な幅を [mm]単位、小数点精度 1 で定
        義します。
        force 整数 適切な把持力を [N]単位で定義します。有効な範囲は 20
        ～140N です。
        speed 整数 適切な把持速度を [%]単位で定義します。有効な範囲は
        10～100%です。
        waiting 整数
        0：プログラムはグリッパーが動作中も続行されます。
        1：プログラムはグリッパーが動作を終了するまで待って
        から把持のステータスをチェックします。把持が発生し
        ていなかった場合、プログラムは停止します。
        """
        if width is None:
            width = self._default_grip_width
        self._twofg.grip(
            t_width=width, n_force=force, p_speed=speed, f_wait=waiting,
        )
        
    def release(
        self,
        width: Optional[float] = None,
        waiting: int = 1,
    ) -> None:
        """
        指定したワーク幅まで開放動作を実行します．

        TwofgRelease(<fWidth>, <lWaiting>)
        <fWidth> ： 目標となるフィンガー間の距離[mm]（VT_R4）
        <lWaiting> ： 待機設定（VT_I4）
                        0：待機しない
                        1：目標ワーク幅に達するまで待機する

        OR_2FG_RELEASE(instance, width, waiting)
        width シングル 現在のセットアップで適切な幅を [mm]単位、小数点精度
        1 で定義します。
        waiting 整数
        0：プログラムはグリッパーが動作中も続行されます。
        1：プログラムはグリッパーが動作を停止するまで待ちま
        す。

        widthがNoneの場合、把持モードに応じて最大まで開放する。
        """
        if width is None:
            width = self._default_release_width
        self._twofg.move(
            t_width=width, f_wait=waiting,
        )

    def get_ext_width(self) -> Optional[int]:
        """
        フィンガー間の幅を取得します．
        """
        ret = self._twofg.get_ext_width()
        if ret == CONN_ERR:
            return None
        return ret
    
    def get_force(self) -> Optional[float]:
        """
        把持力を取得します．
        """
        ret = self._twofg.get_force()
        if ret == CONN_ERR:
            return None
        return ret

    def stop(
        self,
    ) -> None:
        """
        動作を停止させます．
        """
        self._twofg.stop()
