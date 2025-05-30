import os
import sys
from typing import Optional

package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vendor'))
sys.path.append(package_dir)

from on_robot.device import Device
from on_robot.vgc10 import VG, CONN_ERR, RET_OK, RET_FAIL


class VGC10Control:
    def __init__(
        self,
        host: str = "192.168.5.46",
    ) -> None:
        device = Device(Global_cbip=host)
        self._vgc10 = VG(device)

    def connect_and_setup(self) -> bool:
        """
        VGC10に接続します.
        """
        return self._vgc10.isConnected()

    def disconnect(self) -> None:
        """
        接続を切断します.
        """
        pass

    def grip(
        self,
        vacuumA: int = 5,
        vacuumB: int = 5,
        waiting: bool = True,
    ) -> int:
        """
        指定した真空値で吸着動作を実行します．

        Args:
            vacuumA (int): チャンネルAの真空値 (1-80 kPa)
            vacuumB (int): チャンネルBの真空値 (1-80 kPa)
            waiting (bool): 待機するか

        Returns:
            int: RET_OK, RET_FAIL, or CONN_ERR
        """
        return self._vgc10.grip(vacuumA=vacuumA, vacuumB=vacuumB, waiting=waiting)

    def release(
        self,
        channelA: bool = True,
        channelB: bool = True,
        waiting: bool = True,
    ) -> int:
        """
        指定したチャンネルを開放します．

        Args:
            channelA (bool): チャンネルAを開放
            channelB (bool): チャンネルBを開放
            waiting (bool): 真空が抜けるまで待機するか

        Returns:
            int: RET_OK, RET_FAIL, or CONN_ERR
        """
        return self._vgc10.release(channelA=channelA, channelB=channelB, waiting=waiting)

    def get_vacuum_A(self) -> Optional[float]:
        """
        チャンネルAの真空値を取得します.

        Returns:
            Optional[float]: 空気圧 or None
        """
        ret = self._vgc10.getvacA()
        if ret == CONN_ERR:
            return None
        return ret

    def get_vacuum_B(self) -> Optional[float]:
        """
        チャンネルBの真空値を取得します.

        Returns:
            Optional[float]: 真空値 or None
        """
        ret = self._vgc10.getvacB()
        if ret == CONN_ERR:
            return None
        return ret

    def idle(
        self,
        channelA: bool = True,
        channelB: bool = True,
    ) -> int:
        """
        目的のチャンネルのポンプをオフにします.影響を受けるチャンネルのバルブは
        閉じたままになります.これは、数秒まで物体を把持したままにできる低エネルギ
        ー状態です.

        Args:
            channelA (bool): チャンネルAのポンプを停止
            channelB (bool): チャンネルBのポンプを停止

        Returns:
            int: RET_OK, RET_FAIL, or CONN_ERR
        """
        return self._vgc10.idle(channelA=channelA, channelB=channelB)