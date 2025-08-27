from functools import partial
import re
import sys
import os
import glob
import json
import pandas as pd  # データフレーム操作・時系列変換用
from PyQt6 import QtWidgets, QtCore, QtGui  # PyQt6: GUI構築用

# pyqtgraph: 高速・インタラクティブなプロットライブラリ
import pyqtgraph as pg
from pyqtgraph import AxisItem
import numpy as np  # 数値配列処理用

class LogLoader:
    """
    ログファイルの読み込み・パースを担当するクラス。
    state.jsonl, control.jsonl, log.txt をDataFrameやリストで返す。
    """
    def __init__(self, log_dir):
        self.log_dir = log_dir

    def list_dates(self):
        # log/配下の日付ディレクトリ一覧を返す
        return sorted([d for d in os.listdir(self.log_dir) if os.path.isdir(os.path.join(self.log_dir, d))])

    def list_times(self, date):
        # log/日付/配下の時刻ディレクトリ一覧を返す
        date_dir = os.path.join(self.log_dir, date)
        return sorted([d for d in os.listdir(date_dir) if os.path.isdir(os.path.join(date_dir, d))])

    def load_state(self, date, time) -> dict[str, pd.DataFrame]:
        # state.jsonlを1行1レコードのJSON Linesとして読み込み、DataFrame化
        path = os.path.join(self.log_dir, date, time, 'state.jsonl')
        if not os.path.exists(path):
            return {}
        records = []
        with open(path) as f:
            for line in f:
                js = json.loads(line)
                data = {}
                data["time"] = js["time"]
                data["kind"] = js["kind"]
                for i in range(6):
                    data[f"J{i+1}"] = js["joint"][i]
                pose_names = ["X", "Y", "Z", "RX", "RY", "RZ"]
                for i in range(6):
                    data[pose_names[i]] = js["pose"][i]
                records.append(data)
        df = pd.DataFrame(records)
        ret = {}
        for kind in df["kind"].unique():
            ret[kind] = df[df["kind"] == kind]
        return ret

    def load_control(self, date, time) -> dict[str, pd.DataFrame]:
        # control.jsonlも同様にDataFrame化
        path = os.path.join(self.log_dir, date, time, 'control.jsonl')
        if not os.path.exists(path):
            return {}
        records = []
        with open(path) as f:
            for line in f:
                js = json.loads(line)
                data = {}
                data["time"] = js["time"]
                data["kind"] = js["kind"]
                for i in range(6):
                    data[f"J{i+1}"] = js["joint"][i]
                data["max_ratio"] = js.get("max_ratio", None)
                data["accel_max_ratio"] = js.get("accel_max_ratio", None)
                records.append(data)
        df = pd.DataFrame(records)
        ret = {}
        for kind in df["kind"].unique():
            ret[kind] = df[df["kind"] == kind]
        return ret

    def load_events(self, date, time):
        # log.txtを1行ずつリストで返す
        path = os.path.join(self.log_dir, date, time, 'log.txt')
        if not os.path.exists(path):
            return None
        events = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                # [日時][モジュール][レベル] メッセージ
                match = re.match(r'\[(.*?)\]\[(.*?)\]\[(.*?)\] (.*)', line)
                if match:
                    dt_str, module, level, message = match.groups()
                    events.append({
                        'time': pd.to_datetime(dt_str).tz_localize('Asia/Tokyo').timestamp(),
                        'module': module,
                        'level': level,
                        'message': message,
                        'original_message': line,
                        'is_event': 1,
                    })
        return pd.DataFrame(events)

class LogViewer(QtWidgets.QWidget):
    def __init__(self, log_dir):
        super().__init__()
        self.loader = LogLoader(log_dir)
        self.init_ui()
        # ウィンドウのデフォルトサイズを大きくする（8行1列のプロットが見やすいように）
        self.resize(800, 1000)

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()
        # 日付・時刻選択UI
        date_label = QtWidgets.QLabel('日付:')
        self.date_combo = QtWidgets.QComboBox()
        self.date_combo.addItems(self.loader.list_dates())  # ログ日付一覧
        self.date_combo.currentTextChanged.connect(self.update_times)
        time_label = QtWidgets.QLabel('時刻:')
        self.time_combo = QtWidgets.QComboBox()
        self.time_combo.currentTextChanged.connect(self.load_and_plot)
        hlayout = QtWidgets.QHBoxLayout()
        hlayout.addWidget(date_label)
        hlayout.addWidget(self.date_combo)
        hlayout.addWidget(time_label)
        hlayout.addWidget(self.time_combo)
        layout.addLayout(hlayout)
        # スケールリセットボタン
        self.reset_btn = QtWidgets.QPushButton('スケールリセット')
        self.reset_btn.clicked.connect(self.reset_view)
        layout.addWidget(self.reset_btn)
        # プロットエリア
        self.combos = []
        self.plots = []
        for _ in range(8):
            row_layout = QtWidgets.QHBoxLayout()
            combo = QtWidgets.QComboBox()
            plot = pg.PlotWidget()
            row_layout.addWidget(combo)
            row_layout.addWidget(plot, stretch=1)
            layout.addLayout(row_layout)
            self.combos.append(combo)
            self.plots.append(plot)
        self.setLayout(layout)
        self.update_times(self.date_combo.currentText())

    def update_times(self, date):
        # 日付が選択されたとき、時刻リストを更新
        self.time_combo.clear()
        self.time_combo.addItems(self.loader.list_times(date))
        if self.time_combo.count() > 0:
            self.load_and_plot(self.time_combo.currentText())

    def load_and_plot(self, time):
        # 日付・時刻で指定されたログファイルを読み込み、8行1列のプロットを生成
        date = self.date_combo.currentText()
        time = self.time_combo.currentText()
        state_dfs = self.loader.load_state(date, time)
        control_dfs = self.loader.load_control(date, time)
        self.target_df = control_dfs.get("target")
        self.control_df = control_dfs.get("control")
        self.state_df = state_dfs.get("state")
        self.event_df = self.loader.load_events(date, time)
        self.all_items = \
            ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'X', 'Y', 'Z', 'RX', 'RY', 'RZ', 'Event']
        self.default_items = \
            ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'X', 'Event']

        # 既存のPlotWidget/QComboBoxをクリア
        for combo, plot in zip(self.combos, self.plots):
            plot.clear()
            combo.clear()

        # x軸を日時表示にするAxisItem
        class DateAxisItem(pg.AxisItem):
            def tickStrings(self, values, scale, spacing):
                # spacing: tick間隔（秒）
                labels = []
                # 1日=86400秒, 1時間=3600秒
                for i, v in enumerate(values):
                    dt = pd.to_datetime(float(v), unit='s', utc=True).tz_convert('Asia/Tokyo')
                    if spacing >= 86400:  # 1日以上の間隔なら日付のみ
                        label = dt.strftime('%Y-%m-%d')
                    elif spacing >= 3600:  # 1時間以上なら時刻（最左だけ日付＋時刻）
                        if i == 0:
                            label = dt.strftime('%Y-%m-%d %H:%M')
                        else:
                            label = dt.strftime('%H:%M')
                    elif spacing >= 60:  # 1分以上なら時:分
                        if i == 0:
                            label = dt.strftime('%Y-%m-%d %H:%M')
                        else:
                            label = dt.strftime('%H:%M')
                    else:  # 1分未満なら時:分:秒
                        if i == 0:
                            label = dt.strftime('%Y-%m-%d %H:%M:%S')
                        else:
                            label = dt.strftime('%H:%M:%S')
                    labels.append(label)
                return labels

        # プロット作成
        class FixedDigitsAxisItem(pg.AxisItem):
            def tickStrings(self, values, scale, spacing):
                # 指数表記で全体6桁程度に揃える（例: 1.23e+03）
                return [f"{v:+8.2e}" for v in values]

        self.proxies = []
        for i in range(8):
            combo = self.combos[i]
            p = self.plots[i]
            axis_bottom = DateAxisItem(orientation='bottom')
            axis_left = FixedDigitsAxisItem(orientation='left')
            p.setAxisItems({'bottom': axis_bottom, 'left': axis_left})
            p.setBackground('w')
            p.showGrid(x=True, y=True, alpha=0.3)
            # 軸・ラベル・グリッド・テキストを黒に
            p.getAxis('left').setPen(pg.mkPen('k'))
            p.getAxis('bottom').setPen(pg.mkPen('k'))
            p.getAxis('left').setTextPen(pg.mkPen('k'))
            p.getAxis('bottom').setTextPen(pg.mkPen('k'))
            p.addLegend(colCount=3, offset=(0, 1), anchor=(1, 0))
            p.setXLink(self.plots[0])            
            proxy = pg.SignalProxy(
                p.scene().sigMouseMoved,
                rateLimit=60,
                slot=partial(self.on_hover, i),
            )
            self.proxies.append(proxy)
            item = self.default_items[i]
            combo.clear()
            combo.addItems(self.all_items)
            i = self.all_items.index(item)
            combo.setCurrentIndex(i)
            combo.currentTextChanged.connect(self.update_curve)
            # y軸ラベルの幅は自動調整・小さめフォント
            axis_left.setStyle(autoExpandTextSpace=True)
            axis_left.setTickFont(QtGui.QFont("IPAGothic", 8))
            # x軸ラベルの重なり防止: フォントサイズを小さく、autoExpandTextSpace有効化
            axis_bottom.setStyle(autoExpandTextSpace=True)
            axis_bottom.setTickFont(QtGui.QFont("IPAGothic", 8))
        self.reset_view()

    def update_curve(self):
        # リストボックスで選択された要素に応じてプロットを更新
        combo = self.sender()
        if combo is None:
            return
        idx = self.combos.index(combo)
        item = combo.currentText()
        p = self.plots[idx]
        # 既存のプロットをクリア
        p.clear()
        self.plot(item, p)

    def plot(self, item, p):
        if item == 'Event':
            if self.event_df is not None:
                # Event選択時はバーコード型（縦線）プロット
                ts = self.event_df["time"].values
                ls = self.event_df["level"].values
                l2c = {
                    'DEBUG': 'black',
                    'INFO': 'black',
                    'WARNING': 'orange',
                    'ERROR': 'red',
                    'CRITICAL': 'red',
                }
                for t, l in zip(ts, ls):
                    line = pg.InfiniteLine(
                        pos=t,
                        angle=90,
                        pen=pg.mkPen(l2c.get(l, 'k'), width=1))
                    p.addItem(line)
                # y軸範囲を0-1に固定
                p.setYRange(0, 1)
        else:
            if self.target_df is not None:
                if item in self.target_df.columns:
                    ts = self.target_df["time"].values
                    y = self.target_df[item].values
                else:
                    ts = []
                    y = []
                p.plot(ts, y, pen=pg.mkPen('r', width=2), name='target')
            if self.control_df is not None:
                if item in self.control_df.columns:
                    ts = self.control_df["time"].values
                    y = self.control_df[item].values
                else:
                    ts = []
                    y = []
                p.plot(ts, y, pen=pg.mkPen('g', width=2), name='control')
            if self.state_df is not None:
                if item in self.state_df.columns:
                    ts = self.state_df["time"].values
                    y = self.state_df[item].values
                else:
                    ts = []
                    y = []
                p.plot(ts, y, pen=pg.mkPen('b', width=2), name='state')

    def reset_view(self):
        # スケールリセットボタン押下時、全パネルの表示範囲を初期化
        for p in self.plots:
            p.enableAutoRange(axis=pg.ViewBox.XYAxes)

    def on_hover(self, i, evt):
        # イベント有無パネル上でマウスを動かしたとき、該当時刻のイベント内容をポップアップ表示
        pos = evt[0]
        vb = self.plots[i].getViewBox()
        if vb.sceneBoundingRect().contains(pos):
            mouse_point = vb.mapSceneToView(pos)
            x = mouse_point.x()
            # 最近傍のイベントを探す
            if self.event_df is None or len(self.event_df) == 0:
                return
            idx = (abs(self.event_df['time'] - x)).argmin()
            row = self.event_df.iloc[idx]
            QtWidgets.QToolTip.showText(QtGui.QCursor.pos(), row['original_message'])


if __name__ == '__main__':
    from PyQt6.QtGui import QFont
    # 日本語対応フォント名（環境に応じて変更可）
    # 例: "IPAGothic", "Noto Sans CJK JP", "VL Gothic" など
    jp_font = QFont("IPAGothic")
    jp_font.setPointSize(10)
    QtWidgets.QApplication.setFont(jp_font)
    app = QtWidgets.QApplication(sys.argv)
    log_dir = 'log'
    win = LogViewer(log_dir)
    win.setWindowTitle('Log Viewer')
    win.show()
    sys.exit(app.exec())
