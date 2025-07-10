import multiprocessing.shared_memory as sm
import sys

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from config import (
    ABS_JOINT_LIMIT,
    SHM_NAME,
    SHM_SIZE,
    T_INTV,
)

class JointMonitorPlot(QtWidgets.QWidget):
    def __init__(
        self,
        max_points=500,
    ):
        super().__init__()
        self.n_joints = len(ABS_JOINT_LIMIT)
        self.max_points = max_points
        self.t_intv = T_INTV
        self.t = 0
        self.xdata = []
        self.ydata = {
            'state': [[] for _ in range(self.n_joints)],
            'target': [[] for _ in range(self.n_joints)],
            'control': [[] for _ in range(self.n_joints)],
        }
        self.shm = sm.SharedMemory(SHM_NAME)
        self.pose = np.ndarray(
            (SHM_SIZE,), dtype=np.float32, buffer=self.shm.buf)

        self.plots = []
        self.curves = []
        layout = QtWidgets.QVBoxLayout()
        for i in range(self.n_joints):
            plot = pg.PlotWidget(title=f"Joint {i+1}")
            if i == 0:
                plot.addLegend()
            curve_state = plot.plot(pen="b", name="state")
            curve_target = plot.plot(pen="r", name="target")
            curve_control = plot.plot(pen="g", name="control")
            self.plots.append(plot)
            self.curves.append({
                "state": curve_state,
                "target": curve_target,
                "control": curve_control
            })
            layout.addWidget(plot)
        self.setLayout(layout)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(self.t_intv * 1000))

    def update_plot(self):
        self.t += self.t_intv
        self.xdata.append(self.t)
        for i in range(self.n_joints):
            self.ydata['state'][i].append(float(self.pose[i]))
            self.ydata['target'][i].append(float(self.pose[i+6]))
            self.ydata['control'][i].append(float(self.pose[i+24]))
        if len(self.xdata) > self.max_points:
            self.xdata = self.xdata[-self.max_points:]
            for k in self.ydata:
                for i in range(self.n_joints):
                    self.ydata[k][i] = self.ydata[k][i][-self.max_points:]
        for i in range(self.n_joints):
            for k in self.ydata:
                self.curves[i][k].setData(
                    np.array(self.xdata), np.array(self.ydata[k][i]))


def run_joint_monitor_gui():
    app = QtWidgets.QApplication(sys.argv)
    win = JointMonitorPlot()
    win.setWindowTitle('Cobotta Pro Joint Monitor')
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    run_joint_monitor_gui()
