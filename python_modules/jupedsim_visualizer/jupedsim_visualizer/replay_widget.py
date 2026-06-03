# SPDX-License-Identifier: LGPL-3.0-or-later
from jupedsim import RoutingEngine
from jupedsim.recording import Recording
from PySide6.QtCore import QSignalBlocker, QTimer
from PySide6.QtGui import QPaintEvent
from PySide6.QtWidgets import (
    QVBoxLayout,
    QWidget,
)

from jupedsim_visualizer.geometry import Geometry
from jupedsim_visualizer.geometry_widget import RenderWidget
from jupedsim_visualizer.player_controls import PlayerControlWidget
from jupedsim_visualizer.trajectory import Trajectory


class ReplayWidget(QWidget):
    def __init__(
        self,
        navi: RoutingEngine,
        rec: Recording,
        geo: Geometry,
        trajectory: Trajectory,
        parent=None,
    ):
        QWidget.__init__(self, parent)
        self.rec = rec
        self.trajectory = trajectory
        self.control = PlayerControlWidget(parent=self)
        self.render_widget = RenderWidget(
            geo, navi, [geo, trajectory], parent=self
        )
        self.geo = geo
        layout = QVBoxLayout()
        layout.addWidget(self.render_widget, 1)
        layout.addWidget(self.control)
        self.setLayout(layout)
        self.control.play.toggled.connect(self.play)
        self.control.forward.clicked.connect(self.frame_forward)
        self.control.backward.clicked.connect(self.frame_backward)
        self.control.slider.setMaximum(self.rec.num_frames - 1)
        self.control.slider.valueChanged.connect(self.goto_frame)
        self.control.begin.clicked.connect(lambda: self.goto_frame(0))
        self.control.end.clicked.connect(
            lambda: self.goto_frame(self.trajectory.num_frames - 1)
        )

    def frame_forward(self):
        self.trajectory.advance_frame(self.control.speed_selector.value())
        self.control.update_replay_time(
            self.trajectory.current_index * (1 / self.rec.fps)
        )
        self.render_widget.render()
        with QSignalBlocker(self.control.slider):
            self.control.slider.setValue(self.trajectory.current_index)

    def frame_backward(self):
        self.trajectory.advance_frame(-self.control.speed_selector.value())
        self.control.update_replay_time(
            self.trajectory.current_index * (1 / self.rec.fps)
        )
        self.render_widget.render()
        with QSignalBlocker(self.control.slider):
            self.control.slider.setValue(self.trajectory.current_index)

    def goto_frame(self, index: int):
        self.trajectory.goto_frame(index)
        self.control.update_replay_time(
            self.trajectory.current_index * (1 / self.rec.fps)
        )
        self.render_widget.render()
        with QSignalBlocker(self.control.slider):
            self.control.slider.setValue(self.trajectory.current_index)

    def play(self, checked: bool):
        if checked:
            self.timer = QTimer()
            self.timer.setInterval(int(1000.0 / self.rec.fps))
            self.timer.timeout.connect(self.frame_forward)
            self.timer.start()
        else:
            if self.timer:
                self.timer.stop()

    def render(self):
        self.render_widget.render()

    def paintEvent(self, event: QPaintEvent) -> None:
        self.render()
        return super().paintEvent(event)
